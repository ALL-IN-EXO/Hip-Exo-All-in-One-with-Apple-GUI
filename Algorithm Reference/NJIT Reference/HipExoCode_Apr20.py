"""
Biodynamics Lab Hip Exoskeleton Control Script
================================================
Based on: HipExoCode_Feb24.py
Updated: 2025-04-20

Changes from Feb24:
  - Added LPMS IMU sensor support as an alternative to motor encoder sensing
  - Only 3 LPMS sensors used: Pelvis (IMU#1), Right Femur (IMU#2), Left Femur (IMU#4)
  - Hip joint angle = pelvis_angle - femur_angle (in degrees, then converted to radians)
  - Added USE_LPMS global flag and runtime switching via UDP commands
  - New UDP command: SET_SOURCE LPMS / SET_SOURCE ENCODER
  - New UDP command: CALIBRATE_IMU (sets zero offset from standing still)
  - Sensor reading abstracted into read_joint_states() – all mode functions use this
  - LPMS data acquisition runs in a background thread (hip_data_acquisition)
  - Encoder data is always read (for logging + motor torque apply) even when USE_LPMS=True

IMPORTANT: LPMS sign conventions (LPMS_LH_SIGN / LPMS_RH_SIGN) must be verified
against physical hardware before deployment. Defaults assume the same mounting
orientation as used in HipKneeExo with LPMS.

License:
  CC BY-NC
"""

import time
import socket
import struct
import threading
from collections import deque
from datetime import datetime
import math
import torch
import torch.nn as nn
import os
import zipfile
import io
from myactuator_rmd_py import Driver, ProtocolException, can
import numpy as np
from real_filter import RealFilter, OneEuroFilter
from scipy import signal as sp_signal
import subprocess

# LPMS imports (openzen + data classes from LPMS_Module)
try:
    import openzen
    from LPMS_Module import IMUData, AngleCorrector, stop_sensors as lpms_stop_sensors
    LPMS_AVAILABLE = True
except ImportError as _e:
    LPMS_AVAILABLE = False
    print(f"[WARN] LPMS not available: {_e}. USE_LPMS will be forced False.")

# ─────────────────────────────────────────────
# Hip LPMS Sensor Configuration (3 sensors only)
# ─────────────────────────────────────────────
LPMS_PELVIS_MAC  = "00:04:3E:86:26:FB"   # IMU #1 → Pelvis
LPMS_R_FEMUR_MAC = "00:04:3E:86:27:03"   # IMU #2 → Right Femur
LPMS_L_FEMUR_MAC = "00:04:3E:86:27:27"   # IMU #4 → Left Femur  (skip IMU #3)

# Sign correction applied to LPMS joint angles before feeding into models.
# +1 = use LPMS angle as-is; -1 = negate.
# NOTE: These default to +1 (flexion-positive, consistent with HipKneeExo/LPMS convention).
#       For legacy models that were trained with extension-positive left hip, set
#       LPMS_LH_SIGN = -1 and LPMS_RH_SIGN = -1 if needed after physical verification.
LPMS_LH_SIGN = -1.0  # left hip  (negate: LPMS pelvis-femur is extension-positive)
LPMS_RH_SIGN = -1.0  # right hip (negate: LPMS pelvis-femur is extension-positive)

# ─────────────────────────────────────────────
# HipOnlySensorDataManager
# ─────────────────────────────────────────────
class HipOnlySensorDataManager:
    """
    Manages 3 LPMS IMU sensors for a hip-only exoskeleton.
    Sensor order (matches imus list from connect_hip_sensors):
      0 → Pelvis
      1 → Right Femur
      2 → Left Femur

    Hip joint angle (degrees) = pelvis_angle - femur_angle
    Result is then converted to radians.
    """

    def __init__(self, velocity_type="central_difference"):
        if not LPMS_AVAILABLE:
            raise RuntimeError("openzen / LPMS_Module not available")

        self.velocity_type = velocity_type
        corrector = AngleCorrector()

        # IMUData objects (sensor_id used only for AngleCorrector dict key)
        self.pelvis      = IMUData("Pelvis",      0, corrector)
        self.right_femur = IMUData("Right_Femur", 1, corrector)
        self.left_femur  = IMUData("Left_Femur",  4, corrector)  # physical IMU #4

        # Calibration offsets in degrees
        self.offsets = {
            'pelvis':       0.0,
            'right_femur':  0.0,
            'left_femur':   0.0,
        }

        self.connected    = False
        self.sensor_count = 0
        print("Hip-only LPMS Data Manager initialized "
              f"(velocity_type={velocity_type})")

    # ------------------------------------------------------------------
    def get_hip_joint_states(self):
        """
        Returns (lh_rad, rh_rad, lhv_rad_s, rhv_rad_s) using latest IMU data,
        or None if any sensor data is not yet available.
        """
        p  = self.pelvis.get_latest()
        rf = self.right_femur.get_latest()
        lf = self.left_femur.get_latest()

        if p is None or rf is None or lf is None:
            return None

        # Apply calibration offsets (degrees)
        p_ang  = p['angle']  - self.offsets['pelvis']
        rf_ang = rf['angle'] - self.offsets['right_femur']
        lf_ang = lf['angle'] - self.offsets['left_femur']

        # Joint angles: proximal − distal (degrees)
        rh_deg = p_ang - rf_ang
        lh_deg = p_ang - lf_ang

        # Velocity selection
        vel_key = ('velocity_gyro'
                   if self.velocity_type == 'gyroscope'
                   else 'velocity_central')
        rhv_deg = p[vel_key] - rf[vel_key]
        lhv_deg = p[vel_key] - lf[vel_key]

        # Convert to radians, apply sign correction for model convention
        lh  = LPMS_LH_SIGN * math.radians(lh_deg)
        rh  = LPMS_RH_SIGN * math.radians(rh_deg)
        lhv = LPMS_LH_SIGN * math.radians(lhv_deg)
        rhv = LPMS_RH_SIGN * math.radians(rhv_deg)

        # Clamp velocity to physiologically realistic range.
        # Spikes (up to ±57 rad/s observed) are caused by OpenZen event-queue
        # backlog making dt very small in the central-difference calculation.
        _MAX_VEL = 5.0   # rad/s  (~286 deg/s, well above normal walking)
        lhv = max(-_MAX_VEL, min(_MAX_VEL, lhv))
        rhv = max(-_MAX_VEL, min(_MAX_VEL, rhv))

        return lh, rh, lhv, rhv

    # ------------------------------------------------------------------
    def calibrate_offsets(self, duration=5.0, sample_interval=0.02):
        """
        Average each sensor's angle over `duration` seconds as the standing
        reference. Stores results in self.offsets so that subsequent
        get_hip_joint_states() returns ~0 at the calibration posture.
        """
        print(f"[LPMS] Calibrating hip offsets for {duration:.1f} s "
              "(stand still, neutral posture)…")
        sums   = {'pelvis': 0.0, 'right_femur': 0.0, 'left_femur': 0.0}
        counts = {k: 0 for k in sums}

        t0 = time.time()
        while time.time() - t0 < duration:
            for key, sensor in [('pelvis',      self.pelvis),
                                 ('right_femur', self.right_femur),
                                 ('left_femur',  self.left_femur)]:
                d = sensor.get_latest()
                if d:
                    sums[key]   += d['angle']
                    counts[key] += 1
            time.sleep(sample_interval)

        for k in sums:
            self.offsets[k] = (sums[k] / counts[k]) if counts[k] > 0 else 0.0

        lh_off = self.offsets['pelvis'] - self.offsets['left_femur']
        rh_off = self.offsets['pelvis'] - self.offsets['right_femur']
        print(f"[LPMS] Calibration done:  LH joint offset = {lh_off:.2f}°, "
              f"RH joint offset = {rh_off:.2f}°")
        print(f"[LPMS] Sensor offsets: Pelvis={self.offsets['pelvis']:.2f}°, "
              f"RF={self.offsets['right_femur']:.2f}°, "
              f"LF={self.offsets['left_femur']:.2f}°")
        return self.offsets.copy()

    def stop(self):
        """No-op; kept for interface compatibility."""
        pass


# ─────────────────────────────────────────────
# connect_hip_sensors()
# ─────────────────────────────────────────────
def connect_hip_sensors(stream_freq=100,
                        enable_raw_gyr=True,
                        enable_raw_acc=False,
                        enable_raw_mag=False,
                        enable_quat=False,
                        enable_euler=True,
                        max_retries=3):
    """
    Connect to the 3 hip LPMS sensors via Bluetooth (openzen).

    Returns:
        (client, sensors, imus)
        imus[0] = Pelvis, imus[1] = Right_Femur, imus[2] = Left_Femur
    """
    if not LPMS_AVAILABLE:
        raise RuntimeError("openzen not available – cannot connect LPMS sensors")

    openzen.set_log_level(openzen.ZenLogLevel.Warning)
    err, client = openzen.make_client()
    if err != openzen.ZenError.NoError:
        raise RuntimeError(f"OpenZen init failed: {err}")

    # Flush stale events
    while client.poll_next_event() is not None:
        pass

    mac_names = [
        (LPMS_PELVIS_MAC,  "Pelvis"),
        (LPMS_R_FEMUR_MAC, "Right_Femur"),
        (LPMS_L_FEMUR_MAC, "Left_Femur"),
    ]
    sensors, imus = [], []

    for mac, name in mac_names:
        connected = False
        for attempt in range(1, max_retries + 1):
            print(f"  Connecting {name} ({mac}) – attempt {attempt}/{max_retries}")
            e, sensor = client.obtain_sensor_by_name("Bluetooth", mac)
            if e == openzen.ZenError.NoError:
                imu = sensor.get_any_component_of_type(openzen.component_type_imu)
                sensors.append(sensor)
                imus.append(imu)
                print(f"    {name} connected.")
                connected = True
                break
            if attempt < max_retries:
                time.sleep(2)

        if not connected:
            for s in sensors:
                s.release()
            client.close()
            raise RuntimeError(f"Could not connect {name} after {max_retries} attempts")

    # Configure sensors
    for imu in imus:
        imu.set_int32_property(openzen.ZenImuProperty.SamplingRate, stream_freq)
        imu.set_bool_property(openzen.ZenImuProperty.OutputRawAcc,     enable_raw_acc)
        imu.set_bool_property(openzen.ZenImuProperty.OutputRawGyr,     enable_raw_gyr)
        imu.set_bool_property(openzen.ZenImuProperty.OutputRawMag,     enable_raw_mag)
        imu.set_bool_property(openzen.ZenImuProperty.OutputQuat,       enable_quat)
        imu.set_bool_property(openzen.ZenImuProperty.OutputEuler,      enable_euler)
        imu.set_bool_property(openzen.ZenImuProperty.OutputLinearAcc,  False)
        imu.set_bool_property(openzen.ZenImuProperty.OutputPressure,   False)
        imu.set_bool_property(openzen.ZenImuProperty.OutputTemperature, False)

    time.sleep(2)   # allow sensors to settle
    # Flush events generated during config
    while client.poll_next_event() is not None:
        pass

    print("All 3 hip LPMS sensors connected and configured.")
    return client, sensors, imus


# ─────────────────────────────────────────────
# hip_data_acquisition()  – background thread
# ─────────────────────────────────────────────
_hip_lpms_quit = False   # module-level flag; set True to stop thread


def hip_data_acquisition(client, imus, manager, start_time):
    """
    Background thread: reads openzen events and feeds them into
    HipOnlySensorDataManager.
    imus[0]=Pelvis, imus[1]=Right_Femur, imus[2]=Left_Femur
    """
    global _hip_lpms_quit
    _hip_lpms_quit = False

    sensor_targets = [
        (imus[0], manager.pelvis,      "Pelvis"),
        (imus[1], manager.right_femur, "Right_Femur"),
        (imus[2], manager.left_femur,  "Left_Femur"),
    ]

    print("[LPMS] Hip data acquisition thread started.")
    while not _hip_lpms_quit:
        try:
            zen_event = client.wait_for_next_event()
            if zen_event is None:
                time.sleep(0.005)
                continue
        except Exception as e:
            print(f"[LPMS] Event wait error: {e}")
            time.sleep(0.005)
            continue

        for imu_obj, imu_store, name in sensor_targets:
            if (zen_event.event_type == openzen.ZenEventType.ImuData
                    and zen_event.sensor == imu_obj.sensor
                    and zen_event.component.handle == imu_obj.component.handle):
                try:
                    imu_data = zen_event.data.imu_data
                    ts = imu_data.timestamp   # sensor clock → correct dt for velocity
                    imu_store.add_data(ts, imu_data.r[0], imu_data)
                except Exception as e:
                    print(f"[LPMS] {name} data error: {e}")
                break

    print("[LPMS] Hip data acquisition thread stopped.")


def stop_hip_lpms(client, sensors):
    """Stop LPMS thread and release sensors."""
    global _hip_lpms_quit
    _hip_lpms_quit = True
    time.sleep(0.1)
    try:
        for s in sensors:
            s.release()
        client.close()
        print("[LPMS] Sensors disconnected.")
    except Exception as e:
        print(f"[LPMS] Disconnect error: {e}")


# ─────────────────────────────────────────────
# Global Configuration & Constants
# (identical to HipExoCode_Feb24.py)
# ─────────────────────────────────────────────
NUM_CONTROL_MODES = 7

# Neural network model names and file mapping
NN_NAMES = [
    "walk_scaled_vel", "walk_july1", "walk_july11",
    "walk_100hz_15msdelay", "squat_nov18", "s2s",
    "testing_model2", "myoassist_model", "uchida_bio4",
    "no_assistance",
]
MODEL_FILE_MAPPING = {
    "walk_scaled_vel":        "hip_exo_vs_0.05_delay_100hz_exo_state.pt",
    "walk_july1":             "hip_exo2_exo_state_Jul1.pt",
    "walk_july11":            "hip_exo_exo_state_Jul11.pt",
    "walk_100hz_15msdelay":   "hip_exo_100ms_15ms_delay_exo_state.pt",
    "squat_nov18":            "squat_vary_speed_hip_exo_exo_state.pt",
    "s2s":                    "s2s_hip_exo_exo_state.pt",
    "testing_model2":         "testing_model2.pt",
    #"myoassist_model":        "model_1966080.zip",
    "myoassist_model":        "model_2293760.zip",
    "uchida_bio4":            "uchida_bio4_from_pow_exo_state.pt",
}
MODEL_INPUT_SIZES = {
    "walk_scaled_vel":      17,
    "walk_july1":           16,
    "walk_july11":          16,
    "walk_100hz_15msdelay": 17,
    "squat_nov18":          16,
    "s2s":                  16,
    "testing_model2":       18,
    "myoassist_model":      18,
    "uchida_bio4":          18,
}

# Control Modes
control_mode   = 6      # Default: Zero Torque Mode
nn_current_id  = 2
nn_vscale      = 1.0
nn_delay       = 0.00
pt_file        = MODEL_FILE_MAPPING[NN_NAMES[nn_current_id]]
recording_filename  = None
record_step_counter = 0

SEND_DATA         = True
RECORDING         = False
CONTROL_RATE_HZ   = 50
RECORD_RATE_HZ    = 50
RUNNING           = False
FILTER_TYPE       = None

# ── MARL-sim Butterworth filters (3 Hz velocity, 6 Hz action) ────────────────
# Coefficients are fixed; zi (filter state) is reset on each NN_MODE init.
_MARL_FS   = 50.0   # Hz — control loop rate
_b_vel, _a_vel = sp_signal.butter(2, 3.0 / (_MARL_FS / 2), btype='low')
_b_act, _a_act = sp_signal.butter(2, 6.0 / (_MARL_FS / 2), btype='low')
_zi_ang_L  = sp_signal.lfilter_zi(_b_vel, _a_vel).copy()  # angle uses same 3 Hz
_zi_ang_R  = sp_signal.lfilter_zi(_b_vel, _a_vel).copy()
_zi_vel_L  = sp_signal.lfilter_zi(_b_vel, _a_vel).copy()
_zi_vel_R  = sp_signal.lfilter_zi(_b_vel, _a_vel).copy()
_zi_act_L  = sp_signal.lfilter_zi(_b_act, _a_act).copy()
_zi_act_R  = sp_signal.lfilter_zi(_b_act, _a_act).copy()

# LPMS globals
USE_LPMS      = False
lpms_manager  = None
lpms_client   = None
lpms_sensors  = None
lpms_imus     = None
lpms_fail_count = 0

# UDP network configuration
UDP_IP_PC   = "192.168.1.21"
UDP_IP_RPI  = "192.168.1.22"
DATA_PORT    = 5005
COMMAND_PORT = 5006

# UDP Sockets
sock_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_receive.bind((UDP_IP_RPI, COMMAND_PORT))
sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# CAN interface setup
os.system('sudo ifconfig can0 down')
os.system('sudo ip link set can0 type can bitrate 1000000 restart-ms 100')
os.system('sudo ifconfig can0 up')
os.system('sudo ip link set can0 txqueuelen 1000')

# Motor IDs
L_H_ID = 1
R_H_ID = 2
L_H_driver = Driver("can0", L_H_ID)
R_H_driver = Driver("can0", R_H_ID)

# ─── Motor & Encoder Sign Conventions ─────────────────────────────────
# Left  motor: raw encoder positive = hip FLEXION
# Right motor: raw encoder positive = hip EXTENSION (face-to-face mounting)
LEFT_ENCODER_SIGN  = +1
RIGHT_ENCODER_SIGN = -1
LEFT_MOTOR_CMD_SIGN  = +1   # left motor positive = FLEXION
RIGHT_MOTOR_CMD_SIGN = -1   # right motor positive = EXTENSION → negate

TORQUE_LIMIT = 15
TORQUE_CONST = 2.6
max_torque   = 0.0

kp = 0.15
kv = 0.1
ks = 0.0

time_delay        = 0.25
sampling_interval = 0.005
buffer_capacity   = int(time_delay / sampling_interval)
motion_state_buffer = deque(maxlen=buffer_capacity)

torque_L          = 0.0
torque_R          = 0.0
last_torque_L     = 0.0
last_torque_R     = 0.0
torque_L_filtered = 0.0
torque_R_filtered = 0.0

can_loop_success_count = 0
can_loop_error_count   = 0

num_history = 4
num_state   = 4 * num_history
if nn_delay > 0:
    num_state += 1
num_action  = 2
learningStd = False

cutoff_freq   = 6.0
sample_rate   = 100.0
filter_order  = 1

alpha              = 0.3
moving_avg_window  = 3
rms_window         = 3
window_size        = 7
poly_order         = 3

min_cutoff          = 4.5
beta                = 0.01
dcutoff             = 2.0
min_cutoff_velocity = 2.0
min_cutoff_torque   = 2.5
TORQUE_SLEW_NM_PER_S = 60.0

# Data storage
time_data                = []
left_hip_angle_data      = []
right_hip_angle_data     = []
left_hip_velocity_data   = []
right_hip_velocity_data  = []
left_hip_torque_data     = []
right_hip_torque_data    = []
profiling_results        = []

start_time = time.time()

nn_initialized     = False
spring_initialized = False
imp_initialized    = False
spline_initialized = False
bio_initialized    = False

# MyoAssist history parameters
MYOASSIST_INPUT_HISTORY_FRAMES  = 2
MYOASSIST_OUTPUT_HISTORY_FRAMES = 3


# ─────────────────────────────────────────────
# Filtering
# ─────────────────────────────────────────────
def apply_filter(value, measurement_type, current_time, side):
    global _zi_ang_L, _zi_ang_R, _zi_vel_L, _zi_vel_R, _zi_act_L, _zi_act_R

    # MARL-sim: 3 Hz Butterworth for angle+velocity (state), 6 Hz for action (torque)
    if FILTER_TYPE and FILTER_TYPE.lower() == "marl_sim":
        if measurement_type == "angle":
            if side == "left":
                y, _zi_ang_L = sp_signal.lfilter(_b_vel, _a_vel, [value], zi=_zi_ang_L)
                return float(y[0])
            elif side == "right":
                y, _zi_ang_R = sp_signal.lfilter(_b_vel, _a_vel, [value], zi=_zi_ang_R)
                return float(y[0])
        elif measurement_type == "velocity":
            if side == "left":
                y, _zi_vel_L = sp_signal.lfilter(_b_vel, _a_vel, [value], zi=_zi_vel_L)
                return float(y[0])
            elif side == "right":
                y, _zi_vel_R = sp_signal.lfilter(_b_vel, _a_vel, [value], zi=_zi_vel_R)
                return float(y[0])
        elif measurement_type == "torque":
            if side == "left":
                y, _zi_act_L = sp_signal.lfilter(_b_act, _a_act, [value], zi=_zi_act_L)
                return float(y[0])
            elif side == "right":
                y, _zi_act_R = sp_signal.lfilter(_b_act, _a_act, [value], zi=_zi_act_R)
                return float(y[0])

    if measurement_type == "torque":
        if side == "left":
            return filter_obj_euro_torque_left.filter(value, current_time)
        elif side == "right":
            return filter_obj_euro_torque_right.filter(value, current_time)

    if measurement_type == "velocity" and (
            FILTER_TYPE is None or FILTER_TYPE.lower() == "none"):
        if side == "left":
            return filter_obj_euro_velocity_left_nn.filter(value, current_time)
        elif side == "right":
            return filter_obj_euro_velocity_right_nn.filter(value, current_time)

    if FILTER_TYPE is None or FILTER_TYPE.lower() == "none":
        return value

    if FILTER_TYPE.lower() == "oneeuro":
        if measurement_type == "velocity":
            if side == "left":
                return filter_obj_euro_velocity_left.filter(value, current_time)
            elif side == "right":
                return filter_obj_euro_velocity_right.filter(value, current_time)
        elif measurement_type == "torque":
            if side == "left":
                return filter_obj_euro_torque_left.filter(value, current_time)
            elif side == "right":
                return filter_obj_euro_torque_right.filter(value, current_time)

    elif FILTER_TYPE.lower() == "exp":
        global alpha
        if measurement_type == "velocity":
            if side == "left":
                return filter_obj_velocity_left.exp_filter(value, alpha)
            elif side == "right":
                return filter_obj_velocity_right.exp_filter(value, alpha)
        elif measurement_type == "torque":
            if side == "left":
                return filter_obj_torque_left.exp_filter(value, alpha)
            elif side == "right":
                return filter_obj_torque_right.exp_filter(value, alpha)

    elif FILTER_TYPE.lower() == "butterworth":
        if measurement_type == "velocity":
            if side == "left":
                return filter_obj_velocity_left.butter_filter_realtime(value)
            elif side == "right":
                return filter_obj_velocity_right.butter_filter_realtime(value)
        elif measurement_type == "torque":
            if side == "left":
                return filter_obj_torque_left.butter_filter_realtime(value)
            elif side == "right":
                return filter_obj_torque_right.butter_filter_realtime(value)

    elif FILTER_TYPE.lower() == "moving":
        if measurement_type == "velocity":
            if side == "left":
                return filter_obj_velocity_left.moving_average_filter(
                    value, moving_avg_window)
            elif side == "right":
                return filter_obj_velocity_right.moving_average_filter(
                    value, moving_avg_window)
        elif measurement_type == "torque":
            if side == "left":
                return filter_obj_torque_left.moving_average_filter(
                    value, moving_avg_window)
            elif side == "right":
                return filter_obj_torque_right.moving_average_filter(
                    value, moving_avg_window)

    elif FILTER_TYPE.lower() == "rms":
        if measurement_type == "velocity":
            if side == "left":
                return filter_obj_velocity_left.rms_filter(value, rms_window)
            elif side == "right":
                return filter_obj_velocity_right.rms_filter(value, rms_window)
        elif measurement_type == "torque":
            if side == "left":
                return filter_obj_torque_left.rms_filter(value, rms_window)
            elif side == "right":
                return filter_obj_torque_right.rms_filter(value, rms_window)

    elif FILTER_TYPE.lower() == "kalman":
        if measurement_type == "velocity":
            if side == "left":
                return filter_obj_velocity_left.kalman_filter(value)
            elif side == "right":
                return filter_obj_velocity_right.kalman_filter(value)
        elif measurement_type == "torque":
            if side == "left":
                return filter_obj_torque_left.kalman_filter(value)
            elif side == "right":
                return filter_obj_torque_right.kalman_filter(value)

    elif FILTER_TYPE.lower() == "savgol":
        if measurement_type == "velocity":
            if side == "left":
                return filter_obj_velocity_left.savgol_filter_realtime(
                    value, window_size, poly_order)
            elif side == "right":
                return filter_obj_velocity_right.savgol_filter_realtime(
                    value, window_size, poly_order)
        elif measurement_type == "torque":
            if side == "left":
                return filter_obj_torque_left.savgol_filter_realtime(
                    value, window_size, poly_order)
            elif side == "right":
                return filter_obj_torque_right.savgol_filter_realtime(
                    value, window_size, poly_order)

    return value


# ─────────────────────────────────────────────
# NN Model Classes (identical to Feb24)
# ─────────────────────────────────────────────
def convert_numpy_to_torch_tensor(numpy_state_dict):
    return {k: torch.tensor(v) for k, v in numpy_state_dict.items()}


def weights_init(m):
    if m.__class__.__name__.find('Linear') != -1:
        torch.nn.init.xavier_uniform_(m.weight)
        m.bias.data.zero_()


class ExoNN(nn.Module):
    def __init__(self, num_states, num_actions, learningStd=False, is_cpu=True):
        nn.Module.__init__(self)
        self.is_cpu = is_cpu
        if not self.is_cpu and not torch.cuda.is_available():
            self.is_cpu = True
        self.num_states  = num_states
        self.num_actions = num_actions
        self.num_h1 = self.num_h2 = self.num_h3 = 64
        init_log_std = 1.0 * torch.ones(num_actions)
        if learningStd:
            self.log_std = nn.Parameter(init_log_std)
        else:
            self.log_std = init_log_std
        self.p_fc = nn.Sequential(
            nn.Linear(self.num_states, self.num_h1), nn.ReLU(inplace=True),
            nn.Linear(self.num_h1,    self.num_h2), nn.ReLU(inplace=True),
            nn.Linear(self.num_h2,    self.num_h3), nn.ReLU(inplace=True),
            nn.Linear(self.num_h3,    self.num_actions),
        )
        self.reset()
        if torch.cuda.is_available() and not self.is_cpu:
            if not learningStd:
                self.log_std = self.log_std.cuda()
            self.cuda()

    def reset(self):
        self.p_fc.apply(weights_init)

    def forward(self, x):
        return torch.tanh(self.p_fc(x))

    def load(self, path):
        print(f'load exo nn {path}')
        sd = torch.load(path, map_location=torch.device('cpu'), weights_only=False)
        self.load_state_dict(convert_numpy_to_torch_tensor(sd))

    def save(self, path):
        torch.save(self.state_dict(), path)

    def get_action_no_grad(self, s):
        with torch.no_grad():
            ts = torch.tensor(s)
            return self.forward(ts).cpu().detach().numpy()


class ExoActorCritic(nn.Module):
    def __init__(self, num_states, num_actions, learningStd=False, is_cpu=True):
        nn.Module.__init__(self)
        self.is_cpu = is_cpu
        if not self.is_cpu and not torch.cuda.is_available():
            self.is_cpu = True
        self.num_states  = num_states
        self.num_actions = num_actions
        self.p_fc1 = nn.Linear(num_states, 128)
        self.p_fc2 = nn.Linear(128, 64)
        self.p_fc3 = nn.Linear(64, num_actions)
        self.v_fc1 = nn.Linear(num_states, 128)
        self.v_fc2 = nn.Linear(128, 64)
        self.v_fc3 = nn.Linear(64, 1)
        self.init_weights()
        log_std_val = torch.tensor(np.log(0.35), dtype=torch.float32)
        if learningStd:
            self.log_std = nn.Parameter(torch.zeros(num_actions))
        else:
            self.log_std = nn.Parameter(
                log_std_val.repeat(num_actions), requires_grad=False)
        if torch.cuda.is_available() and not self.is_cpu:
            self.cuda()

    def init_weights(self):
        for m in self.modules():
            if type(m) is nn.Linear:
                nn.init.xavier_normal_(m.weight)
                nn.init.zeros_(m.bias)

    def forward(self, x):
        p = torch.relu(self.p_fc1(x))
        p = torch.relu(self.p_fc2(p))
        return torch.tanh(self.p_fc3(p))

    def load(self, path):
        print(f'load actor-critic nn {path}')
        sd = torch.load(path, map_location=torch.device('cpu'), weights_only=False)
        self.load_state_dict(convert_numpy_to_torch_tensor(sd))

    def save(self, path):
        torch.save(self.state_dict(), path)

    def get_action_no_grad(self, s):
        with torch.no_grad():
            ts = torch.tensor(s)
            return self.forward(ts).cpu().detach().numpy()


class ExoActorCriticUchida(nn.Module):
    """Actor-Critic matching uchida_bio4 state-dict: p_fc / v_fc each 18→64→64→64→output."""

    def __init__(self, num_states, num_actions, is_cpu=True):
        super().__init__()
        self.is_cpu = is_cpu
        h = 64
        self.p_fc = nn.Sequential(
            nn.Linear(num_states, h), nn.ReLU(inplace=True),
            nn.Linear(h, h),          nn.ReLU(inplace=True),
            nn.Linear(h, h),          nn.ReLU(inplace=True),
            nn.Linear(h, num_actions),
        )
        self.v_fc = nn.Sequential(
            nn.Linear(num_states, h), nn.ReLU(inplace=True),
            nn.Linear(h, h),          nn.ReLU(inplace=True),
            nn.Linear(h, h),          nn.ReLU(inplace=True),
            nn.Linear(h, 1),
        )

    def forward(self, x):
        return torch.tanh(self.p_fc(x))

    def load(self, path):
        print(f'load uchida actor-critic nn {path}')
        sd = torch.load(path, map_location=torch.device('cpu'), weights_only=False)
        self.load_state_dict(convert_numpy_to_torch_tensor(sd))

    def get_action_no_grad(self, s):
        with torch.no_grad():
            ts = torch.tensor(s, dtype=torch.float32)
            if ts.dim() == 1:
                ts = ts.unsqueeze(0)
            return self.forward(ts).squeeze(0).cpu().detach().numpy()


class ExoActorMyoAssist(nn.Module):
    SWAP_INDICES = [1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 17, 16]

    def __init__(self, symmetric=True, is_cpu=True):
        nn.Module.__init__(self)
        self.is_cpu = is_cpu
        if not self.is_cpu and not torch.cuda.is_available():
            self.is_cpu = True
        self.symmetric = symmetric
        self._swap_idx = torch.tensor(self.SWAP_INDICES, dtype=torch.long)
        self.net = nn.Sequential(
            nn.Linear(18, 128), nn.Tanh(),
            nn.Linear(128, 64), nn.Tanh(),
            nn.Linear(64, 2),   nn.Tanh(),
        )
        if torch.cuda.is_available() and not self.is_cpu:
            self.cuda()

    def forward(self, obs):
        if self.symmetric:
            # Batch both obs and its swapped version into a single forward pass
            # so the weight matrices are loaded into cache only once.
            batch = torch.cat([obs, obs[:, self._swap_idx]], dim=0)  # (2, 18)
            out   = self.net(batch)                                   # (2, 2)
            return torch.cat([out[0:1, 0:1], out[1:2, 0:1]], dim=-1)
        return self.net(obs)

    def load(self, zip_path):
        print(f'Loading MyoAssist model from {zip_path}')
        with zipfile.ZipFile(zip_path, "r") as z:
            with z.open("policy.pth") as f:
                full_sd = torch.load(io.BytesIO(f.read()),
                                     map_location=torch.device('cpu'),
                                     weights_only=False)
        prefix = "policy_network.exo_policy_net."
        exo_sd = {k[len(prefix):]: v
                  for k, v in full_sd.items() if k.startswith(prefix)}
        self.net.load_state_dict(exo_sd)
        self.eval()
        print(f"Loaded MyoAssist exo actor (symmetric={self.symmetric})")

    def get_action_no_grad(self, s):
        with torch.no_grad():
            ts = torch.tensor(s, dtype=torch.float32)
            if ts.dim() == 1:
                ts = ts.unsqueeze(0)
            return self.forward(ts).squeeze(0).cpu().detach().numpy()


# ─────────────────────────────────────────────
# NN Mode Initialization
# ─────────────────────────────────────────────
def initialize_nn_mode():
    global nn_current_id
    d1_data = []; d2_data = []
    v1_data = []; v2_data = []
    input_history = output_history = None

    current_model_name = NN_NAMES[nn_current_id]
    input_size = MODEL_INPUT_SIZES.get(current_model_name, 16)

    if current_model_name == "myoassist_model":
        exo = ExoActorMyoAssist(symmetric=True, is_cpu=True)
        input_history = deque(maxlen=MYOASSIST_INPUT_HISTORY_FRAMES)
        for _ in range(MYOASSIST_INPUT_HISTORY_FRAMES):
            input_history.append(np.zeros(4))
        output_history = deque(maxlen=MYOASSIST_OUTPUT_HISTORY_FRAMES)
        for _ in range(MYOASSIST_OUTPUT_HISTORY_FRAMES):
            output_history.append(np.zeros(2))
    elif current_model_name == "uchida_bio4":
        exo = ExoActorCriticUchida(input_size, num_action, is_cpu=True)
        input_history = deque(maxlen=MYOASSIST_INPUT_HISTORY_FRAMES)
        for _ in range(MYOASSIST_INPUT_HISTORY_FRAMES):
            input_history.append(np.zeros(4))
        output_history = deque(maxlen=MYOASSIST_OUTPUT_HISTORY_FRAMES)
        for _ in range(MYOASSIST_OUTPUT_HISTORY_FRAMES):
            output_history.append(np.zeros(2))
    elif current_model_name == "testing_model2":
        exo = ExoActorCritic(input_size, num_action, learningStd, is_cpu=True)
    else:
        exo = ExoNN(input_size, num_action, learningStd, is_cpu=True)

    if current_model_name in MODEL_FILE_MAPPING:
        exo.load(MODEL_FILE_MAPPING[current_model_name])
        print(f"Loaded model: {current_model_name}  input_size={input_size}")

    # Warm up PyTorch JIT before the control loop starts so the first real
    # inference doesn't block the CAN bus long enough to trigger motor watchdog.
    with torch.no_grad():
        dummy = torch.zeros(1, input_size, dtype=torch.float32)
        exo(dummy)
    print("Model warmed up.")

    exo_input = [0.0] * input_size
    return d1_data, d2_data, v1_data, v2_data, exo, exo_input, input_history, output_history


# ─────────────────────────────────────────────
# Global Filter Initialization
# ─────────────────────────────────────────────
def initialize_globals():
    global d1_data, d2_data, v1_data, v2_data, exo, exo_input, pt_file
    global input_history, output_history
    d1_data = []; d2_data = []
    v1_data = []; v2_data = []
    input_history = output_history = None
    exo = ExoNN(num_state, num_action, learningStd, is_cpu=True)
    exo_input = [0.0] * num_state
    pt_file = MODEL_FILE_MAPPING[NN_NAMES[nn_current_id]]
    exo.load(pt_file)

    global filter_obj_velocity_left, filter_obj_velocity_right
    global filter_obj_torque_left, filter_obj_torque_right
    global filter_obj_euro_velocity_left, filter_obj_euro_velocity_right
    global filter_obj_euro_torque_left, filter_obj_euro_torque_right
    global filter_obj_euro_velocity_left_nn, filter_obj_euro_velocity_right_nn

    filter_obj_velocity_left  = RealFilter()
    filter_obj_velocity_right = RealFilter()
    filter_obj_torque_left    = RealFilter()
    filter_obj_torque_right   = RealFilter()

    filter_obj_euro_velocity_left   = OneEuroFilter(min_cutoff, beta, dcutoff)
    filter_obj_euro_velocity_right  = OneEuroFilter(min_cutoff, beta, dcutoff)
    filter_obj_euro_torque_left     = OneEuroFilter(min_cutoff_torque, beta, dcutoff)
    filter_obj_euro_torque_right    = OneEuroFilter(min_cutoff_torque, beta, dcutoff)

    # Reset MARL-sim Butterworth filter states
    global _zi_ang_L, _zi_ang_R, _zi_vel_L, _zi_vel_R, _zi_act_L, _zi_act_R
    _zi_ang_L = sp_signal.lfilter_zi(_b_vel, _a_vel).copy()
    _zi_ang_R = sp_signal.lfilter_zi(_b_vel, _a_vel).copy()
    _zi_vel_L = sp_signal.lfilter_zi(_b_vel, _a_vel).copy()
    _zi_vel_R = sp_signal.lfilter_zi(_b_vel, _a_vel).copy()
    _zi_act_L = sp_signal.lfilter_zi(_b_act, _a_act).copy()
    _zi_act_R = sp_signal.lfilter_zi(_b_act, _a_act).copy()
    filter_obj_euro_velocity_left_nn  = OneEuroFilter(min_cutoff_velocity, beta, dcutoff)
    filter_obj_euro_velocity_right_nn = OneEuroFilter(min_cutoff_velocity, beta, dcutoff)


# ─────────────────────────────────────────────
# Recording
# ─────────────────────────────────────────────
def start_new_recording():
    global recording_filename, profiling_results, record_step_counter
    if recording_filename is None:
        ts = time.strftime("%Y%m%d_%H%M%S")
        recording_filename = f'HipExoData_{ts}.csv'
    profiling_results = []
    record_step_counter = 0
    print(f"Started recording: {recording_filename}")


# ─────────────────────────────────────────────
# Unified Sensor Reading  (NEW in Apr20)
# ─────────────────────────────────────────────
def read_joint_states():
    """
    Sends last_torque_L / last_torque_R to motors (gets encoder feedback),
    then returns joint angles and velocities from LPMS or encoder depending
    on USE_LPMS flag.

    Returns:
        (lh, rh, lhv, rhv, encoder_data)  on success, or None on failure.

        lh, rh     : left / right hip angle  [rad]  – model-convention
        lhv, rhv   : left / right hip velocity [rad/s] – model-convention
        encoder_data: tuple (enc_lh, enc_rh, enc_lhv, enc_rhv) for logging,
                      or None if CAN error

    Sign conventions when USE_LPMS=False (encoder path):
        Left:  flexion-positive  (LEFT_ENCODER_SIGN  = +1 raw → flexion)
        Right: flexion-positive  (RIGHT_ENCODER_SIGN = -1 raw → flexion)
        Both are then adjusted inside each mode function to match the
        specific model's training convention (extension-positive for legacy,
        flexion-positive for MyoAssist).  This matches Feb24 behaviour.

    Sign conventions when USE_LPMS=True:
        lh / rh are already in radians with LPMS_LH_SIGN / LPMS_RH_SIGN applied.
        Default: LPMS_LH_SIGN = LPMS_RH_SIGN = +1 (flexion-positive from
        pelvis−femur calculation).  Adjust if model expects different convention.
    """
    global can_loop_success_count, can_loop_error_count, lpms_fail_count

    enc_lh = enc_rh = enc_lhv = enc_rhv = None

    # ── Always send torque & read encoder ─────────────────────────────
    try:
        resp_l = L_H_driver.sendTorqueSetpoint(last_torque_L, TORQUE_CONST)
        resp_r = R_H_driver.sendTorqueSetpoint(last_torque_R, TORQUE_CONST)
        can_loop_success_count += 1

        # Convert to radians, flexion-positive via ENCODER_SIGN
        enc_lh  = LEFT_ENCODER_SIGN  * math.radians(resp_l.shaft_angle)
        enc_rh  = RIGHT_ENCODER_SIGN * math.radians(resp_r.shaft_angle)
        enc_lhv = LEFT_ENCODER_SIGN  * math.radians(resp_l.shaft_speed)
        enc_rhv = RIGHT_ENCODER_SIGN * math.radians(resp_r.shaft_speed)
    except (ProtocolException, can.SocketException,
            can.ControllerProblemError, can.BusOffError) as e:
        can_loop_error_count += 1
        print(f"[CAN ERROR] {e}")

    encoder_data = ((enc_lh, enc_rh, enc_lhv, enc_rhv)
                    if enc_lh is not None else None)

    # ── LPMS path ──────────────────────────────────────────────────────
    if USE_LPMS and lpms_manager and lpms_client:
        try:
            # Drain ALL pending events from OpenZen queue so get_hip_joint_states()
            # returns the freshest sensor data, not a stale buffered value.
            _sensor_map = {
                lpms_imus[0].sensor.handle: (lpms_imus[0], lpms_manager.pelvis),
                lpms_imus[1].sensor.handle: (lpms_imus[1], lpms_manager.right_femur),
                lpms_imus[2].sensor.handle: (lpms_imus[2], lpms_manager.left_femur),
            }
            while True:
                ev = lpms_client.poll_next_event()
                if ev is None:
                    break
                if ev.event_type != openzen.ZenEventType.ImuData:
                    continue
                entry = _sensor_map.get(ev.sensor.handle)
                if entry is None:
                    continue
                imu_obj, store = entry
                if ev.component.handle == imu_obj.component.handle:
                    # Use sensor's own timestamp so central-difference dt is always
                    # the true sensor interval (~10 ms at 100 Hz), not the irregular
                    # Pi wall-clock time which causes velocity spikes when BT bursts.
                    ts = ev.data.imu_data.timestamp
                    store.add_data(ts, ev.data.imu_data.r[0], ev.data.imu_data)

            result = lpms_manager.get_hip_joint_states()
            if result is None:
                raise ValueError("LPMS data not yet available")
            lh, rh, lhv, rhv = result
            return (lh, rh, lhv, rhv, encoder_data)
        except Exception as e:
            lpms_fail_count += 1
            print(f"[LPMS] read failed: {e} – skipping cycle")
            return None

    # ── Encoder path ───────────────────────────────────────────────────
    if encoder_data is not None:
        return (enc_lh, enc_rh, enc_lhv, enc_rhv, encoder_data)

    # Both paths failed
    print("[ERROR] Both encoder and LPMS reads failed – skipping cycle")
    return None


# ─────────────────────────────────────────────
# Torque Utilities
# ─────────────────────────────────────────────
TORQUE_DEADBAND = 0.3   # Nm — torques below this are zeroed (model residual at standstill)

def apply_torque_limit(value):
    return max(-TORQUE_LIMIT, min(TORQUE_LIMIT, value))

def apply_torque_deadband(value):
    return 0.0 if abs(value) < TORQUE_DEADBAND else value


def safe_send_torques(torque_left, torque_right):
    attempt = 0
    while attempt < 3:
        try:
            L_H_driver.sendTorqueSetpoint(torque_left,  TORQUE_CONST)
            R_H_driver.sendTorqueSetpoint(torque_right, TORQUE_CONST)
            return True
        except (ProtocolException, can.SocketException,
                can.ControllerProblemError, can.BusOffError) as e:
            print(f"safe_send_torques error: {e}")
            attempt += 1
            time.sleep(sampling_interval)
    return False


def reset_mode_flags():
    global nn_initialized, spring_initialized, imp_initialized
    global spline_initialized, bio_initialized
    nn_initialized = spring_initialized = imp_initialized = False
    spline_initialized = bio_initialized = False


# ─────────────────────────────────────────────
# Control Mode Functions
# (modified to use read_joint_states)
# ─────────────────────────────────────────────
def NN_mode():
    global d1_data, d2_data, v1_data, v2_data, exo, exo_input
    global torque_L, torque_R, torque_L_filtered, torque_R_filtered
    global last_torque_L, last_torque_R, max_torque, RECORDING
    global nn_delay, input_history, output_history

    current_time = time.time() - start_time
    current_model_name = NN_NAMES[nn_current_id]
    is_myoassist = (current_model_name == "myoassist_model")
    is_uchida    = (current_model_name == "uchida_bio4")

    # ── Read joint states (encoder or LPMS) ───────────────────────────
    result = read_joint_states()
    if result is None:
        return None

    lh_raw, rh_raw, lhv_raw, rhv_raw, encoder_data = result

    # ── Apply model-specific sign conventions ─────────────────────────
    if is_myoassist or is_uchida:
        # Flexion-positive convention; LPMS_*_SIGN already applied.
        left_hip_angle_rad    = lh_raw
        right_hip_angle_rad   = rh_raw
        left_hip_velocity_rad = lhv_raw
        right_hip_velocity_rad = rhv_raw
    else:
        # Legacy models: extension-positive convention.
        # Encoder path (lh_raw = +radians, flexion+) → negate left to get ext+.
        # LPMS path: same correction needed IF LPMS is flexion-positive.
        # NOTE: If LPMS_LH_SIGN was already set to -1, the sign is already correct.
        if USE_LPMS:
            # LPMS returns angles with LPMS_LH/RH_SIGN applied.
            # For legacy models trained with (ext+ left, ext+ right), negate both
            # if LPMS is flexion-positive (default LPMS_LH_SIGN = +1).
            left_hip_angle_rad    = -lh_raw
            right_hip_angle_rad   = -rh_raw
            left_hip_velocity_rad = -lhv_raw
            right_hip_velocity_rad = -rhv_raw
        else:
            # Encoder flexion-positive → negate left to get extension-positive left.
            left_hip_angle_rad    = -lh_raw   # ext+ left
            right_hip_angle_rad   = -rh_raw   # ext+ right (enc was already flex+, negate)
            left_hip_velocity_rad = -lhv_raw
            right_hip_velocity_rad = -rhv_raw

    # Filter velocities — MyoAssist was trained on raw data with no filtering
    if is_myoassist:
        left_hip_velocity_filtered  = left_hip_velocity_rad
        right_hip_velocity_filtered = right_hip_velocity_rad
    else:
        left_hip_velocity_filtered  = apply_filter(
            left_hip_velocity_rad,  "velocity", current_time, "left")
        right_hip_velocity_filtered = apply_filter(
            right_hip_velocity_rad, "velocity", current_time, "right")

    # ── Build model input & infer ─────────────────────────────────────
    if is_myoassist:
        current_input = np.array([
            right_hip_angle_rad,
            left_hip_angle_rad,
            right_hip_velocity_rad,
            left_hip_velocity_rad,
        ])
        parts = list(input_history) + [current_input] + list(output_history)
        obs   = np.concatenate(parts)
        obs_t = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
        with torch.no_grad():
            predicted_output = exo(obs_t).numpy().flatten()
        torque_R = predicted_output[0] * max_torque
        torque_L = predicted_output[1] * max_torque
        input_history.append(current_input.copy())
        output_history.append(predicted_output.copy())
    elif is_uchida:
        # Input: [HipL, HipR, HipLV, HipRV] × 3 (state history) + [ActL, ActR] × 3 (action history)
        left_hip_angle_filtered  = apply_filter(
            left_hip_angle_rad,  "angle", current_time, "left")
        right_hip_angle_filtered = apply_filter(
            right_hip_angle_rad, "angle", current_time, "right")
        current_input = np.array([
            left_hip_angle_filtered,
            right_hip_angle_filtered,
            left_hip_velocity_filtered,
            right_hip_velocity_filtered,
        ])
        parts = list(input_history) + [current_input] + list(output_history)
        obs   = np.concatenate(parts)
        obs_t = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
        with torch.no_grad():
            predicted_output = exo(obs_t).numpy().flatten()
        torque_L = predicted_output[0] * max_torque   # ActionHipL
        torque_R = predicted_output[1] * max_torque   # ActionHipR
        input_history.append(current_input.copy())
        output_history.append(predicted_output.copy())
    else:
        d1_data.append(left_hip_angle_rad)
        d2_data.append(right_hip_angle_rad)
        v1_data.append(left_hip_velocity_filtered)
        v2_data.append(right_hip_velocity_filtered)

        if len(d1_data) > num_history:
            d1_data.pop(0); d2_data.pop(0)
            v1_data.pop(0); v2_data.pop(0)

        if len(d1_data) == num_history:
            input_size = MODEL_INPUT_SIZES.get(current_model_name, 16)
            for i in range(num_history):
                exo_input[4*i]   = d1_data[i]
                exo_input[4*i+1] = d2_data[i]
                exo_input[4*i+2] = v1_data[i] * nn_vscale
                exo_input[4*i+3] = v2_data[i] * nn_vscale

            if input_size == 18 and current_model_name == "testing_model2":
                exo_input[16] = nn_delay
                exo_input[17] = nn_vscale
            elif input_size == 17 and nn_delay > 0:
                exo_input[16] = nn_delay

            exo_input_t = torch.tensor(
                exo_input, dtype=torch.float32).unsqueeze(0)
            with torch.no_grad():
                predicted_output = exo(exo_input_t).numpy().flatten()
            torque_L = -predicted_output[0] * max_torque
            torque_R =  predicted_output[1] * max_torque
        else:
            torque_L = torque_R = 0.0

    # Filter, deadband & limit torques — MyoAssist: no filter on torque output
    if is_myoassist:
        torque_L_filtered = apply_torque_limit(apply_torque_deadband(torque_L))
        torque_R_filtered = apply_torque_limit(apply_torque_deadband(torque_R))
    else:
        torque_L_filtered = apply_torque_limit(apply_torque_deadband(
            apply_filter(torque_L, "torque", current_time, "left")))
        torque_R_filtered = apply_torque_limit(apply_torque_deadband(
            apply_filter(torque_R, "torque", current_time, "right")))

    # Motor command signs + slew rate limit
    torque_cmd_L = LEFT_MOTOR_CMD_SIGN  * torque_L_filtered
    torque_cmd_R = RIGHT_MOTOR_CMD_SIGN * torque_R_filtered
    max_delta    = TORQUE_SLEW_NM_PER_S * (1.0 / CONTROL_RATE_HZ)
    torque_cmd_L = float(np.clip(torque_cmd_L,
                                 last_torque_L - max_delta,
                                 last_torque_L + max_delta))
    torque_cmd_R = float(np.clip(torque_cmd_R,
                                 last_torque_R - max_delta,
                                 last_torque_R + max_delta))

    if not safe_send_torques(torque_cmd_L, torque_cmd_R):
        print("Failed to send torque commands.")
        return None

    last_torque_L = torque_cmd_L
    last_torque_R = torque_cmd_R

    enc_lh_val = enc_rh_val = enc_lhv_val = enc_rhv_val = float('nan')
    if encoder_data is not None:
        enc_lh_val, enc_rh_val, enc_lhv_val, enc_rhv_val = encoder_data

    return {
        "time": current_time,
        "left_angle":      left_hip_angle_rad,
        "right_angle":     right_hip_angle_rad,
        "left_vel":        left_hip_velocity_rad,
        "right_vel":       right_hip_velocity_rad,
        "left_vel_filt":   left_hip_velocity_filtered,
        "right_vel_filt":  right_hip_velocity_filtered,
        "torque_L":        torque_L,
        "torque_R":        torque_R,
        "torque_L_filt":   torque_L_filtered,
        "torque_R_filt":   torque_R_filtered,
        "sensor_source":   "LPMS" if USE_LPMS else "ENCODER",
        "enc_lh":          enc_lh_val,
        "enc_rh":          enc_rh_val,
        "enc_lhv":         enc_lhv_val,
        "enc_rhv":         enc_rhv_val,
    }


def Spring_Mode():
    global torque_L, torque_R, last_torque_L, last_torque_R, ks, spring_initialized

    current_time = time.time() - start_time

    result = read_joint_states()
    if result is None:
        return None

    lh_raw, rh_raw, lhv_raw, rhv_raw, _enc = result

    # Extension-positive convention (same as Feb24)
    left_hip_angle_rad    = -lh_raw if not USE_LPMS else -lh_raw
    right_hip_angle_rad   = -rh_raw if not USE_LPMS else -rh_raw
    left_hip_velocity_rad  = -lhv_raw if not USE_LPMS else -lhv_raw
    right_hip_velocity_rad = -rhv_raw if not USE_LPMS else -rhv_raw

    left_hip_velocity_filtered  = apply_filter(
        left_hip_velocity_rad,  "velocity", current_time, "left")
    right_hip_velocity_filtered = apply_filter(
        right_hip_velocity_rad, "velocity", current_time, "right")

    # Relative angle in degrees (working in raw degree space for spring law)
    left_deg  = math.degrees(-left_hip_angle_rad)   # back to raw-like degrees
    right_deg = math.degrees(-right_hip_angle_rad)
    theta_rel = left_deg - right_deg

    torque_L = -(  -ks * theta_rel)
    torque_R =     ks * theta_rel

    torque_L_filtered = apply_torque_limit(
        apply_filter(torque_L, "torque", current_time, "left"))
    torque_R_filtered = apply_torque_limit(
        apply_filter(torque_R, "torque", current_time, "right"))

    if not safe_send_torques(torque_L_filtered, torque_R_filtered):
        return None

    last_torque_L = torque_L_filtered
    last_torque_R = torque_R_filtered
    spring_initialized = True

    return {
        "time": current_time,
        "left_angle":      left_hip_angle_rad,
        "right_angle":     right_hip_angle_rad,
        "left_vel":        left_hip_velocity_rad,
        "right_vel":       right_hip_velocity_rad,
        "left_vel_filt":   left_hip_velocity_filtered,
        "right_vel_filt":  right_hip_velocity_filtered,
        "torque_L":        torque_L,
        "torque_R":        torque_R,
        "torque_L_filt":   torque_L_filtered,
        "torque_R_filt":   torque_R_filtered,
        "sensor_source":   "LPMS" if USE_LPMS else "ENCODER",
    }


def Impedance_Mode():
    global torque_L, torque_R, torque_L_filtered, torque_R_filtered
    global last_torque_L, last_torque_R, kp, kv, imp_initialized

    Theta_eq = 0.0
    current_time = time.time() - start_time

    result = read_joint_states()
    if result is None:
        return None

    lh_raw, rh_raw, lhv_raw, rhv_raw, _enc = result

    # Extension-positive for left (matching Feb24 convention)
    left_hip_angle_rad    = -lh_raw
    right_hip_angle_rad   = -rh_raw
    left_hip_velocity_rad  = -lhv_raw
    right_hip_velocity_rad = -rhv_raw

    left_hip_velocity_filtered  = apply_filter(
        left_hip_velocity_rad,  "velocity", current_time, "left")
    right_hip_velocity_filtered = apply_filter(
        right_hip_velocity_rad, "velocity", current_time, "right")

    # Convert back to degrees for impedance law (matches Feb24)
    left_deg  = math.degrees(-left_hip_angle_rad)
    right_deg = math.degrees(-right_hip_angle_rad)
    left_vel_deg  = math.degrees(-left_hip_velocity_rad)
    right_vel_deg = math.degrees(-right_hip_velocity_rad)

    torque_L = -(kp * (left_deg  - Theta_eq) + kv * left_vel_deg)
    torque_R =   kp * (left_deg  - Theta_eq) + kv * right_vel_deg

    torque_L_filtered = apply_torque_limit(
        apply_filter(torque_L, "torque", current_time, "left"))
    torque_R_filtered = apply_torque_limit(
        apply_filter(torque_R, "torque", current_time, "right"))

    if not safe_send_torques(torque_L_filtered, torque_R_filtered):
        return None

    last_torque_L = torque_L_filtered
    last_torque_R = torque_R_filtered
    imp_initialized = True

    return {
        "time": current_time,
        "left_angle":      left_hip_angle_rad,
        "right_angle":     right_hip_angle_rad,
        "left_vel":        left_hip_velocity_rad,
        "right_vel":       right_hip_velocity_rad,
        "left_vel_filt":   left_hip_velocity_filtered,
        "right_vel_filt":  right_hip_velocity_filtered,
        "torque_L":        torque_L,
        "torque_R":        torque_R,
        "torque_L_filt":   torque_L_filtered,
        "torque_R_filt":   torque_R_filtered,
        "sensor_source":   "LPMS" if USE_LPMS else "ENCODER",
    }


def DOFC_Mode():
    global torque_L, torque_R, torque_L_filtered, torque_R_filtered
    global last_torque_L, last_torque_R

    K_dofc = 1.0
    current_time = time.time() - start_time

    result = read_joint_states()
    if result is None:
        return None

    lh_raw, rh_raw, _lhv, _rhv, _enc = result

    left_deg  = math.degrees(-lh_raw)
    right_deg = math.degrees(-rh_raw)

    current_motion_state = math.sin(math.radians(left_deg)) - \
                           math.sin(math.radians(right_deg))
    motion_state_buffer.append(current_motion_state)

    if len(motion_state_buffer) < buffer_capacity:
        return None

    # (DOFC torque computation left as pass-through per Feb24)
    return None


def Zero_Mode():
    global torque_L, torque_R, torque_L_filtered, torque_R_filtered
    global last_torque_L, last_torque_R

    current_time = time.time() - start_time
    torque_L = torque_R = 0.0
    torque_L_filtered = torque_R_filtered = 0.0

    result = read_joint_states()
    if result is None:
        # Even if read fails, try sending zero torque
        safe_send_torques(0.0, 0.0)
        return None

    lh_raw, rh_raw, lhv_raw, rhv_raw, enc = result

    if NN_NAMES[nn_current_id] == "no_assistance":
        # flexion-positive convention (same as myoassist/uchida)
        left_hip_angle_rad    = lh_raw
        right_hip_angle_rad   = rh_raw
        left_hip_velocity_rad  = lhv_raw
        right_hip_velocity_rad = rhv_raw
    else:
        # legacy extension-positive convention for other Zero_Mode uses
        left_hip_angle_rad    = -lh_raw
        right_hip_angle_rad   = -rh_raw
        left_hip_velocity_rad  = -lhv_raw
        right_hip_velocity_rad = -rhv_raw

    left_hip_velocity_filtered  = apply_filter(
        left_hip_velocity_rad,  "velocity", current_time, "left")
    right_hip_velocity_filtered = apply_filter(
        right_hip_velocity_rad, "velocity", current_time, "right")

    safe_send_torques(0.0, 0.0)
    last_torque_L = last_torque_R = 0.0

    enc_lh_val = enc_rh_val = enc_lhv_val = enc_rhv_val = float('nan')
    if enc is not None:
        enc_lh_val, enc_rh_val, enc_lhv_val, enc_rhv_val = enc

    return {
        "time": current_time,
        "left_angle":      left_hip_angle_rad,
        "right_angle":     right_hip_angle_rad,
        "left_vel":        left_hip_velocity_rad,
        "right_vel":       right_hip_velocity_rad,
        "left_vel_filt":   left_hip_velocity_filtered,
        "right_vel_filt":  right_hip_velocity_filtered,
        "torque_L":        0.0,
        "torque_R":        0.0,
        "torque_L_filt":   0.0,
        "torque_R_filt":   0.0,
        "sensor_source":   "LPMS" if USE_LPMS else "ENCODER",
        "enc_lh":  enc_lh_val,
        "enc_rh":  enc_rh_val,
        "enc_lhv": enc_lhv_val,
        "enc_rhv": enc_rhv_val,
    }


def Spline_Mode():
    pass


def Biological_Mode():
    pass


def run_mode():
    if   control_mode == 0: return NN_mode()
    elif control_mode == 1: return Spring_Mode()
    elif control_mode == 2: return Impedance_Mode()
    elif control_mode == 3: return Spline_Mode()
    elif control_mode == 4: return Biological_Mode()
    elif control_mode == 5: return DOFC_Mode()
    elif control_mode == 6: return Zero_Mode()
    return None


# ─────────────────────────────────────────────
# UDP Send & Record
# ─────────────────────────────────────────────
CSV_HEADER = (
    "time,"
    "left_hip_angle_rad,right_hip_angle_rad,"
    "left_hip_velocity_rad_raw,right_hip_velocity_rad_raw,"
    "left_hip_velocity_rad_filt,right_hip_velocity_rad_filt,"
    "torque_L_raw,torque_R_raw,"
    "torque_L_filt,torque_R_filt,"
    "sensor_source,"
    "pelvis_angle_deg,right_femur_angle_deg,left_femur_angle_deg,"
    "enc_lh_rad,enc_rh_rad,enc_lhv_rad,enc_rhv_rad\n"
)


def save_profiling_results():
    need_header = (not os.path.exists(recording_filename)
                   or os.path.getsize(recording_filename) == 0)
    with open(recording_filename, 'a') as f:
        if need_header:
            f.write(CSV_HEADER)
        f.writelines(profiling_results)


def send_and_record(data):
    global record_step_counter
    if data is None:
        return

    if SEND_DATA:
        message = struct.pack(
            'ddddddddddd',
            data['time'],
            data['left_angle']     * 57.2958,
            data['right_angle']    * 57.2958,
            data['left_vel']       * 57.2958,
            data['right_vel']      * 57.2958,
            data['left_vel_filt']  * 57.2958,
            data['right_vel_filt'] * 57.2958,
            data['torque_L'],
            data['torque_R'],
            data['torque_L_filt'],
            data['torque_R_filt'],
        )
        sock_send.sendto(message, (UDP_IP_PC, DATA_PORT))

    if RECORDING:
        record_step_counter += 1
        if (record_step_counter - 1) % (CONTROL_RATE_HZ // RECORD_RATE_HZ) == 0:
            src = data.get('sensor_source', 'ENCODER')
            pelvis_deg = right_femur_deg = left_femur_deg = float('nan')
            if USE_LPMS and lpms_manager:
                p  = lpms_manager.pelvis.get_latest()
                rf = lpms_manager.right_femur.get_latest()
                lf = lpms_manager.left_femur.get_latest()
                if p:  pelvis_deg      = p['angle']
                if rf: right_femur_deg = rf['angle']
                if lf: left_femur_deg  = lf['angle']
            profiling_results.append(
                f"{data['time']:.6f},"
                f"{data['left_angle']:.6f},{data['right_angle']:.6f},"
                f"{data['left_vel']:.6f},{data['right_vel']:.6f},"
                f"{data['left_vel_filt']:.6f},{data['right_vel_filt']:.6f},"
                f"{data['torque_L']:.6f},{data['torque_R']:.6f},"
                f"{data['torque_L_filt']:.6f},{data['torque_R_filt']:.6f},"
                f"{src},"
                f"{pelvis_deg:.6f},{right_femur_deg:.6f},{left_femur_deg:.6f},"
                f"{data.get('enc_lh', float('nan')):.6f},"
                f"{data.get('enc_rh', float('nan')):.6f},"
                f"{data.get('enc_lhv', float('nan')):.6f},"
                f"{data.get('enc_rhv', float('nan')):.6f}\n"
            )
        if len(profiling_results) >= 200:
            save_profiling_results()
            profiling_results.clear()


# ─────────────────────────────────────────────
# Command Listener
# ─────────────────────────────────────────────
def listen_for_commands():
    global RUNNING, RECORDING, SEND_DATA, FILTER_TYPE, recording_filename
    global control_mode, max_torque, nn_current_id, nn_vscale, nn_delay
    global kp, kv, ks, exo, exo_input
    global USE_LPMS, lpms_manager, lpms_client, lpms_sensors, lpms_imus
    global L_H_driver, R_H_driver

    while True:
        try:
            data, addr = sock_receive.recvfrom(1024)
            command = data.decode().strip().upper()
            print(f"Received command: {command}")

            # ── NEW: sensor source switching ───────────────────────────
            if command.startswith("SET_SOURCE"):
                parts = command.split()
                if len(parts) < 2:
                    print("SET_SOURCE requires LPMS or ENCODER")
                    continue
                src = parts[1]
                if src == "LPMS":
                    if not LPMS_AVAILABLE:
                        print("[WARN] LPMS libraries not available.")
                        continue
                    try:
                        # Disconnect existing connection if any
                        if lpms_client and lpms_sensors:
                            stop_hip_lpms(lpms_client, lpms_sensors)
                        if lpms_manager:
                            lpms_manager.stop()

                        print("[LPMS] Connecting 3 hip sensors…")
                        lpms_client, lpms_sensors, lpms_imus = connect_hip_sensors()
                        lpms_manager = HipOnlySensorDataManager(velocity_type="gyroscope")
                        lpms_manager.connected    = True
                        lpms_manager.sensor_count = 3

                        acq_thread = threading.Thread(
                            target=hip_data_acquisition,
                            args=(lpms_client, lpms_imus,
                                  lpms_manager, start_time),
                            daemon=True,
                        )
                        acq_thread.start()
                        time.sleep(1.0)   # let first samples arrive
                        USE_LPMS = True
                        print("[LPMS] Source switched to LPMS.")
                    except Exception as e:
                        print(f"[LPMS] Connection failed: {e}")
                        USE_LPMS = False

                elif src == "ENCODER":
                    USE_LPMS = False
                    print("Source switched to ENCODER.")

                else:
                    print(f"Unknown source '{src}'. Use LPMS or ENCODER.")

            # ── NEW: IMU calibration (also accepts SET_OFFSETS from GUI) ──
            elif command in ("CALIBRATE_IMU", "SET_OFFSETS"):
                if USE_LPMS and lpms_manager:
                    print("[LPMS] Starting calibration in background thread…")
                    threading.Thread(
                        target=lpms_manager.calibrate_offsets,
                        kwargs={'duration': 5.0},
                        daemon=True,
                    ).start()
                else:
                    print("[WARN] CALIBRATE_IMU only works when USE_LPMS=True.")

            # ── Existing commands (unchanged from Feb24) ───────────────
            elif command == "NN_MODE":
                reset_mode_flags()
                if NN_NAMES[nn_current_id] == "no_assistance":
                    control_mode = 6   # Zero_Mode: zero torque, full data recording
                    RUNNING = True
                    print("No-assistance mode: zero torque, full data recording.")
                else:
                    control_mode = 0
                    global d1_data, d2_data, v1_data, v2_data, nn_initialized
                    global input_history, output_history
                    (d1_data, d2_data, v1_data, v2_data,
                     exo, exo_input, input_history, output_history) = initialize_nn_mode()
                    nn_initialized = True
                    RUNNING      = True
                    print("Setting to Neural Network Mode…")

            elif command.startswith("SET_NN_MODEL"):
                try:
                    parts = command.split()
                    model_id = parts[1]
                    try:
                        model_number = int(model_id)
                    except ValueError:
                        name_map = {
                            "WALK_SCALED_VEL":      "walk_scaled_vel",
                            "WALK_JULY1":           "walk_july1",
                            "WALK_JULY11":          "walk_july11",
                            "WALK_100HZ_15MSDELAY": "walk_100hz_15msdelay",
                            "SQUAT_NOV18":          "squat_nov18",
                            "S2S":                  "s2s",
                            "TESTING_MODEL2":       "testing_model2",
                            "MYOASSIST_MODEL":      "myoassist_model",
                            "UCHIDA_BIO4":          "uchida_bio4",
                            "NO_ASSISTANCE":        "no_assistance",
                        }
                        name = name_map.get(model_id.upper())
                        model_number = NN_NAMES.index(name) if name else -1

                    if 0 <= model_number < len(NN_NAMES):
                        nn_current_id = model_number
                        if len(parts) >= 5:
                            nn_delay    = float(parts[2])
                            max_torque  = float(parts[3])
                            nn_vscale   = float(parts[4])
                        selected = NN_NAMES[nn_current_id]
                        if selected in MODEL_FILE_MAPPING:
                            input_size = MODEL_INPUT_SIZES.get(selected, 16)
                            if selected == "myoassist_model":
                                exo = ExoActorMyoAssist(symmetric=True, is_cpu=True)
                            elif selected == "uchida_bio4":
                                exo = ExoActorCriticUchida(input_size, num_action,
                                                           is_cpu=True)
                            elif selected == "testing_model2":
                                exo = ExoActorCritic(input_size, num_action,
                                                     learningStd, is_cpu=True)
                            else:
                                exo = ExoNN(input_size, num_action,
                                            learningStd, is_cpu=True)
                            exo.load(MODEL_FILE_MAPPING[selected])
                            exo_input = [0.0] * input_size
                            print(f"Loaded model '{selected}'")
                    else:
                        print("Invalid model number.")
                except Exception as e:
                    print(f"SET_NN_MODEL error: {e}")

            elif command == "SPRING_MODE":
                reset_mode_flags(); RUNNING = True; control_mode = 1
            elif command == "IMP_MODE":
                reset_mode_flags(); RUNNING = True; control_mode = 2
            elif command == "Spline_MODE":
                RUNNING = True; control_mode = 3
            elif command == "BIO_MODE":
                reset_mode_flags(); RUNNING = True; control_mode = 4
            elif command == "DOFC_MODE":
                reset_mode_flags(); RUNNING = True; control_mode = 5
            elif command == "ZERO_MODE":
                reset_mode_flags(); RUNNING = True; control_mode = 6

            elif command == "RECORD_DATA":
                start_new_recording()
                udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                udp_socket.sendto(b"START_RECORDING_IMU", ("127.0.0.1", 6000))
                udp_socket.close()
                RECORDING = True

            elif command in ("STOP_RECORD", "STOP_RECORDING"):
                RECORDING = False
                if profiling_results:
                    save_profiling_results()
                    profiling_results.clear()
                print(f"Recording stopped. File: {recording_filename}")

            elif command.startswith("SET_RECORDING_FILENAME"):
                try:
                    recording_filename = command.split()[1]
                except IndexError:
                    pass

            elif command.startswith("SET_SEND_DATA"):
                opt = command.split()[1] if len(command.split()) > 1 else ""
                SEND_DATA = (opt == "ON")

            elif command.startswith("SET_FILTER_TYPE"):
                try:
                    filter_option = command.split()[1].lower()
                    valid = ["oneeuro", "kalman", "exponential",
                             "savitzky-golay", "butterworth", "marl_sim", "none"]
                    if filter_option in valid:
                        FILTER_TYPE = filter_option
                except IndexError:
                    pass

            elif command.startswith("SET_IMP_STIFFNESS"):
                try: kp = float(command.split()[1])
                except: pass

            elif command.startswith("SET_IMP_VELOCITY_GAIN"):
                try: kv = float(command.split()[1])
                except: pass

            elif command.startswith("SET_SPRING_STIFFNESS"):
                try: ks = float(command.split()[1])
                except: pass

            elif command.startswith("SET_TORQUE"):
                try: max_torque = float(command.split()[1])
                except: pass

            elif command.startswith("SET_NN_VSCALE"):
                try: nn_vscale = float(command.split()[1])
                except: pass

            elif command.startswith("SET_NN_DELAY"):
                try: nn_delay = float(command.split()[1])
                except: pass

            elif command.startswith("SET_NN_EPSILON"):
                try: max_torque = float(command.split()[1])
                except: pass

            elif command.startswith("SET_TORQUE_CONST_RH"):
                pass  # right-hip-only torque const not used in hip-only code

            elif command == "SET_ZERO":
                print("Zeroing encoders…")
                L_H_driver.setCurrentPositionAsEncoderZero()
                R_H_driver.setCurrentPositionAsEncoderZero()
                L_H_driver.reset()
                R_H_driver.reset()
                time.sleep(2)
                # Re-create drivers so the new sockets have clean receive buffers.
                # Motor reset generates CAN error frames that accumulate in the old
                # socket and poison the first sendRecv() of the next mode.
                L_H_driver = Driver("can0", L_H_ID)
                R_H_driver = Driver("can0", R_H_ID)
                print("Motors zeroed.")

            elif command == "STOP_MOTOR":
                RUNNING = RECORDING = False
                safe_send_torques(0.0, 0.0)
                print("Motor stopped.")

            elif command == "EXIT":
                break

        except socket.error:
            pass


# ─────────────────────────────────────────────
# Main Loop
# ─────────────────────────────────────────────
def close_sockets():
    sock_receive.close()
    sock_send.close()


def main_loop():
    imu_recorder_process = None
    try:
        imu_ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
        imu_out = f"WIT_imu_data_{imu_ts}.csv"
        imu_recorder_process = subprocess.Popen(
            ["python3", "imu_recorder_3sensors.py", "--output", imu_out],
            stderr=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
        )
        print(f"IMU recorder started: {imu_out}")
    except Exception as e:
        print(f"[WARN] IMU recorder not started: {e}")

    _cycle_log_counter = 0
    try:
        while True:
            t_start = time.time()
            if RUNNING:
                data = run_mode()
                send_and_record(data)
            else:
                time.sleep(0.1)

            elapsed = time.time() - t_start
            remain  = (1.0 / CONTROL_RATE_HZ) - elapsed
            # Log cycle time every 100 cycles so we can see if we're over budget
            _cycle_log_counter += 1
            if RUNNING and _cycle_log_counter % 100 == 0:
                print(f"[TIMING] cycle={elapsed*1000:.1f}ms  remain={remain*1000:.1f}ms")
            if remain > 0:
                time.sleep(remain)

    except KeyboardInterrupt:
        print("Interrupted – saving data and shutting down…")
        if profiling_results:
            save_profiling_results()
        # Zero motors
        L_H_driver.sendTorqueSetpoint(0.0, TORQUE_CONST)
        R_H_driver.sendTorqueSetpoint(0.0, TORQUE_CONST)
        # Stop LPMS if running
        global lpms_client, lpms_sensors, lpms_manager
        if USE_LPMS and lpms_client and lpms_sensors:
            stop_hip_lpms(lpms_client, lpms_sensors)
        if lpms_manager:
            lpms_manager.stop()
        if imu_recorder_process:
            imu_recorder_process.terminate()


def start_application():
    cmd_thread = threading.Thread(target=listen_for_commands, daemon=True)
    cmd_thread.start()
    print("Hip Exo Ready – listening for UDP commands.")
    print("  SET_SOURCE LPMS    → switch to IMU-based control")
    print("  SET_SOURCE ENCODER → switch back to encoder control")
    print("  CALIBRATE_IMU      → zero LPMS hip angles from current posture")
    main_loop()
    close_sockets()


if __name__ == '__main__':
    initialize_globals()
    start_application()
