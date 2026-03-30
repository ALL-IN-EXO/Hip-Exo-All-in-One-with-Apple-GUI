#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RPi_Unified - 统一树莓派 RL 控制器入口
支持 DNN / LSTMNetwork / LSTMNetworkLegDcp 三种网络

用法:
  source ~/venvs/pytorch-env/bin/activate
  python RL_controller_torch.py --nn dnn             # DNN前馈网络 (默认)
  python RL_controller_torch.py --nn lstm             # LSTM网络
  python RL_controller_torch.py --nn lstm_leg_dcp     # LSTM每腿独立解耦
  python RL_controller_torch.py --nn lstm_pd          # LSTM PD位置误差控制

可选参数:
  --nn          神经网络类型: dnn / lstm / lstm_leg_dcp / lstm_pd (默认 dnn)
  --tag         日志文件名标签 (默认按网络类型自动命名, 如 'dnn', 'lstm_leg_dcp')

示例:
  python RL_controller_torch.py --nn lstm_leg_dcp --tag outdoor_walk

硬件接线:
  Pi TX (GPIO14) → Teensy RX8 (pin 34)
  Pi RX (GPIO15) ← Teensy TX8 (pin 35)
  记得共地。
"""

import serial, struct, time, csv, datetime, math, argparse
import numpy as np
import torch
from collections import deque

# ╔══════════════════════════════════════════════════════════════════╗
# ║                    所有可调配置参数 (改这里)                      ║
# ╚══════════════════════════════════════════════════════════════════╝

# ====== 神经网络选择 (可通过命令行 --nn 覆盖) ======
# 'dnn'             → DNN前馈网络 (18→128→64→2), 带5个滤波器接口
# 'lstm'            → LSTMNetwork (4→256→2)
# 'lstm_leg_dcp'    → LSTMNetworkLegDcp (每腿独立, 2→256→1)
# 'lstm_pd'         → LSTMNetworkPD (每腿独立, PD位置误差控制)
_parser = argparse.ArgumentParser(description='RPi Unified RL Controller')
_parser.add_argument('--nn', choices=['dnn', 'lstm', 'lstm_leg_dcp', 'lstm_pd'],
                     default='dnn', help='神经网络类型 (default: dnn)')
_parser.add_argument('--tag', type=str, default=None,
                     help='日志文件名标签 (默认按 --nn 类型自动命名)')
_args, _ = _parser.parse_known_args()
NN_TYPE = _args.nn

# ====== kp, kd (每个网络的PD增益，定义在最前面方便修改) ======
# DNN:          kp=50.0, kd=14.142 (kd = sqrt(50)*2)
# LSTM:         kp=50, kd=0.5*sqrt(50) ≈ 3.536
# LSTMLegDcp:   kp=50, kd=0.5*sqrt(50) ≈ 3.536
kp, kd = 50.0, 0.5 * np.sqrt(50)

# ====== 模型路径 ======
# DNN 模型:
DNN_MODEL_PATH = './models/dnn/Trained_model3.pt'
# LSTM / LSTMLegDcp 模型:
LSTM_MODEL_PATH = './models/lstm/end2end/walkv2_legdecp/max_exo.pt'
# LSTM PD 模型:
LSTM_PD_MODEL_PATH = './models/lstm/end2end/walkv2_pd/max_exo.pt'

CTRL_HZ = 100
dt_ms = 1000.0 / CTRL_HZ            # 10ms

# ---- 实验 / 日志 ----
motion_type = _args.tag if _args.tag else NN_TYPE

# ---- IMU 符号 ----
# DNN 模型输入flexion为正，需要取反; LSTM 不需要
INVERT_IMU_SIGN = (NN_TYPE == 'dnn')

# ---- 通讯 ----
SER_DEV       = '/dev/ttyAMA0'
BAUDRATE      = 115200
TIMEOUT       = 0.01
PI_USE_BINARY = 1                     # 1=二进制帧, 0=文本CSV

# ---- Delay Buffer ----
USE_FILTERED_FOR_DELAY = True         # True=用滤波后torque做delay, False=用raw NN输出
MAX_RUNTIME_DELAY_MS = 1000.0
BUF_SIZE = int(round(MAX_RUNTIME_DELAY_MS / dt_ms)) + 1

# ---- CSV 日志 ----
SAVE_VERBOSE_LOG = False

# ---- Zero Mean (仅DNN模式) ----
ENABLE_ZERO_MEAN       = False
ZERO_MEAN_BUFFER_SIZE  = 200
ZERO_MEAN_WARMUP       = 100

# ---- 速度计算方式 (仅DNN模式) ----
USE_VELOCITY_FROM_DERIVATIVE  = False
DERIVATIVE_DT                 = 0.01
DERIVATIVE_SMOOTH_FILTER_NAME = None

# ╔══════════════════════════════════════════════════════════════════╗
# ║      DNN专用滤波器配置 (仅NN_TYPE='dnn'时生效)                    ║
# ╚══════════════════════════════════════════════════════════════════╝
FILTER_CONFIG_MODE = 'preset'          # 'preset' / 'custom' / 'coeffs'

# -- preset 模式 --
INPUT_POS_FILTER_NAME  = 'butter_1_5hz_2nd'
INPUT_VEL_FILTER_NAME  = 'butter_1_5hz_2nd'
VEL_FILTER_NAME        = 'butter_6hz_2nd'
REF_FILTER_NAME        = 'butter_1_5hz_2nd'
TORQUE_FILTER_NAME     = 'butter_1_5hz_2nd'

# -- custom 模式 --
FILTER_SAMPLE_RATE  = 100.0
VEL_FILTER_TYPE     = 'butterworth';  VEL_FILTER_CUTOFF    = 12.0;  VEL_FILTER_ORDER    = 2
REF_FILTER_TYPE     = 'butterworth';  REF_FILTER_CUTOFF    = 12.0;  REF_FILTER_ORDER    = 2
TORQUE_FILTER_TYPE  = 'butterworth';  TORQUE_FILTER_CUTOFF = 0.0;   TORQUE_FILTER_ORDER = 2

# -- coeffs 模式 --
VEL_FILTER_B_CUSTOM    = np.array([0.0461, 0.0923, 0.0461])
VEL_FILTER_A_CUSTOM    = np.array([1.0, -1.3073, 0.4918])
REF_FILTER_B_CUSTOM    = np.array([0.0461, 0.0923, 0.0461])
REF_FILTER_A_CUSTOM    = np.array([1.0, -1.3073, 0.4918])
TORQUE_FILTER_B_CUSTOM = np.array([0.0461, 0.0923, 0.0461])
TORQUE_FILTER_A_CUSTOM = np.array([1.0, -1.3073, 0.4918])
TORQUE_FILTER_ENABLE_CUSTOM = False

# ╔══════════════════════════════════════════════════════════════════╗
# ║      LSTM专用滤波器配置 (仅LSTM模式时生效)                        ║
# ╚══════════════════════════════════════════════════════════════════╝
# torque前滤波器系数 (Butterworth低通)
# 选择: 6Hz / 12Hz / 15Hz / 20Hz 等
LSTM_FILTER_B = np.array([0.0913, 0.1826, 0.0913])   # 20Hz
LSTM_FILTER_A = np.array([1.0, -0.9824, 0.3477])

# ╔══════════════════════════════════════════════════════════════════╗
# ║                 以下为运行逻辑，一般不需要改                       ║
# ╚══════════════════════════════════════════════════════════════════╝

from filter_library import create_filter, compute_iir_coeffs, RECOMMENDED_FILTERS

# 延迟环形队列
delay_buf_L = deque([0.0] * BUF_SIZE, maxlen=BUF_SIZE)
delay_buf_R = deque([0.0] * BUF_SIZE, maxlen=BUF_SIZE)


# ========= 加载神经网络 =========
def load_network():
    """根据 NN_TYPE 加载对应的神经网络"""

    if NN_TYPE == 'dnn':
        from networks.dnn import DNN
        print(f"\n{'='*80}")
        print(f"加载 DNN 网络: {DNN_MODEL_PATH}")
        print(f"kp={kp}, kd={kd}")
        print(f"{'='*80}")

        # 解析DNN滤波器配置
        filter_params = _parse_dnn_filter_config()

        return DNN(
            kp=kp, kd=kd,
            saved_policy_path=DNN_MODEL_PATH,
            use_velocity_from_derivative=USE_VELOCITY_FROM_DERIVATIVE,
            derivative_dt=DERIVATIVE_DT,
            derivative_smooth_filter_type=filter_params.get('derivative_smooth_filter_type'),
            derivative_smooth_filter_b=filter_params.get('derivative_smooth_filter_b'),
            derivative_smooth_filter_a=filter_params.get('derivative_smooth_filter_a'),
            input_pos_filter_type=filter_params.get('input_pos_filter_type'),
            input_pos_filter_b=filter_params.get('input_pos_filter_b'),
            input_pos_filter_a=filter_params.get('input_pos_filter_a'),
            input_vel_filter_type=filter_params.get('input_vel_filter_type'),
            input_vel_filter_b=filter_params.get('input_vel_filter_b'),
            input_vel_filter_a=filter_params.get('input_vel_filter_a'),
            vel_filter_type=filter_params.get('vel_filter_type'),
            vel_filter_b=filter_params.get('vel_filter_b'),
            vel_filter_a=filter_params.get('vel_filter_a'),
            ref_filter_type=filter_params.get('ref_filter_type'),
            ref_filter_b=filter_params.get('ref_filter_b'),
            ref_filter_a=filter_params.get('ref_filter_a'),
            torque_filter_type=filter_params.get('torque_filter_type'),
            torque_filter_b=filter_params.get('torque_filter_b'),
            torque_filter_a=filter_params.get('torque_filter_a'),
            torque_filter_enable=filter_params.get('torque_filter_enable', False),
            enable_zero_mean=ENABLE_ZERO_MEAN,
            zero_mean_buffer_size=ZERO_MEAN_BUFFER_SIZE,
            zero_mean_warmup=ZERO_MEAN_WARMUP,
        )

    elif NN_TYPE == 'lstm':
        from networks.lstm_network import LSTMNetwork
        print(f"\n{'='*80}")
        print(f"加载 LSTMNetwork: {LSTM_MODEL_PATH}")
        print(f"kp={kp}, kd={kd}")
        print(f"滤波器: b={LSTM_FILTER_B}, a={LSTM_FILTER_A}")
        print(f"{'='*80}")

        nn_obj = LSTMNetwork(
            kp=kp, kd=kd,
            b=LSTM_FILTER_B, a=LSTM_FILTER_A,
        )
        nn_obj.load_saved_policy(torch.load(LSTM_MODEL_PATH, map_location=torch.device('cpu')))
        nn_obj.eval()
        print('load parameters successfully!!')
        return nn_obj

    elif NN_TYPE == 'lstm_leg_dcp':
        from networks.lstm_leg_dcp import LSTMNetworkLegDcp
        print(f"\n{'='*80}")
        print(f"加载 LSTMNetworkLegDcp: {LSTM_MODEL_PATH}")
        print(f"kp={kp}, kd={kd}")
        print(f"滤波器: b={LSTM_FILTER_B}, a={LSTM_FILTER_A}")
        print(f"{'='*80}")

        nn_obj = LSTMNetworkLegDcp(
            kp=kp, kd=kd,
            b=LSTM_FILTER_B, a=LSTM_FILTER_A,
        )
        nn_obj.load_saved_policy(torch.load(LSTM_MODEL_PATH, map_location=torch.device('cpu')))
        nn_obj.eval()
        print('load parameters successfully!!')
        return nn_obj

    elif NN_TYPE == 'lstm_pd':
        from networks.lstm_pd import LSTMNetworkPD
        print(f"\n{'='*80}")
        print(f"加载 LSTMNetworkPD: {LSTM_PD_MODEL_PATH}")
        print(f"kp={kp}, kd={kd}")
        print(f"滤波器: b={LSTM_FILTER_B}, a={LSTM_FILTER_A}")
        print(f"{'='*80}")

        nn_obj = LSTMNetworkPD(
            kp=kp, kd=kd,
            b=LSTM_FILTER_B, a=LSTM_FILTER_A,
        )
        nn_obj.load_saved_policy(torch.load(LSTM_PD_MODEL_PATH, map_location=torch.device('cpu')))
        nn_obj.eval()
        print('load parameters successfully!!')
        return nn_obj

    else:
        raise ValueError(f"不支持的 NN_TYPE: {NN_TYPE}，应为 'dnn', 'lstm', 'lstm_leg_dcp' 或 'lstm_pd'")


def _parse_dnn_filter_config():
    """解析DNN专用的滤波器配置，返回参数字典"""
    params = {}

    # 求导前平滑滤波器
    derivative_smooth_filter_type = DERIVATIVE_SMOOTH_FILTER_NAME if USE_VELOCITY_FROM_DERIVATIVE else None
    params['derivative_smooth_filter_type'] = derivative_smooth_filter_type
    params['derivative_smooth_filter_b'] = None
    params['derivative_smooth_filter_a'] = None

    if FILTER_CONFIG_MODE == 'preset':
        print(f"[DNN滤波器] 预定义模式 (preset)")
        params['input_pos_filter_type'] = INPUT_POS_FILTER_NAME
        params['input_pos_filter_b'] = None
        params['input_pos_filter_a'] = None
        params['input_vel_filter_type'] = INPUT_VEL_FILTER_NAME
        params['input_vel_filter_b'] = None
        params['input_vel_filter_a'] = None
        params['vel_filter_type'] = VEL_FILTER_NAME
        params['vel_filter_b'] = None
        params['vel_filter_a'] = None
        params['ref_filter_type'] = REF_FILTER_NAME
        params['ref_filter_b'] = None
        params['ref_filter_a'] = None
        params['torque_filter_type'] = TORQUE_FILTER_NAME
        params['torque_filter_b'] = None
        params['torque_filter_a'] = None
        params['torque_filter_enable'] = (TORQUE_FILTER_NAME is not None)

    elif FILTER_CONFIG_MODE == 'custom':
        print(f"[DNN滤波器] 自定义截止频率模式 (custom)")
        vel_b, vel_a = compute_iir_coeffs(VEL_FILTER_CUTOFF, VEL_FILTER_TYPE, FILTER_SAMPLE_RATE, VEL_FILTER_ORDER)
        ref_b, ref_a = compute_iir_coeffs(REF_FILTER_CUTOFF, REF_FILTER_TYPE, FILTER_SAMPLE_RATE, REF_FILTER_ORDER)

        params['input_pos_filter_type'] = None
        params['input_pos_filter_b'] = None
        params['input_pos_filter_a'] = None
        params['input_vel_filter_type'] = None
        params['input_vel_filter_b'] = None
        params['input_vel_filter_a'] = None
        params['vel_filter_type'] = None
        params['vel_filter_b'] = vel_b
        params['vel_filter_a'] = vel_a
        params['ref_filter_type'] = None
        params['ref_filter_b'] = ref_b
        params['ref_filter_a'] = ref_a

        if TORQUE_FILTER_CUTOFF > 0:
            torque_b, torque_a = compute_iir_coeffs(TORQUE_FILTER_CUTOFF, TORQUE_FILTER_TYPE, FILTER_SAMPLE_RATE, TORQUE_FILTER_ORDER)
            params['torque_filter_type'] = None
            params['torque_filter_b'] = torque_b
            params['torque_filter_a'] = torque_a
            params['torque_filter_enable'] = True
        else:
            params['torque_filter_type'] = None
            params['torque_filter_b'] = None
            params['torque_filter_a'] = None
            params['torque_filter_enable'] = False

    elif FILTER_CONFIG_MODE == 'coeffs':
        print(f"[DNN滤波器] 自定义系数模式 (coeffs)")
        params['input_pos_filter_type'] = None
        params['input_pos_filter_b'] = None
        params['input_pos_filter_a'] = None
        params['input_vel_filter_type'] = None
        params['input_vel_filter_b'] = None
        params['input_vel_filter_a'] = None
        params['vel_filter_type'] = None
        params['vel_filter_b'] = VEL_FILTER_B_CUSTOM
        params['vel_filter_a'] = VEL_FILTER_A_CUSTOM
        params['ref_filter_type'] = None
        params['ref_filter_b'] = REF_FILTER_B_CUSTOM
        params['ref_filter_a'] = REF_FILTER_A_CUSTOM
        params['torque_filter_type'] = None
        params['torque_filter_b'] = TORQUE_FILTER_B_CUSTOM if TORQUE_FILTER_ENABLE_CUSTOM else None
        params['torque_filter_a'] = TORQUE_FILTER_A_CUSTOM if TORQUE_FILTER_ENABLE_CUSTOM else None
        params['torque_filter_enable'] = TORQUE_FILTER_ENABLE_CUSTOM

    else:
        raise ValueError(f"不支持的配置模式: {FILTER_CONFIG_MODE}")

    return params


# ========= 加载NN =========
dnn = load_network()

L_Ctl, R_Ctl = 1.0, 1.0
kcontrol = 0.1

# ========= 日志文件 =========
ts = datetime.datetime.now()
root = './output/PI5_' + motion_type + '-'
logf = root + ts.strftime('%Y%m%d-%H%M%S') + '.csv'

csv_header = [
    'Time_ms', 'imu_LTx', 'imu_RTx', 'imu_Lvel', 'imu_Rvel',
    'L_command_actuator', 'R_command_actuator',
    'raw_LExoTorque', 'raw_RExoTorque',
    'filtered_LExoTorque', 'filtered_RExoTorque',
    'L_P', 'L_D', 'R_P', 'R_D',
    'scale_runtime', 'torque_delay_ms',
    'tag',
]


# ========= 串口工具函数 =========
def cksum8(buf: bytes) -> int:
    return sum(buf) & 0xFF


# ---- GUI透传协议常量 ----
RPI_PT_MAGIC = (0x52, 0x4C)   # 'R''L'
RPI_PT_VERSION = 0x01
RPI_PT_CMD_APPLY = 0x01
RPI_FILTER_TYPE_MAP = {
    1: 'butterworth',
    2: 'bessel',
    3: 'chebyshev2',
}
RPI_FILTER_TYPE_REV = {v: k for k, v in RPI_FILTER_TYPE_MAP.items()}  # 反向映射
RPI_FILTER_EN_VEL    = 0x01
RPI_FILTER_EN_REF    = 0x02
RPI_FILTER_EN_TORQUE = 0x04

# ---- NN_TYPE 编码 (用于上行状态) ----
NN_TYPE_CODE = {'dnn': 0, 'lstm': 1, 'lstm_leg_dcp': 2, 'lstm_pd': 3}

# ---- 状态发送间隔 ----
STATUS_SEND_INTERVAL = 50  # 每50帧发送一次 (0.5s @100Hz)


class IdentityFilter:
    def filter(self, x):
        return x


def parse_runtime_cfg(payload: bytes):
    """Parse GUI passthrough payload (40B). Returns dict or None."""
    if len(payload) < 20:
        return None
    if payload[0] != RPI_PT_MAGIC[0] or payload[1] != RPI_PT_MAGIC[1]:
        return None
    if payload[2] != RPI_PT_VERSION or payload[3] != RPI_PT_CMD_APPLY:
        return None

    filter_code = int(payload[4])
    filter_order = int(payload[5]) if payload[5] > 0 else 2
    filter_en_mask = int(payload[6]) if len(payload) > 6 else (RPI_FILTER_EN_VEL | RPI_FILTER_EN_REF)
    scale = float(struct.unpack_from('<f', payload, 8)[0])
    delay_ms = float(struct.unpack_from('<f', payload, 12)[0])
    cutoff_hz = float(struct.unpack_from('<f', payload, 16)[0])

    if not math.isfinite(scale) or not math.isfinite(delay_ms) or not math.isfinite(cutoff_hz):
        return None

    scale = max(0.0, min(3.0, scale))
    delay_ms = max(0.0, min(MAX_RUNTIME_DELAY_MS, delay_ms))
    cutoff_hz = max(0.5, min(CTRL_HZ * 0.45, cutoff_hz))
    filter_order = max(1, min(6, filter_order))

    enable_vr = bool(filter_en_mask & (RPI_FILTER_EN_VEL | RPI_FILTER_EN_REF))
    return {
        "filter_code": filter_code,
        "filter_order": filter_order,
        "scale": scale,
        "delay_ms": delay_ms,
        "cutoff_hz": cutoff_hz,
        "enable_vel": enable_vr,
        "enable_ref": enable_vr,
        "enable_torque": bool(filter_en_mask & RPI_FILTER_EN_TORQUE),
    }


def apply_runtime_filter_to_dnn(dnn_obj, filter_code, cutoff_hz, filter_order, enable_vel, enable_ref, enable_torque):
    """Rebuild runtime filters (仅DNN模式有效)."""
    if NN_TYPE != 'dnn':
        return False

    filter_type = RPI_FILTER_TYPE_MAP.get(filter_code)
    if filter_type is None:
        print(f"[RPi CFG] Unknown filter code: {filter_code}")
        return False

    kwargs = dict(cutoff=float(cutoff_hz), order=int(filter_order),
                  filter_type=filter_type, sample_rate=float(CTRL_HZ))
    try:
        dnn_obj.left_vel_filter = create_filter(**kwargs) if enable_vel else IdentityFilter()
        dnn_obj.right_vel_filter = create_filter(**kwargs) if enable_vel else IdentityFilter()
        dnn_obj.left_ref_filter = create_filter(**kwargs) if enable_ref else IdentityFilter()
        dnn_obj.right_ref_filter = create_filter(**kwargs) if enable_ref else IdentityFilter()
        dnn_obj.torque_filter_enable = bool(enable_torque)
        if enable_torque:
            dnn_obj.left_torque_filter = create_filter(**kwargs)
            dnn_obj.right_torque_filter = create_filter(**kwargs)
        else:
            dnn_obj.left_torque_filter = None
            dnn_obj.right_torque_filter = None
    except Exception as exc:
        print(f"[RPi CFG] Failed: {exc}")
        return False

    print(f"[RPi CFG] Filter applied: {filter_type} {cutoff_hz:.2f}Hz order={filter_order}")
    return True


def apply_runtime_filter_to_lstm(nn_obj, filter_code, cutoff_hz, filter_order, enable_torque):
    """Rebuild runtime torque pre-filter for LSTM/LSTMLegDcp networks."""
    if NN_TYPE == 'dnn':
        return False

    if not enable_torque:
        nn_obj.left_exo_filter = IdentityFilter()
        nn_obj.right_exo_filter = IdentityFilter()
        print(f"[RPi CFG] LSTM torque filter DISABLED")
        return True

    filter_type = RPI_FILTER_TYPE_MAP.get(filter_code)
    if filter_type is None:
        print(f"[RPi CFG] Unknown filter code: {filter_code}")
        return False

    try:
        b, a = compute_iir_coeffs(float(cutoff_hz), filter_type,
                                   float(CTRL_HZ), int(filter_order))
        from filter_library import IIRFilter
        nn_obj.left_exo_filter = IIRFilter(b=b, a=a)
        nn_obj.right_exo_filter = IIRFilter(b=b, a=a)
        nn_obj.b = b
        nn_obj.a = a
    except Exception as exc:
        print(f"[RPi CFG] LSTM filter rebuild failed: {exc}")
        return False

    print(f"[RPi CFG] LSTM torque filter: {filter_type} {cutoff_hz:.2f}Hz order={filter_order}")
    return True


def read_packet(ser: serial.Serial):
    """
    Return dict or None:
      {'type':'imu', 'Lpos':..., 'Rpos':..., 'Lvel':..., 'Rvel':..., 'exo_delay':..., 'logtag':...}
      {'type':'cfg', 'payload': bytes}
    """
    if not PI_USE_BINARY:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            return None
        try:
            vals = tuple(map(float, line.split(',')))
            if len(vals) < 6:
                return None
            return {
                'type': 'imu',
                'Lpos': vals[0], 'Rpos': vals[1],
                'Lvel': vals[2], 'Rvel': vals[3],
                'exo_delay': vals[4], 'logtag': str(vals[5]),
            }
        except ValueError:
            return None

    # Binary mode
    while True:
        h = ser.read(1)
        if not h:
            return None
        if h == b'\xA5' and ser.read(1) == b'\x5A':
            break

    ln = ser.read(1)
    if not ln:
        return None
    total_len = int(ln[0])
    if total_len < 1 or total_len > 96:
        return None

    typ = ser.read(1)
    if not typ:
        return None
    typ_v = typ[0]

    payload_len = total_len - 1
    payload = ser.read(payload_len)
    if len(payload) != payload_len:
        return None

    chk = ser.read(1)
    if not chk or chk[0] != cksum8(typ + payload):
        return None

    if typ_v == 0x01:
        if payload_len != 31:
            return None
        Lpos, Rpos, Lvel, Rvel, exo_delay, logtag = struct.unpack('<fffff11s', payload)
        return {
            'type': 'imu',
            'Lpos': Lpos, 'Rpos': Rpos,
            'Lvel': Lvel, 'Rvel': Rvel,
            'exo_delay': exo_delay,
            'logtag': logtag.decode(errors='ignore').rstrip('\x00'),
        }

    if typ_v == 0x02:
        return {'type': 'cfg', 'payload': payload}

    return None


def send_torque(ser: serial.Serial, tau_L: float, tau_R: float, L_p, L_d, R_p, R_d):
    """向 Teensy 发送 6×float32 (小端), header=AA 55"""
    payload = struct.pack('<ffffff', tau_L, tau_R, L_p, L_d, R_p, R_d)
    packet = b'\xAA\x55' + payload
    ser.write(packet)
    ser.flush()


def send_status(ser: serial.Serial, filter_source, filter_type_code,
                filter_order, enable_mask, cutoff_hz, scale, delay_ms):
    """
    向 Teensy 发送 RPi 状态 (header=AA 56), Teensy 转存到 rpi_uplink_buf → BLE → GUI
    40 bytes payload:
      [0-1] magic 'RL'
      [2]   version
      [3]   nn_type (0=dnn,1=lstm,2=lstm_leg_dcp)
      [4]   filter_source (0=base, 1=runtime_override)
      [5]   filter_type_code (1=butter, 2=bessel, 3=cheby2)
      [6]   filter_order
      [7]   enable_mask (bit0=vel, bit1=ref, bit2=torque)
      [8..11]  cutoff_hz  float32
      [12..15] scale      float32
      [16..19] delay_ms   float32
      [20..39] reserved
    """
    buf = bytearray(40)
    buf[0] = 0x52  # 'R'
    buf[1] = 0x4C  # 'L'
    buf[2] = 0x01  # version
    buf[3] = NN_TYPE_CODE.get(NN_TYPE, 0) & 0xFF
    buf[4] = int(filter_source) & 0xFF
    buf[5] = int(filter_type_code) & 0xFF
    buf[6] = int(filter_order) & 0xFF
    buf[7] = int(enable_mask) & 0xFF
    struct.pack_into('<f', buf, 8, float(cutoff_hz))
    struct.pack_into('<f', buf, 12, float(scale))
    struct.pack_into('<f', buf, 16, float(delay_ms))
    packet = b'\xAA\x56' + bytes(buf)
    ser.write(packet)
    ser.flush()


# ========= 主程序 =========
def main():
    ser = serial.Serial(SER_DEV, BAUDRATE, timeout=TIMEOUT)
    ser.reset_input_buffer()
    start = time.time()
    last_flush = time.time()

    runtime_scale = 1.0
    runtime_delay_ms = 0.0
    last_gui_tag = ""
    frame_count = 0
    status_dirty = True  # 启动时立即发送一次状态

    # 当前滤波器状态跟踪 (用于上行状态)
    cur_filter_source = 0      # 0=base, 1=runtime_override
    cur_filter_type_code = 0   # 0=preset/N/A
    cur_filter_order = 2
    cur_enable_mask = 0x00
    cur_cutoff_hz = 0.0

    # 初始化基础滤波器状态 (LSTM模式: torque filter always on)
    if NN_TYPE != 'dnn':
        cur_enable_mask = RPI_FILTER_EN_TORQUE
        cur_filter_type_code = 1  # Butterworth (LSTM默认使用Butterworth)
        cur_filter_order = 2
        # 从 LSTM_FILTER_B[1] (=2*b0 for Butterworth 2nd order) 反推截止频率
        # Butterworth 2nd order: b0 = ((1-cos(wc))/2)^... 精确反推复杂，
        # 使用 scipy 反算:
        try:
            from scipy.signal import butter
            # 尝试匹配: 遍历常用截止频率找最接近的
            best_fc, best_err = 20.0, 1e9
            for fc in [3, 6, 12, 15, 20, 25, 30]:
                tb, _ = butter(2, fc / (CTRL_HZ / 2))
                err = float(np.sum((tb - LSTM_FILTER_B)**2))
                if err < best_err:
                    best_err = err
                    best_fc = fc
            cur_cutoff_hz = float(best_fc)
        except Exception:
            cur_cutoff_hz = 20.0  # fallback
    else:
        # DNN: 根据preset配置
        if FILTER_CONFIG_MODE == 'preset':
            cur_enable_mask = (RPI_FILTER_EN_VEL | RPI_FILTER_EN_REF)
            if TORQUE_FILTER_NAME:
                cur_enable_mask |= RPI_FILTER_EN_TORQUE

    print(f"\n[启动] NN_TYPE={NN_TYPE}, kp={kp}, kd={kd}")
    print(f"[启动] 串口={SER_DEV}, 波特率={BAUDRATE}")
    print(f"[启动] 日志={logf}\n")

    with open(logf, 'w', newline='') as fcsv:
        wr = csv.DictWriter(fcsv, fieldnames=csv_header)
        wr.writeheader()

        while True:
            pkt = read_packet(ser)
            if pkt is None:
                continue

            # ---- GUI运行时配置 (DNN模式支持滤波器热更新) ----
            if pkt['type'] == 'cfg':
                cfg = parse_runtime_cfg(pkt['payload'])
                if cfg is None:
                    continue

                runtime_scale = cfg['scale']
                runtime_delay_ms = cfg['delay_ms']

                if NN_TYPE == 'dnn':
                    apply_runtime_filter_to_dnn(
                        dnn, cfg['filter_code'], cfg['cutoff_hz'], cfg['filter_order'],
                        cfg['enable_vel'], cfg['enable_ref'], cfg['enable_torque'],
                    )
                else:
                    apply_runtime_filter_to_lstm(
                        dnn, cfg['filter_code'], cfg['cutoff_hz'], cfg['filter_order'],
                        cfg['enable_torque'],
                    )

                # 更新滤波器状态跟踪
                cur_filter_source = 1  # runtime_override
                cur_filter_type_code = cfg['filter_code']
                cur_filter_order = cfg['filter_order']
                cur_cutoff_hz = cfg['cutoff_hz']
                en = 0
                if cfg['enable_vel']:  en |= RPI_FILTER_EN_VEL
                if cfg['enable_ref']:  en |= RPI_FILTER_EN_REF
                if cfg['enable_torque']: en |= RPI_FILTER_EN_TORQUE
                cur_enable_mask = en
                status_dirty = True  # 立即回传状态

                delay_buf_L.clear()
                delay_buf_L.extend([0.0] * BUF_SIZE)
                delay_buf_R.clear()
                delay_buf_R.extend([0.0] * BUF_SIZE)

                print(f"[RPi CFG] scale={runtime_scale:.3f}, delay={runtime_delay_ms:.1f}ms")
                continue

            if pkt['type'] != 'imu':
                continue

            Lpos = pkt['Lpos']
            Rpos = pkt['Rpos']
            Lvel = pkt['Lvel']
            Rvel = pkt['Rvel']
            exo_delay = pkt['exo_delay']
            logtag = pkt['logtag']
            if isinstance(logtag, str) and logtag:
                last_gui_tag = logtag
            tag_to_log = last_gui_tag

            # ========= IMU符号处理 =========
            if INVERT_IMU_SIGN:
                Lpos = -Lpos
                Rpos = -Rpos
                Lvel = -Lvel
                Rvel = -Rvel

            exo_delay = max(0.0, min(MAX_RUNTIME_DELAY_MS, float(runtime_delay_ms)))

            now = time.time() - start

            # ========= NN 推理 =========
            action = dnn.generate_assistance(Lpos, Rpos, Lvel, Rvel)
            L_cmd = dnn.hip_torque_L
            R_cmd = dnn.hip_torque_R
            L_p, L_d, R_p, R_d = dnn.L_p, dnn.L_d, dnn.R_p, dnn.R_d

            # ---- 获取滤波后的torque ----
            if hasattr(dnn, 'filtered_hip_torque_L'):
                filter_L_cmd = dnn.filtered_hip_torque_L
                filter_R_cmd = dnn.filtered_hip_torque_R
            else:
                filter_L_cmd = L_cmd
                filter_R_cmd = R_cmd

            # ---- delay buffer ----
            delay_frames = int(round(exo_delay / dt_ms))
            delay_frames = max(0, min(delay_frames, BUF_SIZE - 1))

            delay_input_L = filter_L_cmd if USE_FILTERED_FOR_DELAY else L_cmd
            delay_input_R = filter_R_cmd if USE_FILTERED_FOR_DELAY else R_cmd

            delay_buf_L.appendleft(delay_input_L)
            delay_buf_R.appendleft(delay_input_R)
            L_cmd_shifted = delay_buf_L[delay_frames]
            R_cmd_shifted = delay_buf_R[delay_frames]

            L_cmd_final = L_cmd_shifted * runtime_scale
            R_cmd_final = R_cmd_shifted * runtime_scale

            # ---- 发送给 Teensy ----
            send_torque(ser, L_cmd_final, R_cmd_final, L_p, L_d, R_p, R_d)

            # ---- 定期发送RPi状态 (供GUI显示) ----
            frame_count += 1
            if status_dirty or (frame_count % STATUS_SEND_INTERVAL == 0):
                send_status(ser, cur_filter_source, cur_filter_type_code,
                            cur_filter_order, cur_enable_mask, cur_cutoff_hz,
                            runtime_scale, runtime_delay_ms)
                status_dirty = False

            # ---- 日志 ----
            data_dict = {
                'Time_ms': f'{now:.4f}',
                'imu_LTx': f'{Lpos:.4f}',
                'imu_RTx': f'{Rpos:.4f}',
                'imu_Lvel': f'{Lvel:.4f}',
                'imu_Rvel': f'{Rvel:.4f}',
                'L_command_actuator': f'{L_cmd_final:.6f}',
                'R_command_actuator': f'{R_cmd_final:.6f}',
                'raw_LExoTorque': f'{L_cmd:.6f}',
                'raw_RExoTorque': f'{R_cmd:.6f}',
                'filtered_LExoTorque': f'{filter_L_cmd:.6f}',
                'filtered_RExoTorque': f'{filter_R_cmd:.6f}',
                'L_P': f'{L_p:.6f}',
                'L_D': f'{L_d:.6f}',
                'R_P': f'{R_p:.6f}',
                'R_D': f'{R_d:.6f}',
                'scale_runtime': f'{runtime_scale:.4f}',
                'torque_delay_ms': f'{exo_delay:.3f}',
                'tag': tag_to_log,
            }

            wr.writerow(data_dict)
            if time.time() - last_flush > 0.5:
                fcsv.flush()
                last_flush = time.time()

            # ---- 控制台输出 ----
            print(f'| time:{now:6.2f}s | Lθ:{Lpos:7.2f}° | Rθ:{Rpos:7.2f}° | '
                  f'Lω:{Lvel:7.2f} | Rω:{Rvel:7.2f} | '
                  f'τL:{L_cmd_final:6.2f} | τR:{R_cmd_final:6.2f} | scale:{runtime_scale:4.2f} | '
                  f'Rp:{R_p:6.2f} | Rd:{R_d:6.2f} ')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nQuit.')
