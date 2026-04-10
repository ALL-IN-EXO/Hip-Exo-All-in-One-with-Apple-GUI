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
# +1: 同时覆盖 0ms 档位和 MAX_RUNTIME_DELAY_MS 档位 (100Hz 下即 0..100 帧)
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
# --nn lstm: 20Hz (legacy default)
LSTM_FILTER_B = np.array([0.0913, 0.1826, 0.0913])   # 20Hz
LSTM_FILTER_A = np.array([1.0, -0.9824, 0.3477])
# --nn lstm_leg_dcp: 5Hz 2nd-order Butterworth (computed at import time)
from filter_library import compute_iir_coeffs as _cic
LSTM_LEGDCP_FILTER_B, LSTM_LEGDCP_FILTER_A = _cic(5.0, 'butterworth', float(CTRL_HZ), 2)
del _cic

# ╔══════════════════════════════════════════════════════════════════╗
# ║                 以下为运行逻辑，一般不需要改                       ║
# ╚══════════════════════════════════════════════════════════════════╝

from filter_library import create_filter, compute_iir_coeffs, RECOMMENDED_FILTERS

# 延迟环形队列
delay_buf_L = deque([0.0] * BUF_SIZE, maxlen=BUF_SIZE)
delay_buf_R = deque([0.0] * BUF_SIZE, maxlen=BUF_SIZE)

# ---- Auto Delay Optimization (RL only) ----
AUTO_TARGET_RATIO = 0.95
AUTO_UPDATE_INTERVAL_S = 1.0
AUTO_DWELL_S = 2.0
AUTO_SCAN_HALF_RANGE_MS = 100.0
AUTO_SCAN_STEP_MS = 10.0
AUTO_MAX_STEP_MS = 20.0
AUTO_WINDOW_CYCLES = 6.0
AUTO_WINDOW_MIN_S = 4.0
AUTO_WINDOW_MAX_S = 12.0
AUTO_WINDOW_FALLBACK_S = 8.0
AUTO_VALID_MIN_MEAN_ABS_VEL_DPS = 8.0
AUTO_VALID_MIN_ABS_POWER_W = 2.0
AUTO_HIST_MARGIN_S = 2.0
AUTO_MAX_DELAY_S = MAX_RUNTIME_DELAY_MS / 1000.0
AUTO_HIST_FRAMES = int(round((AUTO_WINDOW_MAX_S + AUTO_MAX_DELAY_S + AUTO_HIST_MARGIN_S) * CTRL_HZ))


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
        print(f"滤波器: b={LSTM_LEGDCP_FILTER_B}, a={LSTM_LEGDCP_FILTER_A}")
        print(f"{'='*80}")

        nn_obj = LSTMNetworkLegDcp(
            kp=kp, kd=kd,
            b=LSTM_LEGDCP_FILTER_B, a=LSTM_LEGDCP_FILTER_A,
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


def estimate_gait_freq_hz(angle_hist: np.ndarray, fs: float) -> float:
    """
    Estimate gait frequency from hip angle autocorrelation.
    Returns 0.0 if insufficient signal quality.
    """
    x = np.asarray(angle_hist, dtype=np.float32)
    if x.size < int(2.0 * fs):
        return 0.0
    if not np.all(np.isfinite(x)):
        return 0.0
    x = x - float(np.mean(x))
    if float(np.std(x)) < 1.0:
        return 0.0

    ac = np.correlate(x, x, mode='full')[x.size - 1:]
    if ac.size < 8:
        return 0.0
    ac[0] = 0.0

    # Walking range: 0.3~3.0 Hz => lag ~ [fs/3, fs/0.3]
    lag_min = max(1, int(round(fs / 3.0)))
    lag_max = min(ac.size - 1, int(round(fs / 0.3)))
    if lag_max <= lag_min:
        return 0.0

    seg = ac[lag_min:lag_max + 1]
    k = int(np.argmax(seg))
    lag = lag_min + k
    if lag <= 0 or ac[lag] <= 0:
        return 0.0

    freq = float(fs / lag)
    if freq < 0.3 or freq > 3.0:
        return 0.0
    return freq


def auto_window_seconds(gait_freq_hz: float) -> float:
    if gait_freq_hz > 0.0:
        t_win = AUTO_WINDOW_CYCLES / gait_freq_hz
    else:
        t_win = AUTO_WINDOW_FALLBACK_S
    return float(max(AUTO_WINDOW_MIN_S, min(AUTO_WINDOW_MAX_S, t_win)))


def compute_leg_power_metrics(tau_src: np.ndarray, vel_dps: np.ndarray, delay_frames: int,
                              window_frames: int, scale: float):
    """
    Evaluate one leg under a candidate delay.
    tau_src: torque source before delay (Nm), same stream used by delay buffer.
    vel_dps: angular velocity (deg/s).
    """
    n = int(min(len(tau_src), len(vel_dps)))
    if n < 8:
        return None

    start = max(0, n - int(window_frames))
    idx = np.arange(start, n, dtype=np.int32)
    src_idx = idx - int(delay_frames)
    valid = src_idx >= 0
    if not np.any(valid):
        return None

    idx = idx[valid]
    src_idx = src_idx[valid]
    tau = tau_src[src_idx] * float(scale)
    vel = vel_dps[idx]
    power = tau * vel * (np.pi / 180.0)

    if power.size < 8:
        return None

    pos = power[power > 0.0]
    neg = power[power < 0.0]

    t_total = float(power.size) / float(CTRL_HZ)
    if t_total <= 1e-6:
        return None

    w_pos = float(np.sum(pos)) / float(CTRL_HZ)
    w_neg_abs = float(np.sum(np.abs(neg))) / float(CTRL_HZ)
    w_tot = w_pos + w_neg_abs
    ratio = (w_pos / w_tot) if w_tot > 1e-9 else 0.0
    pos_per_s = w_pos / t_total
    neg_per_s = -w_neg_abs / t_total
    abs_power_per_s = w_tot / t_total
    mean_abs_vel = float(np.mean(np.abs(vel)))

    return {
        'ratio': ratio,
        'pos_per_s': pos_per_s,
        'neg_per_s': neg_per_s,
        'abs_power_per_s': abs_power_per_s,
        'mean_abs_vel': mean_abs_vel,
    }


def evaluate_delay_candidate(delay_ms: float, tau_src_L: np.ndarray, tau_src_R: np.ndarray,
                             vel_L: np.ndarray, vel_R: np.ndarray,
                             window_frames: int):
    d_frames = int(round(float(delay_ms) / dt_ms))
    mL = compute_leg_power_metrics(tau_src_L, vel_L, d_frames, window_frames, 1.0)
    mR = compute_leg_power_metrics(tau_src_R, vel_R, d_frames, window_frames, 1.0)
    if mL is None or mR is None:
        return None

    ratio = 0.5 * (mL['ratio'] + mR['ratio'])
    pos_per_s = mL['pos_per_s'] + mR['pos_per_s']
    neg_per_s = mL['neg_per_s'] + mR['neg_per_s']
    abs_power_per_s = mL['abs_power_per_s'] + mR['abs_power_per_s']
    mean_abs_vel = 0.5 * (mL['mean_abs_vel'] + mR['mean_abs_vel'])
    return {
        'delay_ms': float(delay_ms),
        'ratio': float(ratio),
        'pos_per_s': float(pos_per_s),
        'neg_per_s': float(neg_per_s),
        'abs_power_per_s': float(abs_power_per_s),
        'mean_abs_vel': float(mean_abs_vel),
    }


def pick_best_delay_candidate(candidates, current_delay_ms: float, target_ratio: float):
    if not candidates:
        return None

    feasible = [c for c in candidates if c['ratio'] >= float(target_ratio)]
    if feasible:
        return max(
            feasible,
            key=lambda c: (c['pos_per_s'], c['ratio'], -abs(c['delay_ms'] - current_delay_ms))
        )
    return max(
        candidates,
        key=lambda c: (c['ratio'], c['pos_per_s'], -abs(c['delay_ms'] - current_delay_ms))
    )


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
    auto_flags = int(payload[20]) if len(payload) > 20 else 0

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
        "auto_delay_enable": bool(auto_flags & 0x01),
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
      {'type':'imu', 'Lpos':..., 'Rpos':..., 'Lvel':..., 'Rvel':..., 'logtag':...}
      {'type':'cfg', 'payload': bytes}
    """
    if not PI_USE_BINARY:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            return None
        try:
            vals = tuple(map(float, line.split(',')))
            if len(vals) < 5:
                return None
            return {
                'type': 'imu',
                'Lpos': vals[0], 'Rpos': vals[1],
                'Lvel': vals[2], 'Rvel': vals[3],
                'logtag': str(vals[4]),
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
        if payload_len != 27:
            return None
        Lpos, Rpos, Lvel, Rvel, logtag = struct.unpack('<ffff11s', payload)
        return {
            'type': 'imu',
            'Lpos': Lpos, 'Rpos': Rpos,
            'Lvel': Lvel, 'Rvel': Rvel,
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
                filter_order, enable_mask, cutoff_hz, scale, delay_ms,
                auto_enable=False, auto_motion_valid=False,
                power_ratio=0.0, pos_per_s=0.0, neg_per_s=0.0, best_delay_ms=0.0):
    """
    向 Teensy 发送 RPi 状态 (header=AA 56), Teensy 转存到 rpi_uplink_buf → BLE → GUI
    40 bytes payload:
      [0-1] magic 'RL'
      [2]   version
      [3]   nn_type (0=dnn,1=lstm,2=lstm_leg_dcp,3=lstm_pd)
      [4]   filter_source (0=base, 1=runtime_override)
      [5]   filter_type_code (1=butter, 2=bessel, 3=cheby2)
      [6]   filter_order
      [7]   enable_mask (bit0=vel, bit1=ref, bit2=torque)
      [8..11]  cutoff_hz  float32
      [12..15] scale      float32
      [16..19] delay_ms   float32
      [20] auto_flags (bit0=auto_enable, bit1=motion_valid)
      [21..23] reserved
      [24..27] power_ratio float32
      [28..31] pos_per_s   float32
      [32..35] neg_per_s   float32
      [36..39] best_delay_ms float32
    """
    buf = bytearray(40)
    buf[0] = 0x52  # 'R'
    buf[1] = 0x4C  # 'L'
    buf[2] = 0x02  # version
    buf[3] = NN_TYPE_CODE.get(NN_TYPE, 0) & 0xFF
    buf[4] = int(filter_source) & 0xFF
    buf[5] = int(filter_type_code) & 0xFF
    buf[6] = int(filter_order) & 0xFF
    buf[7] = int(enable_mask) & 0xFF
    struct.pack_into('<f', buf, 8, float(cutoff_hz))
    struct.pack_into('<f', buf, 12, float(scale))
    struct.pack_into('<f', buf, 16, float(delay_ms))
    auto_flags = 0
    if auto_enable:
        auto_flags |= 0x01
    if auto_motion_valid:
        auto_flags |= 0x02
    buf[20] = auto_flags & 0xFF
    struct.pack_into('<f', buf, 24, float(power_ratio))
    struct.pack_into('<f', buf, 28, float(pos_per_s))
    struct.pack_into('<f', buf, 32, float(neg_per_s))
    struct.pack_into('<f', buf, 36, float(best_delay_ms))
    packet = b'\xAA\x56' + bytes(buf)
    ser.write(packet)
    ser.flush()


# ========= 主程序 =========
def main():
    ser = serial.Serial(SER_DEV, BAUDRATE, timeout=TIMEOUT)
    ser.reset_input_buffer()
    start = time.time()
    last_flush = time.time()

    # runtime_scale: GUI 运行时下发的最终扭矩倍率 (乘在 delay 后命令上)
    runtime_scale = 1.0
    # LegDcp default delay: 100ms; others: 0ms
    runtime_delay_ms = 100.0 if NN_TYPE == 'lstm_leg_dcp' else 0.0
    auto_delay_enable = False
    auto_last_eval_ts = 0.0
    auto_last_change_ts = 0.0
    auto_cur_ratio = 0.0
    auto_cur_pos_per_s = 0.0
    auto_cur_neg_per_s = 0.0
    auto_best_delay_ms = runtime_delay_ms
    auto_motion_valid = False
    auto_gait_freq_hz = 0.0
    auto_window_s = AUTO_WINDOW_FALLBACK_S

    hist_tau_src_L = deque(maxlen=AUTO_HIST_FRAMES)
    hist_tau_src_R = deque(maxlen=AUTO_HIST_FRAMES)
    hist_vel_L = deque(maxlen=AUTO_HIST_FRAMES)
    hist_vel_R = deque(maxlen=AUTO_HIST_FRAMES)
    hist_ang_L = deque(maxlen=AUTO_HIST_FRAMES)
    hist_ang_R = deque(maxlen=AUTO_HIST_FRAMES)

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
        cur_filter_type_code = 1  # Butterworth
        cur_filter_order = 2
        if NN_TYPE == 'lstm_leg_dcp':
            cur_cutoff_hz = 5.0   # LegDcp default: 5Hz
        else:
            cur_cutoff_hz = 20.0  # lstm / lstm_pd default: 20Hz
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
                prev_auto_delay_enable = auto_delay_enable
                auto_delay_enable = bool(cfg.get('auto_delay_enable', auto_delay_enable))
                if not auto_delay_enable:
                    auto_motion_valid = False
                    auto_cur_ratio = 0.0
                    auto_cur_pos_per_s = 0.0
                    auto_cur_neg_per_s = 0.0
                    auto_best_delay_ms = runtime_delay_ms

                # cfg change invalidates history (filter/scale/delay 都会让旧窗口数据不再代表当前配置):
                # 清空 history + 重置评估节拍，避免用跨配置数据做评估。
                # 同时：auto False→True 的上升沿触发一次冷启动 dwell，
                # 让新窗口先填满再允许 scan 调整 delay。
                hist_tau_src_L.clear()
                hist_tau_src_R.clear()
                hist_vel_L.clear()
                hist_vel_R.clear()
                hist_ang_L.clear()
                hist_ang_R.clear()
                _now_wall = time.time()
                auto_last_eval_ts = _now_wall
                if auto_delay_enable and not prev_auto_delay_enable:
                    auto_last_change_ts = _now_wall

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

                print(
                    f"[RPi CFG] scale={runtime_scale:.3f}, delay={runtime_delay_ms:.1f}ms, "
                    f"auto_delay={'ON' if auto_delay_enable else 'OFF'}"
                )
                continue

            if pkt['type'] != 'imu':
                continue

            Lpos = pkt['Lpos']
            Rpos = pkt['Rpos']
            Lvel = pkt['Lvel']
            Rvel = pkt['Rvel']
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

            delay_ms = max(0.0, min(MAX_RUNTIME_DELAY_MS, float(runtime_delay_ms)))

            now = time.time() - start

            # ========= NN 推理 =========
            action = dnn.generate_assistance(Lpos, Rpos, Lvel, Rvel)
            L_cmd = dnn.hip_torque_L
            R_cmd = dnn.hip_torque_R
            # L_p/L_d 由各网络直接给出并原样透传到日志与 Teensy。
            # 例如 lstm_pd 中 D 项已带符号(L_d = -dqTd*kd)，总扭矩是 L_p + L_d。
            L_p, L_d, R_p, R_d = dnn.L_p, dnn.L_d, dnn.R_p, dnn.R_d

            # ---- 获取滤波后的torque ----
            if hasattr(dnn, 'filtered_hip_torque_L'):
                filter_L_cmd = dnn.filtered_hip_torque_L
                filter_R_cmd = dnn.filtered_hip_torque_R
            else:
                filter_L_cmd = L_cmd
                filter_R_cmd = R_cmd

            delay_input_L = filter_L_cmd if USE_FILTERED_FOR_DELAY else L_cmd
            delay_input_R = filter_R_cmd if USE_FILTERED_FOR_DELAY else R_cmd

            # ---- auto-delay history (for ratio/energy evaluation) ----
            hist_tau_src_L.append(float(delay_input_L))
            hist_tau_src_R.append(float(delay_input_R))
            hist_vel_L.append(float(Lvel))
            hist_vel_R.append(float(Rvel))
            hist_ang_L.append(float(Lpos))
            hist_ang_R.append(float(Rpos))

            # ---- auto-delay optimization (1 Hz, does not touch 100 Hz inference path) ----
            now_wall = time.time()
            if auto_delay_enable and (now_wall - auto_last_eval_ts) >= AUTO_UPDATE_INTERVAL_S:
                auto_last_eval_ts = now_wall

                tau_src_L_np = np.asarray(hist_tau_src_L, dtype=np.float32)
                tau_src_R_np = np.asarray(hist_tau_src_R, dtype=np.float32)
                vel_L_np = np.asarray(hist_vel_L, dtype=np.float32)
                vel_R_np = np.asarray(hist_vel_R, dtype=np.float32)
                ang_L_np = np.asarray(hist_ang_L, dtype=np.float32)
                ang_R_np = np.asarray(hist_ang_R, dtype=np.float32)

                gf_L = estimate_gait_freq_hz(ang_L_np, float(CTRL_HZ))
                gf_R = estimate_gait_freq_hz(ang_R_np, float(CTRL_HZ))
                if gf_L > 0.0 and gf_R > 0.0:
                    auto_gait_freq_hz = 0.5 * (gf_L + gf_R)
                else:
                    auto_gait_freq_hz = max(gf_L, gf_R)

                auto_window_s = auto_window_seconds(auto_gait_freq_hz)
                window_frames = max(16, int(round(auto_window_s * CTRL_HZ)))

                cur_eval = evaluate_delay_candidate(
                    runtime_delay_ms, tau_src_L_np, tau_src_R_np,
                    vel_L_np, vel_R_np, window_frames
                )
                if cur_eval is not None:
                    auto_cur_ratio = cur_eval['ratio']
                    auto_cur_pos_per_s = cur_eval['pos_per_s'] * float(runtime_scale)
                    auto_cur_neg_per_s = cur_eval['neg_per_s'] * float(runtime_scale)
                    auto_best_delay_ms = cur_eval['delay_ms']
                    auto_motion_valid = (
                        cur_eval['mean_abs_vel'] >= AUTO_VALID_MIN_MEAN_ABS_VEL_DPS and
                        cur_eval['abs_power_per_s'] >= AUTO_VALID_MIN_ABS_POWER_W
                    )
                else:
                    auto_cur_ratio = 0.0
                    auto_cur_pos_per_s = 0.0
                    auto_cur_neg_per_s = 0.0
                    auto_best_delay_ms = runtime_delay_ms
                    auto_motion_valid = False

                # Dwell 与自适应窗口挂钩：每次改 delay 后至少等 window_s + 1s，
                # 保证评估窗口里 100% 都是新 delay 稳态下的数据，不会被上次 delay
                # 残留的物理响应污染。
                effective_dwell_s = max(AUTO_DWELL_S, auto_window_s + 1.0)
                if auto_motion_valid and (now_wall - auto_last_change_ts) >= effective_dwell_s:
                    d_lo = max(0.0, runtime_delay_ms - AUTO_SCAN_HALF_RANGE_MS)
                    d_hi = min(MAX_RUNTIME_DELAY_MS, runtime_delay_ms + AUTO_SCAN_HALF_RANGE_MS)
                    cand_delays = np.arange(d_lo, d_hi + 0.5 * AUTO_SCAN_STEP_MS, AUTO_SCAN_STEP_MS, dtype=np.float32)
                    if cand_delays.size == 0:
                        cand_delays = np.array([runtime_delay_ms], dtype=np.float32)

                    candidates = []
                    for c_delay in cand_delays:
                        c_eval = evaluate_delay_candidate(
                            float(c_delay), tau_src_L_np, tau_src_R_np,
                            vel_L_np, vel_R_np, window_frames
                        )
                        if c_eval is not None:
                            candidates.append(c_eval)

                    best = pick_best_delay_candidate(candidates, runtime_delay_ms, AUTO_TARGET_RATIO)
                    if best is not None:
                        auto_best_delay_ms = best['delay_ms']
                        step_ms = max(-AUTO_MAX_STEP_MS, min(AUTO_MAX_STEP_MS, auto_best_delay_ms - runtime_delay_ms))
                        if abs(step_ms) >= 0.5:
                            runtime_delay_ms = max(0.0, min(MAX_RUNTIME_DELAY_MS, runtime_delay_ms + step_ms))
                            auto_last_change_ts = now_wall
                            status_dirty = True

            # ---- delay buffer ----
            delay_ms = max(0.0, min(MAX_RUNTIME_DELAY_MS, float(runtime_delay_ms)))
            delay_frames = int(round(delay_ms / dt_ms))
            delay_frames = max(0, min(delay_frames, BUF_SIZE - 1))

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
                            runtime_scale, runtime_delay_ms,
                            auto_enable=auto_delay_enable,
                            auto_motion_valid=auto_motion_valid,
                            power_ratio=auto_cur_ratio,
                            pos_per_s=auto_cur_pos_per_s,
                            neg_per_s=auto_cur_neg_per_s,
                            best_delay_ms=auto_best_delay_ms)
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
                'torque_delay_ms': f'{delay_ms:.3f}',
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
