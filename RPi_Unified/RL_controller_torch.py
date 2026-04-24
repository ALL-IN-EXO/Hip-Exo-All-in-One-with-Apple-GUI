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

import os, serial, struct, time, csv, datetime, math, argparse
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
# 统一通路: raw torque -> (主循环统一torque滤波) -> delay -> scale -> send
USE_FILTERED_FOR_DELAY = True         # 保留兼容标志，当前统一管线始终使用滤波后torque做delay
MAX_RUNTIME_DELAY_MS = 1000.0
# +1: 同时覆盖 0ms 档位和 MAX_RUNTIME_DELAY_MS 档位 (100Hz 下即 0..100 帧)
BUF_SIZE = int(round(MAX_RUNTIME_DELAY_MS / dt_ms)) + 1

# ---- Runtime delay defaults (per NN) ----
# Note:
# - lstm_leg_dcp keeps historical default 100ms
# - lstm_pd uses 200ms baseline so Grid auto-delay starts around expected timing
DEFAULT_RUNTIME_DELAY_MS = {
    'dnn': 0.0,
    'lstm': 0.0,
    'lstm_leg_dcp': 100.0,
    'lstm_pd': 200.0,
}

# ---- CSV 日志 ----
SAVE_VERBOSE_LOG = True

# ---- Zero Mean (仅DNN模式) ----
ENABLE_ZERO_MEAN       = False
ZERO_MEAN_BUFFER_SIZE  = 200
ZERO_MEAN_WARMUP       = 100

# ---- LSTM-PD 输出去均值 (仅NN_TYPE='lstm_pd'时生效) ----
# 作用位置: 滤波后、delay前。用于把扭矩中心拉回 0，缓解正负不对称。
LSTM_PD_ZERO_MEAN_ENABLE = True
LSTM_PD_ZERO_MEAN_WINDOW_S = 6.0
LSTM_PD_ZERO_MEAN_WARMUP_S = 2.0

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
# --nn lstm: legacy default (内部滤波已旁路，系数仅保留兼容注释)
LSTM_FILTER_B = np.array([0.0913, 0.1826, 0.0913])   # 20Hz
LSTM_FILTER_A = np.array([1.0, -0.9824, 0.3477])
# internal filter bypass (all NN): identity
NN_INTERNAL_FILTER_BYPASS_B = np.array([1.0], dtype=np.float64)
NN_INTERNAL_FILTER_BYPASS_A = np.array([1.0], dtype=np.float64)

# Unified torque filter defaults (main-loop pre-delay path)
UNIFIED_TORQUE_FILTER_ENABLE_DEFAULT = True
UNIFIED_TORQUE_FILTER_CODE_DEFAULT = 1      # 1=Butterworth
UNIFIED_TORQUE_FILTER_ORDER_DEFAULT = 2
UNIFIED_TORQUE_FILTER_CUTOFF_DEFAULT = 5.0
# --nn lstm_leg_dcp: 5Hz 2nd-order Butterworth (computed at import time)
from filter_library import compute_iir_coeffs as _cic
LSTM_LEGDCP_FILTER_B, LSTM_LEGDCP_FILTER_A = _cic(5.0, 'butterworth', float(CTRL_HZ), 2)
del _cic

# ╔══════════════════════════════════════════════════════════════════╗
# ║                 以下为运行逻辑，一般不需要改                       ║
# ╚══════════════════════════════════════════════════════════════════╝

from filter_library import create_filter, compute_iir_coeffs, RECOMMENDED_FILTERS, IIRFilter

# 延迟环形队列
delay_buf_L = deque([0.0] * BUF_SIZE, maxlen=BUF_SIZE)
delay_buf_R = deque([0.0] * BUF_SIZE, maxlen=BUF_SIZE)

# ---- Auto Delay Optimization (RL only) ----
AUTO_TARGET_RATIO = 0.95
AUTO_UPDATE_INTERVAL_S = 1.0
AUTO_DWELL_S = 0.5
# True : effective_dwell = max(AUTO_DWELL_S, auto_window_s + 1.0) —— 等窗口 100% 新数据
# False: effective_dwell = AUTO_DWELL_S                           —— 固定常数, 追踪更快
AUTO_DWELL_ADAPTIVE = False
AUTO_SCAN_HALF_RANGE_MS = 100.0
AUTO_SCAN_STEP_MS = 10.0
AUTO_MAX_STEP_MS = 40.0
AUTO_WINDOW_CYCLES = 6.0
AUTO_WINDOW_MIN_S = 8.0
AUTO_WINDOW_MAX_S = 8.0
AUTO_WINDOW_FALLBACK_S = 8.0
AUTO_VALID_MIN_MEAN_ABS_VEL_DPS = 8.0
AUTO_VALID_MIN_ABS_POWER_W = 2.0
AUTO_HIST_MARGIN_S = 2.0
AUTO_MAX_DELAY_S = MAX_RUNTIME_DELAY_MS / 1000.0
AUTO_HIST_FRAMES = int(round((AUTO_WINDOW_MAX_S + AUTO_MAX_DELAY_S + AUTO_HIST_MARGIN_S) * CTRL_HZ))
# Auto-delay power/motion evaluation velocity source:
# True  -> use finite-difference d(angle)/dt (with wrap protection), hardcoded (no GUI)
# False -> use IMU raw angular velocity LTAVx/RTAVx
AUTO_PWR_USE_ANGLE_DIFF_VEL = True
AUTO_PWR_ANGLE_WRAP_DEG = 180.0

# Runtime optimizer method: 'grid' (legacy local scan) or 'bo' (Bayesian optimization)
AUTO_OPT_METHOD_DEFAULT = 'grid'

# Objective: J = pos_per_s - λ * max(0, target_ratio - ratio)^2 - μ * |neg_per_s|
AUTO_OBJ_LAMBDA = 20.0
AUTO_OBJ_MU = 1.0

# BO (1D GP over delay_ms)
AUTO_BO_MIN_SAMPLES = 3
AUTO_BO_HISTORY_MAX = 80
AUTO_BO_LENGTH_SCALE_MS = 35.0
AUTO_BO_SIGMA_F = 1.0
AUTO_BO_SIGMA_N = 0.25
AUTO_BO_JITTER = 1e-6
AUTO_BO_ACQ = 'ucb'            # 'ucb' | 'ei'
AUTO_BO_UCB_BETA = 2.0
AUTO_BO_EI_XI = 0.01


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
            # Torque filter is unified in main loop for all NN types.
            torque_filter_type=None,
            torque_filter_b=None,
            torque_filter_a=None,
            torque_filter_enable=False,
            enable_zero_mean=ENABLE_ZERO_MEAN,
            zero_mean_buffer_size=ZERO_MEAN_BUFFER_SIZE,
            zero_mean_warmup=ZERO_MEAN_WARMUP,
        )

    elif NN_TYPE == 'lstm':
        from networks.lstm_network import LSTMNetwork
        print(f"\n{'='*80}")
        print(f"加载 LSTMNetwork: {LSTM_MODEL_PATH}")
        print(f"kp={kp}, kd={kd}")
        print(f"内部exo_filter: bypass(identity), 统一torque滤波在主循环配置")
        print(f"{'='*80}")

        nn_obj = LSTMNetwork(
            kp=kp, kd=kd,
            b=NN_INTERNAL_FILTER_BYPASS_B, a=NN_INTERNAL_FILTER_BYPASS_A,
        )
        nn_obj.load_saved_policy(torch.load(LSTM_MODEL_PATH, map_location=torch.device('cpu')))
        nn_obj.eval()
        print('[统一扭矩滤波] LSTM internal exo_filter bypassed (handled in main loop).')
        print('load parameters successfully!!')
        return nn_obj

    elif NN_TYPE == 'lstm_leg_dcp':
        from networks.lstm_leg_dcp import LSTMNetworkLegDcp
        print(f"\n{'='*80}")
        print(f"加载 LSTMNetworkLegDcp: {LSTM_MODEL_PATH}")
        print(f"kp={kp}, kd={kd}")
        print(f"内部exo_filter: bypass(identity), 统一torque滤波在主循环配置")
        print(f"{'='*80}")

        nn_obj = LSTMNetworkLegDcp(
            kp=kp, kd=kd,
            b=NN_INTERNAL_FILTER_BYPASS_B, a=NN_INTERNAL_FILTER_BYPASS_A,
        )
        nn_obj.load_saved_policy(torch.load(LSTM_MODEL_PATH, map_location=torch.device('cpu')))
        nn_obj.eval()
        print('[统一扭矩滤波] LSTM-LegDcp internal exo_filter bypassed (handled in main loop).')
        print('load parameters successfully!!')
        return nn_obj

    elif NN_TYPE == 'lstm_pd':
        from networks.lstm_pd import LSTMNetworkPD
        print(f"\n{'='*80}")
        print(f"加载 LSTMNetworkPD: {LSTM_PD_MODEL_PATH}")
        print(f"kp={kp}, kd={kd}")
        print(f"内部exo_filter: bypass(identity), 统一torque滤波在主循环配置")
        print(f"{'='*80}")

        nn_obj = LSTMNetworkPD(
            kp=kp, kd=kd,
            b=NN_INTERNAL_FILTER_BYPASS_B, a=NN_INTERNAL_FILTER_BYPASS_A,
        )
        nn_obj.load_saved_policy(torch.load(LSTM_PD_MODEL_PATH, map_location=torch.device('cpu')))
        nn_obj.eval()
        print('[统一扭矩滤波] LSTM-PD internal exo_filter bypassed (handled in main loop).')
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
os.makedirs('./output', exist_ok=True)
root = './output/PI5_' + motion_type + '-'
logf = root + ts.strftime('%Y%m%d-%H%M%S') + '.csv'

csv_header = [
    'Time_ms', 'imu_LTx', 'imu_RTx', 'imu_Lvel', 'imu_Rvel',
    'teensy_t_cs_u16', 'teensy_t_s_unwrapped',
    'auto_eval_Lvel', 'auto_eval_Rvel', 'auto_vel_source',
    'sync_sample_id', 'sync_LTx', 'sync_RTx', 'sync_Lvel', 'sync_Rvel',
    'sync_ctrl_pwr_L', 'sync_ctrl_pwr_R',
    'L_command_actuator', 'R_command_actuator',
    'raw_LExoTorque', 'raw_RExoTorque',
    'filtered_LExoTorque', 'filtered_RExoTorque',
    'pd_zm_bias_L', 'pd_zm_bias_R',
    'L_P', 'L_D', 'R_P', 'R_D',
    'scale_runtime', 'torque_delay_ms_L', 'torque_delay_ms_R',
    'auto_opt_method', 'auto_delay_enable',
    'auto_motion_valid_L', 'auto_motion_valid_R',
    'auto_ratio_L', 'auto_ratio_R',
    'auto_pos_per_s_L', 'auto_pos_per_s_R',
    'auto_neg_per_s_L', 'auto_neg_per_s_R',
    'auto_best_delay_ms_L', 'auto_best_delay_ms_R',
    'auto_window_s', 'auto_gait_freq_hz',
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

# auto_flags bit layout (GUI passthrough [20] and status uplink [20]):
# bit0: auto_delay_enable
# bit1: motion_valid_L (status only)
# bit2: motion_valid_R (status only)
# bit3: auto_method_bo (0=grid, 1=bo)
RPI_AUTO_FLAG_ENABLE         = 0x01
RPI_AUTO_FLAG_MOTION_VALID_L = 0x02
RPI_AUTO_FLAG_MOTION_VALID_R = 0x04
RPI_AUTO_FLAG_METHOD_BO      = 0x08

# ---- NN_TYPE 编码 (用于上行状态) ----
NN_TYPE_CODE = {'dnn': 0, 'lstm': 1, 'lstm_leg_dcp': 2, 'lstm_pd': 3}

# ---- 状态发送间隔 ----
STATUS_SEND_INTERVAL = 50  # 每50帧发送一次 (0.5s @100Hz)


class IdentityFilter:
    def filter(self, x):
        return x


def build_unified_torque_filters(filter_code, cutoff_hz, filter_order, enable_torque):
    """
    Build the unified torque filters used by all NN types in main loop.
    Returns: (left_filter, right_filter, used_filter_code, used_cutoff_hz, used_order, used_enable)
    """
    if not enable_torque:
        return IdentityFilter(), IdentityFilter(), int(filter_code), float(cutoff_hz), int(filter_order), False

    code = int(filter_code)
    order = max(1, min(6, int(filter_order)))
    cutoff = float(cutoff_hz)
    filter_type = RPI_FILTER_TYPE_MAP.get(code)
    if filter_type is None:
        code = UNIFIED_TORQUE_FILTER_CODE_DEFAULT
        filter_type = RPI_FILTER_TYPE_MAP[code]

    # Keep cutoff in valid IIR range for the current CTRL_HZ
    cutoff = max(0.5, min(float(CTRL_HZ) * 0.45, cutoff))
    try:
        b, a = compute_iir_coeffs(cutoff, filter_type, float(CTRL_HZ), order)
        return IIRFilter(b=b, a=a), IIRFilter(b=b, a=a), code, cutoff, order, True
    except Exception as exc:
        print(f"[统一扭矩滤波] rebuild failed ({exc}), fallback to default 5Hz Butterworth 2nd")
        code = UNIFIED_TORQUE_FILTER_CODE_DEFAULT
        order = UNIFIED_TORQUE_FILTER_ORDER_DEFAULT
        cutoff = UNIFIED_TORQUE_FILTER_CUTOFF_DEFAULT
        b, a = compute_iir_coeffs(cutoff, RPI_FILTER_TYPE_MAP[code], float(CTRL_HZ), order)
        return IIRFilter(b=b, a=a), IIRFilter(b=b, a=a), code, cutoff, order, True


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


def _finite_diff_vel_deg_s(cur_ang_deg: float, prev_ang_deg: float, fs_hz: float,
                           wrap_deg: float = 180.0) -> float:
    """
    Angular finite difference with wrap protection.
    Example (wrap_deg=180): if angle jumps from +179 to -179 deg, delta is +2 deg.
    """
    d = float(cur_ang_deg) - float(prev_ang_deg)
    period = 2.0 * float(wrap_deg)
    if period > 0.0:
        while d > float(wrap_deg):
            d -= period
        while d < -float(wrap_deg):
            d += period
    return d * float(fs_hz)


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


def evaluate_delay_candidate_leg(delay_ms: float, tau_src: np.ndarray,
                                 vel: np.ndarray, window_frames: int):
    """
    Evaluate one leg under a candidate delay. scale is fixed to 1.0 — ratio is
    scale-invariant, and pos_per_s ordering between candidates at the same scale
    is also scale-invariant, so per-leg selection is unaffected. Display scaling
    is applied by the caller (runtime_scale * pos_per_s).
    """
    d_frames = int(round(float(delay_ms) / dt_ms))
    m = compute_leg_power_metrics(tau_src, vel, d_frames, window_frames, 1.0)
    if m is None:
        return None
    return {
        'delay_ms': float(delay_ms),
        'ratio': float(m['ratio']),
        'pos_per_s': float(m['pos_per_s']),
        'neg_per_s': float(m['neg_per_s']),
        'abs_power_per_s': float(m['abs_power_per_s']),
        'mean_abs_vel': float(m['mean_abs_vel']),
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


def auto_objective_from_eval(ev: dict) -> float:
    """Scalar objective used by auto-delay optimizer (scale-invariant, before runtime_scale display)."""
    if ev is None:
        return -1e9
    ratio = float(ev.get('ratio', 0.0))
    pos_per_s = float(ev.get('pos_per_s', 0.0))
    neg_per_s = float(ev.get('neg_per_s', 0.0))
    penalty_ratio = max(0.0, AUTO_TARGET_RATIO - ratio)
    return pos_per_s - AUTO_OBJ_LAMBDA * (penalty_ratio ** 2) - AUTO_OBJ_MU * abs(neg_per_s)


def _norm_pdf(z: np.ndarray) -> np.ndarray:
    return np.exp(-0.5 * z * z) / math.sqrt(2.0 * math.pi)


def _norm_cdf(z: np.ndarray) -> np.ndarray:
    return 0.5 * (1.0 + np.vectorize(math.erf)(z / math.sqrt(2.0)))


def _rbf_kernel_1d(x1: np.ndarray, x2: np.ndarray, length_scale: float, sigma_f: float) -> np.ndarray:
    d = (x1[:, None] - x2[None, :]) / max(1e-6, float(length_scale))
    return (float(sigma_f) ** 2) * np.exp(-0.5 * d * d)


def gp_predict_1d(x_obs: np.ndarray, y_obs: np.ndarray, x_query: np.ndarray):
    """
    Lightweight 1D GP prediction with RBF kernel.
    Returns (mu, std) at query points, or (None, None) on numerical failure.
    """
    if x_obs.size < 1 or y_obs.size < 1 or x_query.size < 1:
        return None, None

    y_mean = float(np.mean(y_obs))
    y_std = float(np.std(y_obs))
    y_scale = y_std if y_std > 1e-6 else 1.0
    y_n = (y_obs - y_mean) / y_scale

    k_xx = _rbf_kernel_1d(
        x_obs, x_obs,
        length_scale=AUTO_BO_LENGTH_SCALE_MS,
        sigma_f=AUTO_BO_SIGMA_F
    )
    k_xx += (AUTO_BO_SIGMA_N ** 2 + AUTO_BO_JITTER) * np.eye(x_obs.size, dtype=np.float64)

    try:
        l = np.linalg.cholesky(k_xx)
        alpha = np.linalg.solve(l.T, np.linalg.solve(l, y_n))
        k_xs = _rbf_kernel_1d(
            x_obs, x_query,
            length_scale=AUTO_BO_LENGTH_SCALE_MS,
            sigma_f=AUTO_BO_SIGMA_F
        )
        mu_n = k_xs.T @ alpha
        v = np.linalg.solve(l, k_xs)
        var_n = (AUTO_BO_SIGMA_F ** 2) - np.sum(v * v, axis=0)
        var_n = np.maximum(var_n, 1e-12)
    except np.linalg.LinAlgError:
        return None, None

    mu = y_mean + y_scale * mu_n
    std = y_scale * np.sqrt(var_n)
    return mu.astype(np.float64), std.astype(np.float64)


def propose_bo_delay_candidate(history_x, history_y, current_delay_ms: float,
                               d_lo: float, d_hi: float, step_ms: float) -> float:
    """
    Propose next delay (ms) by 1D BO over local bounds [d_lo, d_hi].
    Falls back to deterministic seeding when history is short.
    """
    x_query = np.arange(d_lo, d_hi + 0.5 * step_ms, step_ms, dtype=np.float64)
    if x_query.size == 0:
        return float(current_delay_ms)

    x_obs = np.asarray(history_x, dtype=np.float64)
    y_obs = np.asarray(history_y, dtype=np.float64)

    # Cold start: deterministic local seeding around current point.
    if x_obs.size < AUTO_BO_MIN_SAMPLES:
        span = max(step_ms, 0.5 * (d_hi - d_lo))
        seed_offsets = np.array([0.0, +0.5 * span, -0.5 * span, +span, -span], dtype=np.float64)
        sampled = set(np.round(x_obs / max(step_ms, 1e-6)).astype(np.int64).tolist())
        for off in seed_offsets:
            cand = float(np.clip(current_delay_ms + off, d_lo, d_hi))
            cand_bin = int(round(cand / max(step_ms, 1e-6)))
            if cand_bin not in sampled:
                return cand
        return float(np.clip(current_delay_ms, d_lo, d_hi))

    mu, std = gp_predict_1d(x_obs, y_obs, x_query)
    if mu is None or std is None:
        return float(np.clip(current_delay_ms, d_lo, d_hi))

    if AUTO_BO_ACQ == 'ei':
        y_best = float(np.max(y_obs))
        imp = mu - y_best - AUTO_BO_EI_XI
        z = np.zeros_like(imp)
        nz = std > 1e-12
        z[nz] = imp[nz] / std[nz]
        ei = np.zeros_like(imp)
        if np.any(nz):
            ei[nz] = imp[nz] * _norm_cdf(z[nz]) + std[nz] * _norm_pdf(z[nz])
        acq = ei
    else:
        acq = mu + AUTO_BO_UCB_BETA * std

    best_idx = int(np.argmax(acq))
    return float(x_query[best_idx])


def make_bo_state():
    return {
        'x': deque(maxlen=AUTO_BO_HISTORY_MAX),
        'y': deque(maxlen=AUTO_BO_HISTORY_MAX),
    }


def reset_bo_state(state: dict):
    state['x'].clear()
    state['y'].clear()


def bo_record_sample(state: dict, delay_ms: float, objective: float):
    if not (math.isfinite(delay_ms) and math.isfinite(objective)):
        return
    state['x'].append(float(delay_ms))
    state['y'].append(float(objective))


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
        "auto_delay_enable": bool(auto_flags & RPI_AUTO_FLAG_ENABLE),
        "auto_method": "bo" if (auto_flags & RPI_AUTO_FLAG_METHOD_BO) else "grid",
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
        # Torque filter is unified in main loop; force-disable DNN internal torque filter.
        dnn_obj.torque_filter_enable = False
        dnn_obj.left_torque_filter = None
        dnn_obj.right_torque_filter = None
    except Exception as exc:
        print(f"[RPi CFG] Failed: {exc}")
        return False

    print(
        f"[RPi CFG] DNN filter applied: {filter_type} {cutoff_hz:.2f}Hz order={filter_order} "
        f"(torque handled by unified main-loop filter)"
    )
    return True


def apply_runtime_filter_to_lstm(nn_obj, filter_code, cutoff_hz, filter_order, enable_torque):
    """LSTM internal torque pre-filter is bypassed; unified filter is in main loop."""
    if NN_TYPE == 'dnn':
        return False

    nn_obj.left_exo_filter = IdentityFilter()
    nn_obj.right_exo_filter = IdentityFilter()
    nn_obj.b = NN_INTERNAL_FILTER_BYPASS_B
    nn_obj.a = NN_INTERNAL_FILTER_BYPASS_A
    print("[RPi CFG] LSTM internal exo_filter bypassed (torque handled by unified main-loop filter)")
    return True


def read_packet(ser: serial.Serial):
    """
    Return dict or None:
      {'type':'imu', 'Lpos':..., 'Rpos':..., 'Lvel':..., 'Rvel':..., 'logtag':..., 't_cs':...}
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
                't_cs': None,
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
        # v2: payload_len=29 => t_cs(u16)+4 floats+tag11
        # v1 (legacy): payload_len=27 => 4 floats+tag11
        if payload_len == 29:
            t_cs, Lpos, Rpos, Lvel, Rvel, logtag = struct.unpack('<Hffff11s', payload)
            return {
                'type': 'imu',
                't_cs': int(t_cs) & 0xFFFF,
                'Lpos': Lpos, 'Rpos': Rpos,
                'Lvel': Lvel, 'Rvel': Rvel,
                'logtag': logtag.decode(errors='ignore').rstrip('\x00'),
            }
        if payload_len == 27:
            Lpos, Rpos, Lvel, Rvel, logtag = struct.unpack('<ffff11s', payload)
            return {
                'type': 'imu',
                't_cs': None,
                'Lpos': Lpos, 'Rpos': Rpos,
                'Lvel': Lvel, 'Rvel': Rvel,
                'logtag': logtag.decode(errors='ignore').rstrip('\x00'),
            }
        return None

    if typ_v == 0x02:
        return {'type': 'cfg', 'payload': payload}

    return None


# Full IMU v2 frame on the wire: header(2) + len(1) + type(1) + payload(29) + cksum(1) = 34B.
# Used by read_freshest_packet() to cheaply detect "another IMU frame already queued".
IMU_PACKET_BYTES = 34


def read_freshest_packet(ser: serial.Serial):
    """
    Drop-in replacement for read_packet() used by the main 100Hz loop.

    Teensy emits IMU frames at ~1kHz while we consume at 100Hz, and the OS
    serial FIFO is strict-FIFO: once we fall behind, every subsequent
    read_packet() hands us an older frame, so real-time IMU is permanently
    delayed. Here we peek at ser.in_waiting and, if at least one more full
    IMU frame is already buffered, we keep reading and discard older IMU
    frames until we hit the newest one. cfg frames are surfaced in order
    so GUI runtime config is never dropped.
    """
    pkt = read_packet(ser)
    if pkt is None:
        return None
    if not PI_USE_BINARY:
        return pkt
    while pkt is not None and pkt.get('type') == 'imu' and ser.in_waiting >= IMU_PACKET_BYTES:
        nxt = read_packet(ser)
        if nxt is None:
            return pkt
        if nxt.get('type') != 'imu':
            return nxt
        pkt = nxt
    return pkt


def _sat_i16(v: float, scale_factor: float = 1.0) -> int:
    x = int(round(float(v) * float(scale_factor)))
    if x < -32768:
        return -32768
    if x > 32767:
        return 32767
    return x


def send_torque(ser: serial.Serial, tau_L: float, tau_R: float, L_p, L_d, R_p, R_d,
                sample_id: int,
                sync_ang_L: float, sync_ang_R: float,
                sync_vel_L: float, sync_vel_R: float,
                ctrl_pwr_L: float, ctrl_pwr_R: float):
    """
    向 Teensy 发送同步控制帧 (header=AA 59), payload 40 bytes:
      sample_id(u16),
      tauL,tauR,Lp,Ld,Rp,Rd (6xfloat32),
      sync_ang_L/R (i16, deg*100),
      sync_vel_L/R (i16, deg/s*10),
      ctrl_pwr_L/R (i16, W*100),
      flags(u8), reserved(u8)
    """
    flags = 0x01  # sync-valid
    payload = struct.pack(
        '<HffffffhhhhhhBB',
        int(sample_id) & 0xFFFF,
        float(tau_L), float(tau_R),
        float(L_p), float(L_d), float(R_p), float(R_d),
        _sat_i16(sync_ang_L, 100.0), _sat_i16(sync_ang_R, 100.0),
        _sat_i16(sync_vel_L, 10.0), _sat_i16(sync_vel_R, 10.0),
        _sat_i16(ctrl_pwr_L, 100.0), _sat_i16(ctrl_pwr_R, 100.0),
        int(flags) & 0xFF,
        0,
    )
    packet = b'\xAA\x59' + payload
    ser.write(packet)


def _pack_i16(buf: bytearray, off: int, value: float, scale_factor: float):
    """Clamp + pack a float into int16 little-endian at buf[off..off+1]."""
    v = int(round(float(value) * float(scale_factor)))
    if v < -32768:
        v = -32768
    elif v > 32767:
        v = 32767
    struct.pack_into('<h', buf, off, v)


def send_status(ser: serial.Serial, filter_source, filter_type_code,
                filter_order, enable_mask, cutoff_hz, scale,
                delay_ms_L, delay_ms_R,
                auto_enable=False,
                auto_method='grid',
                motion_valid_L=False, motion_valid_R=False,
                ratio_L=0.0, ratio_R=0.0,
                pos_per_s_L=0.0, pos_per_s_R=0.0,
                neg_per_s_L=0.0, neg_per_s_R=0.0,
                best_delay_L=0.0, best_delay_R=0.0):
    """
    向 Teensy 发送 RPi 状态 (header=AA 56), Teensy 转存到 rpi_uplink_buf → BLE → GUI
    40 bytes payload (v3, int16-packed L/R pairs):
      [0-1] magic 'RL'
      [2]   version = 3
      [3]   nn_type (0=dnn,1=lstm,2=lstm_leg_dcp,3=lstm_pd)
      [4]   filter_source (0=base, 1=runtime_override)
      [5]   filter_type_code (1=butter, 2=bessel, 3=cheby2)
      [6]   filter_order
      [7]   enable_mask (bit0=vel, bit1=ref, bit2=torque)
      [8..11]  cutoff_hz  float32
      [12..15] scale      float32
      [16..17] delay_ms_L    int16 ×10   (0.1ms)
      [18..19] delay_ms_R    int16 ×10
      [20]     auto_flags bit0=auto_enable, bit1=motion_valid_L,
                               bit2=motion_valid_R, bit3=method_bo
      [21..23] reserved
      [24..25] ratio_L       int16 ×10000 (0.01%)
      [26..27] ratio_R       int16 ×10000
      [28..29] pos_per_s_L   int16 ×100   (0.01 W)
      [30..31] pos_per_s_R   int16 ×100
      [32..33] neg_per_s_L   int16 ×100
      [34..35] neg_per_s_R   int16 ×100
      [36..37] best_delay_L  int16 ×10    (0.1ms)
      [38..39] best_delay_R  int16 ×10
    """
    buf = bytearray(40)
    buf[0] = 0x52  # 'R'
    buf[1] = 0x4C  # 'L'
    buf[2] = 0x03  # version 3 (per-leg L/R packed int16)
    buf[3] = NN_TYPE_CODE.get(NN_TYPE, 0) & 0xFF
    buf[4] = int(filter_source) & 0xFF
    buf[5] = int(filter_type_code) & 0xFF
    buf[6] = int(filter_order) & 0xFF
    buf[7] = int(enable_mask) & 0xFF
    struct.pack_into('<f', buf, 8, float(cutoff_hz))
    struct.pack_into('<f', buf, 12, float(scale))

    _pack_i16(buf, 16, delay_ms_L, 10.0)
    _pack_i16(buf, 18, delay_ms_R, 10.0)

    auto_flags = 0
    if auto_enable:
        auto_flags |= RPI_AUTO_FLAG_ENABLE
    if str(auto_method).lower() == 'bo':
        auto_flags |= RPI_AUTO_FLAG_METHOD_BO
    if motion_valid_L:
        auto_flags |= RPI_AUTO_FLAG_MOTION_VALID_L
    if motion_valid_R:
        auto_flags |= RPI_AUTO_FLAG_MOTION_VALID_R
    buf[20] = auto_flags & 0xFF
    # buf[21..23] = 0 (reserved)

    _pack_i16(buf, 24, ratio_L, 10000.0)
    _pack_i16(buf, 26, ratio_R, 10000.0)
    _pack_i16(buf, 28, pos_per_s_L, 100.0)
    _pack_i16(buf, 30, pos_per_s_R, 100.0)
    _pack_i16(buf, 32, neg_per_s_L, 100.0)
    _pack_i16(buf, 34, neg_per_s_R, 100.0)
    _pack_i16(buf, 36, best_delay_L, 10.0)
    _pack_i16(buf, 38, best_delay_R, 10.0)

    packet = b'\xAA\x56' + bytes(buf)
    ser.write(packet)


# ========= 主程序 =========
def main():
    ser = serial.Serial(SER_DEV, BAUDRATE, timeout=TIMEOUT)
    ser.reset_input_buffer()
    start = time.time()
    last_flush = time.time()

    # runtime_scale: GUI 运行时下发的最终扭矩倍率 (乘在 delay 后命令上)
    # LSTM-PD 默认使用更保守的 scale=0.4，其它网络维持 1.0
    runtime_scale = 0.4 if NN_TYPE == 'lstm_pd' else 1.0
    pd_zm_enabled = (NN_TYPE == 'lstm_pd') and bool(LSTM_PD_ZERO_MEAN_ENABLE)
    pd_zm_window_frames = max(8, int(round(LSTM_PD_ZERO_MEAN_WINDOW_S * CTRL_HZ)))
    pd_zm_warmup_frames = max(1, int(round(LSTM_PD_ZERO_MEAN_WARMUP_S * CTRL_HZ)))
    pd_zm_hist_L = deque(maxlen=pd_zm_window_frames)
    pd_zm_hist_R = deque(maxlen=pd_zm_window_frames)
    pd_zm_bias_L = 0.0
    pd_zm_bias_R = 0.0
    # Per-NN default delay baseline (lstm_pd defaults to 200ms).
    runtime_delay_ms = float(DEFAULT_RUNTIME_DELAY_MS.get(NN_TYPE, 0.0))
    # 每腿当前实际生效的 delay (auto 模式下独立漂移; 非 auto 模式与 runtime_delay_ms 同步)
    runtime_delay_ms_L = runtime_delay_ms
    runtime_delay_ms_R = runtime_delay_ms

    auto_delay_enable = False
    auto_opt_method = AUTO_OPT_METHOD_DEFAULT
    auto_last_eval_ts = 0.0
    auto_last_change_ts_L = 0.0
    auto_last_change_ts_R = 0.0

    auto_cur_ratio_L = 0.0
    auto_cur_ratio_R = 0.0
    auto_cur_pos_per_s_L = 0.0
    auto_cur_pos_per_s_R = 0.0
    auto_cur_neg_per_s_L = 0.0
    auto_cur_neg_per_s_R = 0.0
    auto_best_delay_ms_L = runtime_delay_ms
    auto_best_delay_ms_R = runtime_delay_ms
    auto_motion_valid_L = False
    auto_motion_valid_R = False

    auto_gait_freq_hz = 0.0
    auto_window_s = AUTO_WINDOW_FALLBACK_S

    hist_tau_src_L = deque(maxlen=AUTO_HIST_FRAMES)
    hist_tau_src_R = deque(maxlen=AUTO_HIST_FRAMES)
    hist_vel_L = deque(maxlen=AUTO_HIST_FRAMES)
    hist_vel_R = deque(maxlen=AUTO_HIST_FRAMES)
    hist_ang_L = deque(maxlen=AUTO_HIST_FRAMES)
    hist_ang_R = deque(maxlen=AUTO_HIST_FRAMES)
    auto_prev_ang_L = None
    auto_prev_ang_R = None
    auto_bo_state_L = make_bo_state()
    auto_bo_state_R = make_bo_state()

    # Per-frame sample id + delayed sync-input buffers (aligned with delay_frames_L/R)
    sample_counter = 0
    teensy_prev_t_cs = None
    teensy_wrap_count = 0
    teensy_t_s_unwrapped = 0.0
    sync_sid_buf = deque([0] * BUF_SIZE, maxlen=BUF_SIZE)
    sync_ang_L_buf = deque([0.0] * BUF_SIZE, maxlen=BUF_SIZE)
    sync_ang_R_buf = deque([0.0] * BUF_SIZE, maxlen=BUF_SIZE)
    sync_vel_L_buf = deque([0.0] * BUF_SIZE, maxlen=BUF_SIZE)
    sync_vel_R_buf = deque([0.0] * BUF_SIZE, maxlen=BUF_SIZE)

    last_gui_tag = ""
    frame_count = 0
    status_dirty = True  # 启动时立即发送一次状态

    # 当前滤波器状态跟踪 (用于上行状态)
    # 统一策略: torque filter 默认 5Hz 2nd Butterworth, 由主循环统一应用
    cur_filter_source = 0  # 0=base, 1=runtime_override
    cur_filter_type_code = UNIFIED_TORQUE_FILTER_CODE_DEFAULT
    cur_filter_order = UNIFIED_TORQUE_FILTER_ORDER_DEFAULT
    cur_cutoff_hz = UNIFIED_TORQUE_FILTER_CUTOFF_DEFAULT
    cur_enable_mask = (RPI_FILTER_EN_TORQUE if UNIFIED_TORQUE_FILTER_ENABLE_DEFAULT else 0x00)
    # DNN 仍保留 vel/ref 运行时滤波开关；LSTM 家族仅 torque 有意义
    if NN_TYPE == 'dnn':
        cur_enable_mask |= (RPI_FILTER_EN_VEL | RPI_FILTER_EN_REF)

    torque_filter_L, torque_filter_R, cur_filter_type_code, cur_cutoff_hz, cur_filter_order, torque_on = \
        build_unified_torque_filters(
            cur_filter_type_code, cur_cutoff_hz, cur_filter_order,
            bool(cur_enable_mask & RPI_FILTER_EN_TORQUE)
        )
    if torque_on:
        cur_enable_mask |= RPI_FILTER_EN_TORQUE
    else:
        cur_enable_mask &= ~RPI_FILTER_EN_TORQUE

    print(f"\n[启动] NN_TYPE={NN_TYPE}, kp={kp}, kd={kd}")
    print(f"[启动] 串口={SER_DEV}, 波特率={BAUDRATE}")
    print(f"[启动] AutoDelay optimizer={auto_opt_method} (grid|bo)")
    print(
        f"[启动] AutoDelay power velocity source="
        f"{'d(angle)/dt' if AUTO_PWR_USE_ANGLE_DIFF_VEL else 'imu_raw(LTAVx/RTAVx)'}"
    )
    _startup_filter_name = RPI_FILTER_TYPE_MAP.get(cur_filter_type_code, '?')
    print(
        f"[启动] Unified torque filter="
        f"{'ON' if (cur_enable_mask & RPI_FILTER_EN_TORQUE) else 'OFF'} "
        f"{_startup_filter_name} {cur_cutoff_hz:.1f}Hz order={cur_filter_order}"
    )
    if NN_TYPE == 'lstm_pd':
        print(
            f"[启动] lstm_pd zero-mean={'ON' if pd_zm_enabled else 'OFF'} "
            f"(window={pd_zm_window_frames}f, warmup={pd_zm_warmup_frames}f)"
        )
    print(f"[启动] 日志={logf}\n")

    with open(logf, 'w', newline='') as fcsv:
        wr = csv.DictWriter(fcsv, fieldnames=csv_header)
        wr.writeheader()

        while True:
            pkt = read_freshest_packet(ser)
            if pkt is None:
                continue

            # ---- GUI运行时配置 (DNN模式支持滤波器热更新) ----
            if pkt['type'] == 'cfg':
                cfg = parse_runtime_cfg(pkt['payload'])
                if cfg is None:
                    continue

                runtime_scale = cfg['scale']
                runtime_delay_ms = cfg['delay_ms']
                # GUI 下发单个 delay → 同时赋给 L/R，不管 auto 是否打开。
                # auto ON 时 L/R 之后会从这个基线独立漂移；auto OFF 时 L/R 永远跟这个值。
                runtime_delay_ms_L = runtime_delay_ms
                runtime_delay_ms_R = runtime_delay_ms

                prev_auto_delay_enable = auto_delay_enable
                auto_delay_enable = bool(cfg.get('auto_delay_enable', auto_delay_enable))
                auto_opt_method = str(cfg.get('auto_method', auto_opt_method)).lower()
                if auto_opt_method not in ('grid', 'bo'):
                    auto_opt_method = 'grid'
                # When auto turns off, clear motion_valid flags but keep last known
                # ratio/power metrics so the GUI continues to show the most-recent values.
                if not auto_delay_enable:
                    auto_motion_valid_L = False
                    auto_motion_valid_R = False
                    auto_best_delay_ms_L = runtime_delay_ms
                    auto_best_delay_ms_R = runtime_delay_ms
                    reset_bo_state(auto_bo_state_L)
                    reset_bo_state(auto_bo_state_R)

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
                auto_prev_ang_L = None
                auto_prev_ang_R = None
                reset_bo_state(auto_bo_state_L)
                reset_bo_state(auto_bo_state_R)
                if pd_zm_enabled:
                    pd_zm_hist_L.clear()
                    pd_zm_hist_R.clear()
                    pd_zm_bias_L = 0.0
                    pd_zm_bias_R = 0.0
                _now_wall = time.time()
                auto_last_eval_ts = _now_wall
                if auto_delay_enable and not prev_auto_delay_enable:
                    # Grid bootstrap for LSTM-PD: if GUI baseline is still ~0ms at the
                    # moment auto turns on, seed to 200ms so scan does not start at 0ms.
                    if NN_TYPE == 'lstm_pd' and auto_opt_method == 'grid' and runtime_delay_ms <= 0.5:
                        runtime_delay_ms = 200.0
                        runtime_delay_ms_L = runtime_delay_ms
                        runtime_delay_ms_R = runtime_delay_ms
                        auto_best_delay_ms_L = runtime_delay_ms
                        auto_best_delay_ms_R = runtime_delay_ms
                    auto_last_change_ts_L = _now_wall
                    auto_last_change_ts_R = _now_wall
                    reset_bo_state(auto_bo_state_L)
                    reset_bo_state(auto_bo_state_R)

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

                # Unified torque filter (all NN): filter -> delay -> scale -> send
                torque_filter_L, torque_filter_R, used_code, used_cutoff, used_order, used_enable = \
                    build_unified_torque_filters(
                        cfg['filter_code'], cfg['cutoff_hz'], cfg['filter_order'], cfg['enable_torque']
                    )

                # 更新滤波器状态跟踪
                cur_filter_source = 1  # runtime_override
                cur_filter_type_code = used_code
                cur_filter_order = used_order
                cur_cutoff_hz = used_cutoff
                en = 0
                if cfg['enable_vel']:  en |= RPI_FILTER_EN_VEL
                if cfg['enable_ref']:  en |= RPI_FILTER_EN_REF
                if used_enable: en |= RPI_FILTER_EN_TORQUE
                cur_enable_mask = en
                status_dirty = True  # 立即回传状态

                delay_buf_L.clear()
                delay_buf_L.extend([0.0] * BUF_SIZE)
                delay_buf_R.clear()
                delay_buf_R.extend([0.0] * BUF_SIZE)
                sync_sid_buf.clear()
                sync_sid_buf.extend([0] * BUF_SIZE)
                sync_ang_L_buf.clear()
                sync_ang_L_buf.extend([0.0] * BUF_SIZE)
                sync_ang_R_buf.clear()
                sync_ang_R_buf.extend([0.0] * BUF_SIZE)
                sync_vel_L_buf.clear()
                sync_vel_L_buf.extend([0.0] * BUF_SIZE)
                sync_vel_R_buf.clear()
                sync_vel_R_buf.extend([0.0] * BUF_SIZE)

                print(
                    f"[RPi CFG] scale={runtime_scale:.3f}, delay={runtime_delay_ms:.1f}ms "
                    f"(L/R snapped), auto_delay={'ON' if auto_delay_enable else 'OFF'}, "
                    f"method={auto_opt_method}"
                )
                continue

            if pkt['type'] != 'imu':
                continue

            sample_counter += 1
            pkt_t_cs = pkt.get('t_cs', None)
            if pkt_t_cs is not None:
                t_cs_u16 = int(pkt_t_cs) & 0xFFFF
                if teensy_prev_t_cs is not None:
                    dt_cs = t_cs_u16 - int(teensy_prev_t_cs)
                    if dt_cs < -30000:
                        teensy_wrap_count += 1
                    elif dt_cs > 30000:
                        teensy_wrap_count = 0
                teensy_prev_t_cs = t_cs_u16
                teensy_t_s_unwrapped = ((teensy_wrap_count * 65536) + t_cs_u16) / 100.0
                cur_sample_id = t_cs_u16
            else:
                t_cs_u16 = -1
                cur_sample_id = int(sample_counter) & 0xFFFF

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

            now = time.time() - start

            # ========= NN 推理 =========
            action = dnn.generate_assistance(Lpos, Rpos, Lvel, Rvel)
            L_cmd = dnn.hip_torque_L
            R_cmd = dnn.hip_torque_R
            # L_p/L_d 由各网络直接给出并原样透传到日志与 Teensy。
            # 例如 lstm_pd 中 D 项已带符号(L_d = -dqTd*kd)，总扭矩是 L_p + L_d。
            L_p, L_d, R_p, R_d = dnn.L_p, dnn.L_d, dnn.R_p, dnn.R_d

            # ---- 统一扭矩滤波 (all NN, pre-delay path) ----
            filter_L_cmd = torque_filter_L.filter(float(L_cmd))
            filter_R_cmd = torque_filter_R.filter(float(R_cmd))

            # ---- lstm_pd: optional post-filter zero-mean balancing (RPi only) ----
            if pd_zm_enabled:
                pd_zm_hist_L.append(float(filter_L_cmd))
                pd_zm_hist_R.append(float(filter_R_cmd))
                if len(pd_zm_hist_L) >= pd_zm_warmup_frames:
                    pd_zm_bias_L = float(np.mean(pd_zm_hist_L))
                    pd_zm_bias_R = float(np.mean(pd_zm_hist_R))
                else:
                    pd_zm_bias_L = 0.0
                    pd_zm_bias_R = 0.0
                filter_L_cmd = float(filter_L_cmd) - pd_zm_bias_L
                filter_R_cmd = float(filter_R_cmd) - pd_zm_bias_R
            else:
                pd_zm_bias_L = 0.0
                pd_zm_bias_R = 0.0

            # 统一管线固定为: filter -> delay -> scale -> send
            delay_input_L = float(filter_L_cmd)
            delay_input_R = float(filter_R_cmd)

            # ---- velocity source for auto-delay power/motion evaluation ----
            # This affects ONLY auto-delay metrics/gating path (hist_vel_*), not NN inputs.
            if AUTO_PWR_USE_ANGLE_DIFF_VEL:
                if auto_prev_ang_L is None or auto_prev_ang_R is None:
                    vel_eval_L = float(Lvel)
                    vel_eval_R = float(Rvel)
                else:
                    vel_eval_L = _finite_diff_vel_deg_s(
                        Lpos, auto_prev_ang_L, float(CTRL_HZ), AUTO_PWR_ANGLE_WRAP_DEG
                    )
                    vel_eval_R = _finite_diff_vel_deg_s(
                        Rpos, auto_prev_ang_R, float(CTRL_HZ), AUTO_PWR_ANGLE_WRAP_DEG
                    )
                auto_prev_ang_L = float(Lpos)
                auto_prev_ang_R = float(Rpos)
                auto_vel_source = 'angle_diff'
            else:
                vel_eval_L = float(Lvel)
                vel_eval_R = float(Rvel)
                auto_vel_source = 'imu_raw'

            # ---- auto-delay history (for ratio/energy evaluation) ----
            hist_tau_src_L.append(float(delay_input_L))
            hist_tau_src_R.append(float(delay_input_R))
            hist_vel_L.append(float(vel_eval_L))
            hist_vel_R.append(float(vel_eval_R))
            hist_ang_L.append(float(Lpos))
            hist_ang_R.append(float(Rpos))

            # ---- auto-delay metrics + optimization (1 Hz, does not touch 100 Hz inference path) ----
            # Metrics are always computed regardless of auto_delay_enable so the GUI always shows
            # live power ratio even when auto delay is OFF.  Only the "apply new delay" step is
            # gated by auto_delay_enable.
            now_wall = time.time()
            if (now_wall - auto_last_eval_ts) >= AUTO_UPDATE_INTERVAL_S:
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

                # Dwell 两种模式（见常量 AUTO_DWELL_ADAPTIVE）：
                #   adaptive=True  → max(AUTO_DWELL_S, auto_window_s + 1.0), 等窗口填满新数据
                #   adaptive=False → 固定 AUTO_DWELL_S, 响应更快（默认）
                if AUTO_DWELL_ADAPTIVE:
                    effective_dwell_s = max(AUTO_DWELL_S, auto_window_s + 1.0)
                else:
                    effective_dwell_s = AUTO_DWELL_S

                def _auto_step_leg(side_tau_np, side_vel_np, cur_delay_ms,
                                   last_change_ts, bo_state):
                    """
                    Run one 1Hz evaluation + optional scan/apply for a single leg.
                    Returns a dict with new state for that leg.
                    """
                    cur = evaluate_delay_candidate_leg(
                        cur_delay_ms, side_tau_np, side_vel_np, window_frames
                    )
                    if cur is None:
                        return {
                            'ratio': 0.0,
                            'pos_per_s': 0.0,
                            'neg_per_s': 0.0,
                            'best_delay_ms': float(cur_delay_ms),
                            'motion_valid': False,
                            'new_delay_ms': float(cur_delay_ms),
                            'last_change_ts': float(last_change_ts),
                            'changed': False,
                        }

                    motion_valid = (
                        cur['mean_abs_vel'] >= AUTO_VALID_MIN_MEAN_ABS_VEL_DPS and
                        cur['abs_power_per_s'] >= AUTO_VALID_MIN_ABS_POWER_W
                    )
                    cur_obj = auto_objective_from_eval(cur)
                    pos_per_s_disp = cur['pos_per_s'] * float(runtime_scale)
                    neg_per_s_disp = cur['neg_per_s'] * float(runtime_scale)
                    best_delay_ms = float(cur_delay_ms)
                    new_delay_ms = float(cur_delay_ms)
                    new_last_change = float(last_change_ts)
                    changed = False

                    if auto_delay_enable and motion_valid:
                        bo_record_sample(bo_state, cur_delay_ms, cur_obj)

                    if auto_delay_enable and motion_valid and (now_wall - last_change_ts) >= effective_dwell_s:
                        d_lo = max(0.0, cur_delay_ms - AUTO_SCAN_HALF_RANGE_MS)
                        d_hi = min(MAX_RUNTIME_DELAY_MS, cur_delay_ms + AUTO_SCAN_HALF_RANGE_MS)
                        if auto_opt_method == 'bo':
                            best_delay_ms = propose_bo_delay_candidate(
                                bo_state['x'], bo_state['y'],
                                cur_delay_ms, d_lo, d_hi, AUTO_SCAN_STEP_MS
                            )
                        else:
                            cand_delays = np.arange(
                                d_lo, d_hi + 0.5 * AUTO_SCAN_STEP_MS,
                                AUTO_SCAN_STEP_MS, dtype=np.float32
                            )
                            if cand_delays.size == 0:
                                cand_delays = np.array([cur_delay_ms], dtype=np.float32)

                            cands = []
                            for c_delay in cand_delays:
                                ev = evaluate_delay_candidate_leg(
                                    float(c_delay), side_tau_np, side_vel_np, window_frames
                                )
                                if ev is not None:
                                    cands.append(ev)

                            best = pick_best_delay_candidate(cands, cur_delay_ms, AUTO_TARGET_RATIO)
                            if best is not None:
                                best_delay_ms = best['delay_ms']

                        step_ms = max(
                            -AUTO_MAX_STEP_MS,
                            min(AUTO_MAX_STEP_MS, best_delay_ms - cur_delay_ms)
                        )
                        if abs(step_ms) >= 0.5:
                            new_delay_ms = max(
                                0.0,
                                min(MAX_RUNTIME_DELAY_MS, cur_delay_ms + step_ms)
                            )
                            new_last_change = now_wall
                            changed = True

                    return {
                        'ratio': cur['ratio'],
                        'pos_per_s': pos_per_s_disp,
                        'neg_per_s': neg_per_s_disp,
                        'best_delay_ms': best_delay_ms,
                        'motion_valid': motion_valid,
                        'new_delay_ms': new_delay_ms,
                        'last_change_ts': new_last_change,
                        'changed': changed,
                    }

                state_L = _auto_step_leg(
                    tau_src_L_np, vel_L_np, runtime_delay_ms_L, auto_last_change_ts_L, auto_bo_state_L
                )
                state_R = _auto_step_leg(
                    tau_src_R_np, vel_R_np, runtime_delay_ms_R, auto_last_change_ts_R, auto_bo_state_R
                )

                auto_cur_ratio_L = state_L['ratio']
                auto_cur_pos_per_s_L = state_L['pos_per_s']
                auto_cur_neg_per_s_L = state_L['neg_per_s']
                auto_best_delay_ms_L = state_L['best_delay_ms']
                auto_motion_valid_L = state_L['motion_valid']
                auto_last_change_ts_L = state_L['last_change_ts']
                if state_L['changed']:
                    runtime_delay_ms_L = state_L['new_delay_ms']
                    status_dirty = True

                auto_cur_ratio_R = state_R['ratio']
                auto_cur_pos_per_s_R = state_R['pos_per_s']
                auto_cur_neg_per_s_R = state_R['neg_per_s']
                auto_best_delay_ms_R = state_R['best_delay_ms']
                auto_motion_valid_R = state_R['motion_valid']
                auto_last_change_ts_R = state_R['last_change_ts']
                if state_R['changed']:
                    runtime_delay_ms_R = state_R['new_delay_ms']
                    status_dirty = True

            # ---- delay buffer (per-leg) ----
            delay_ms_L_eff = max(0.0, min(MAX_RUNTIME_DELAY_MS, float(runtime_delay_ms_L)))
            delay_ms_R_eff = max(0.0, min(MAX_RUNTIME_DELAY_MS, float(runtime_delay_ms_R)))
            delay_frames_L = max(0, min(int(round(delay_ms_L_eff / dt_ms)), BUF_SIZE - 1))
            delay_frames_R = max(0, min(int(round(delay_ms_R_eff / dt_ms)), BUF_SIZE - 1))

            delay_buf_L.appendleft(delay_input_L)
            delay_buf_R.appendleft(delay_input_R)
            sync_sid_buf.appendleft(cur_sample_id)
            sync_ang_L_buf.appendleft(float(Lpos))
            sync_ang_R_buf.appendleft(float(Rpos))
            sync_vel_L_buf.appendleft(float(Lvel))
            sync_vel_R_buf.appendleft(float(Rvel))
            L_cmd_shifted = delay_buf_L[delay_frames_L]
            R_cmd_shifted = delay_buf_R[delay_frames_R]

            L_cmd_final = L_cmd_shifted * runtime_scale
            R_cmd_final = R_cmd_shifted * runtime_scale

            sync_sample_id_L = int(sync_sid_buf[delay_frames_L]) & 0xFFFF
            sync_sample_id_R = int(sync_sid_buf[delay_frames_R]) & 0xFFFF
            sync_sample_id = max(sync_sample_id_L, sync_sample_id_R) & 0xFFFF
            sync_ang_L = float(sync_ang_L_buf[delay_frames_L])
            sync_ang_R = float(sync_ang_R_buf[delay_frames_R])
            sync_vel_L = float(sync_vel_L_buf[delay_frames_L])
            sync_vel_R = float(sync_vel_R_buf[delay_frames_R])
            # GUI power display should represent instantaneous power:
            # P(t) = tau_out(t) * vel_current(t)
            # where tau_out(t) is delayed+scaled command sent this cycle.
            # NOTE:
            # - sync_vel_* keeps delayed/synchronized velocity for signal inspection.
            # - ctrl_pwr_* uses current IMU velocity (Lvel/Rvel), not sync_vel_*.
            sync_ctrl_pwr_L = float(L_cmd_final) * float(Lvel) * (np.pi / 180.0)
            sync_ctrl_pwr_R = float(R_cmd_final) * float(Rvel) * (np.pi / 180.0)

            # ---- 发送给 Teensy ----
            send_torque(
                ser, L_cmd_final, R_cmd_final, L_p, L_d, R_p, R_d,
                sample_id=sync_sample_id,
                sync_ang_L=sync_ang_L, sync_ang_R=sync_ang_R,
                sync_vel_L=sync_vel_L, sync_vel_R=sync_vel_R,
                ctrl_pwr_L=sync_ctrl_pwr_L, ctrl_pwr_R=sync_ctrl_pwr_R,
            )

            # ---- 定期发送RPi状态 (供GUI显示) ----
            frame_count += 1
            if status_dirty or (frame_count % STATUS_SEND_INTERVAL == 0):
                send_status(ser, cur_filter_source, cur_filter_type_code,
                            cur_filter_order, cur_enable_mask, cur_cutoff_hz,
                            runtime_scale,
                            delay_ms_L=runtime_delay_ms_L,
                            delay_ms_R=runtime_delay_ms_R,
                            auto_enable=auto_delay_enable,
                            auto_method=auto_opt_method,
                            motion_valid_L=auto_motion_valid_L,
                            motion_valid_R=auto_motion_valid_R,
                            ratio_L=auto_cur_ratio_L,
                            ratio_R=auto_cur_ratio_R,
                            pos_per_s_L=auto_cur_pos_per_s_L,
                            pos_per_s_R=auto_cur_pos_per_s_R,
                            neg_per_s_L=auto_cur_neg_per_s_L,
                            neg_per_s_R=auto_cur_neg_per_s_R,
                            best_delay_L=auto_best_delay_ms_L,
                            best_delay_R=auto_best_delay_ms_R)
                status_dirty = False

            # ---- 日志 ----
            data_dict = {
                'Time_ms': f'{now:.4f}',
                'imu_LTx': f'{Lpos:.4f}',
                'imu_RTx': f'{Rpos:.4f}',
                'imu_Lvel': f'{Lvel:.4f}',
                'imu_Rvel': f'{Rvel:.4f}',
                'teensy_t_cs_u16': str(int(t_cs_u16)),
                'teensy_t_s_unwrapped': f'{teensy_t_s_unwrapped:.4f}',
                'auto_eval_Lvel': f'{vel_eval_L:.4f}',
                'auto_eval_Rvel': f'{vel_eval_R:.4f}',
                'auto_vel_source': auto_vel_source,
                'sync_sample_id': str(int(sync_sample_id)),
                'sync_LTx': f'{sync_ang_L:.4f}',
                'sync_RTx': f'{sync_ang_R:.4f}',
                'sync_Lvel': f'{sync_vel_L:.4f}',
                'sync_Rvel': f'{sync_vel_R:.4f}',
                'sync_ctrl_pwr_L': f'{sync_ctrl_pwr_L:.6f}',
                'sync_ctrl_pwr_R': f'{sync_ctrl_pwr_R:.6f}',
                'L_command_actuator': f'{L_cmd_final:.6f}',
                'R_command_actuator': f'{R_cmd_final:.6f}',
                'raw_LExoTorque': f'{L_cmd:.6f}',
                'raw_RExoTorque': f'{R_cmd:.6f}',
                'filtered_LExoTorque': f'{filter_L_cmd:.6f}',
                'filtered_RExoTorque': f'{filter_R_cmd:.6f}',
                'pd_zm_bias_L': f'{pd_zm_bias_L:.6f}',
                'pd_zm_bias_R': f'{pd_zm_bias_R:.6f}',
                'L_P': f'{L_p:.6f}',
                'L_D': f'{L_d:.6f}',
                'R_P': f'{R_p:.6f}',
                'R_D': f'{R_d:.6f}',
                'scale_runtime': f'{runtime_scale:.4f}',
                'torque_delay_ms_L': f'{delay_ms_L_eff:.3f}',
                'torque_delay_ms_R': f'{delay_ms_R_eff:.3f}',
                'auto_opt_method': auto_opt_method,
                'auto_delay_enable': str(int(bool(auto_delay_enable))),
                'auto_motion_valid_L': str(int(bool(auto_motion_valid_L))),
                'auto_motion_valid_R': str(int(bool(auto_motion_valid_R))),
                'auto_ratio_L': f'{auto_cur_ratio_L:.6f}',
                'auto_ratio_R': f'{auto_cur_ratio_R:.6f}',
                'auto_pos_per_s_L': f'{auto_cur_pos_per_s_L:.6f}',
                'auto_pos_per_s_R': f'{auto_cur_pos_per_s_R:.6f}',
                'auto_neg_per_s_L': f'{auto_cur_neg_per_s_L:.6f}',
                'auto_neg_per_s_R': f'{auto_cur_neg_per_s_R:.6f}',
                'auto_best_delay_ms_L': f'{auto_best_delay_ms_L:.3f}',
                'auto_best_delay_ms_R': f'{auto_best_delay_ms_R:.3f}',
                'auto_window_s': f'{auto_window_s:.3f}',
                'auto_gait_freq_hz': f'{auto_gait_freq_hz:.4f}',
                'tag': tag_to_log,
            }

            wr.writerow(data_dict)
            if time.time() - last_flush > 0.5:
                fcsv.flush()
                last_flush = time.time()

            # ---- 控制台输出 ----
            _zm_txt = (f' | zmL:{pd_zm_bias_L:6.2f} | zmR:{pd_zm_bias_R:6.2f}'
                       if pd_zm_enabled else '')
            print(f'| time:{now:6.2f}s | Lθ:{Lpos:7.2f}° | Rθ:{Rpos:7.2f}° | '
                  f'Lω:{Lvel:7.2f} | Rω:{Rvel:7.2f} | '
                  f'τL:{L_cmd_final:6.2f} | τR:{R_cmd_final:6.2f} | scale:{runtime_scale:4.2f} | '
                  f'Rp:{R_p:6.2f} | Rd:{R_d:6.2f}{_zm_txt} ')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nQuit.')
