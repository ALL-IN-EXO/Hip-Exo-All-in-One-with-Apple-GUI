# RPi_Unified - 树莓派统一 RL 控制器

统一了原 `Deployment_4NN` (DNN) 和 `Deployment_2NN` (LSTM) 两套代码，一个入口支持所有网络类型。

---

## 目录结构

```
RPi_Unified/
├── RL_controller_torch.py        # 唯一入口
├── filter_library.py             # 滤波器工具库
├── models/                       # 神经网络权重文件
│   ├── dnn/                      # DNN 前馈网络 (.pt)
│   │   ├── Trained_model.pt
│   │   ├── Trained_model1.pt
│   │   ├── Trained_model2.pt
│   │   └── Trained_model3.pt
│   └── lstm/                     # LSTM 网络 (.pt)
│       ├── end2end/
│       │   ├── run/
│       │   ├── walk/
│       │   ├── walkv2/
│       │   └── walkv2_legdecp/
│       └── lstm/
├── networks/                     # 网络定义 (每个 class 一个文件)
│   ├── __init__.py
│   ├── base_network.py           # Network (nn.Module) 基础前馈网络
│   ├── dnn.py                    # DNN class
│   ├── lstm_network.py           # LSTMNetwork class
│   ├── lstm_leg_dcp.py           # LSTMNetworkLegDcp class
│   └── lstm_pd.py                # LSTMNetworkPD class (PD位置误差控制)
└── output/                       # 运行时 CSV 日志输出
```

---

## 快速使用

```bash
source ~/venvs/pytorch-env/bin/activate
cd RPi_Unified
python RL_controller_torch.py
```

### GUI 远程启动依赖（Pi 端）

如果使用 GUI 里的 `Start LegDcp / Start LSTM-PD / Stop Pi RL` 远程按钮，
Pi 端需要安装 `tmux`（GUI 通过 `tmux` 托管 RL 进程）：

```bash
sudo apt update
sudo apt install -y tmux
tmux -V
```

---

## 本地-树莓派自动同步 (代码下发 + 日志回传)

新增工具: `tools/rpi_sync.py`

推荐直接用 shell 包装:
- `tools/deploy_code.sh`：一次部署代码到 Pi
- `tools/pull_data_watch.sh`：持续回拉 `output/` 数据

默认行为:
- 运行一次(不带 `--watch`): 只把本地代码推送到 Pi
- Watch 模式(带 `--watch`): 只把 Pi 的 `output/` 拉回本地

### 1) 先配置免密 SSH (推荐)

```bash
ssh-copy-id aboutberlin@192.168.31.34
```

### 2) 一次部署代码到 Pi (手动执行)

```bash
cd RPi_Unified
./tools/deploy_code.sh
```

### 3) 持续回拉数据 (每2秒, 仅 Pi -> 本地)

```bash
cd RPi_Unified
./tools/pull_data_watch.sh
```

自定义间隔（例如 1 秒）:

```bash
./tools/pull_data_watch.sh 1
```

### 4) 常用参数

```bash
# 仅推送代码
python tools/rpi_sync.py --direction push

# 仅拉取 output
python tools/rpi_sync.py --direction pull

# 一次执行“推+拉”
python tools/rpi_sync.py --direction both

# 如需严格镜像到远端 (会删除远端多余文件)
python tools/rpi_sync.py --delete-remote
```

> 现在脚本在 watch 模式下默认只拉数据，不会循环推代码。

---

## 输出 CSV 可视化小程序 (时间段选择 + 简单分析)

新增工具: `tools/rpi_output_viewer.py`

功能:
- 自动读取 `output/` 下最新 `PI5_*.csv` (也可指定文件)
- 拖拽选择时间段
- 同时看速度、角度、力矩、功率
- 右侧自动统计: 均值/峰值/RMS/做功等

### 运行

```bash
cd RPi_Unified
python tools/rpi_output_viewer.py
```

或指定文件:

```bash
python tools/rpi_output_viewer.py ./output/PI5_dnn-20260320-191126.csv
```

可选参数:

```bash
# 力矩来源: filtered/raw/actuator/auto
python tools/rpi_output_viewer.py --torque-source filtered
```

依赖:

```bash
pip install pandas numpy matplotlib
```

---

## 如何切换网络

通过命令行参数 `--nn` 选择网络类型:

```bash
python RL_controller_torch.py --nn dnn             # DNN前馈网络 (默认)
python RL_controller_torch.py --nn lstm             # LSTM网络
python RL_controller_torch.py --nn lstm_leg_dcp     # LSTM每腿独立解耦
python RL_controller_torch.py --nn lstm_pd          # LSTM PD位置误差控制
python RL_controller_torch.py --nn lstm_pd --tag outdoor_walk  # 自定义日志标签
```

kp/kd 和模型路径在 `RL_controller_torch.py` 顶部配置区修改。

---

## 网络对比

| | DNN | LSTMNetwork | LSTMNetworkLegDcp | LSTMNetworkPD |
|---|---|---|---|---|
| **NN_TYPE** | `'dnn'` | `'lstm'` | `'lstm_leg_dcp'` | `'lstm_pd'` |
| **结构** | 18→128→64→2 前馈 | 4→256(×2层)→2 LSTM | 每腿 2→256(×2层)→1 LSTM | 每腿 2→256(×2层)→1 LSTM |
| **输入** | 历史2帧+当前+历史输出 = 18维 | 左右角度+速度 = 4维 | 同左，内部拆成每腿2维 | 同左，内部拆成每腿2维 |
| **滤波器** | 5个接口 (input_pos, input_vel, vel, ref, torque) | 仅 exo_filter (torque前) | 仅 exo_filter (torque前) | 仅 exo_filter (torque前) |
| **torque 计算** | `((qHr*0.1 - qTd)*50 - dqTd_filtered*14.142)*0.008` | `0.1*action*kp + dqTd*kd` | `0.2*action*kp + dqTd*kd` | `(action-qTd)*kp - dqTd*kd` |
| **kp/kd 推荐值** | kp=50, kd=14.142 | kp=50, kd≈3.536 | kp=50, kd≈3.536 | kp=50, kd≈3.536 |

---

## 滤波器配置

### LSTM 模式

只需设置 torque 前的低通滤波器系数：

```python
LSTM_FILTER_B = np.array([0.0913, 0.1826, 0.0913])   # 20Hz Butterworth 2阶
LSTM_FILTER_A = np.array([1.0, -0.9824, 0.3477])
```

常用系数 (100Hz 采样率, Butterworth 2阶)：

| 截止频率 | b | a |
|---------|---|---|
| 3Hz  | [0.0006, 0.0012, 0.0006] | [1.0, -1.9289, 0.9314] |
| 6Hz  | [0.0055, 0.0111, 0.0055] | [1.0, -1.7786, 0.8008] |
| 12Hz | [0.0461, 0.0923, 0.0461] | [1.0, -1.3073, 0.4918] |
| 15Hz | [0.0675, 0.1349, 0.0675] | [1.0, -1.1430, 0.4128] |
| 20Hz | [0.0913, 0.1826, 0.0913] | [1.0, -0.9824, 0.3477] |

### DNN 模式

支持三种配置方式，通过 `FILTER_CONFIG_MODE` 选择：

- **`'preset'`** — 使用预定义滤波器名称 (如 `'butter_12hz_2nd'`)，详见 `filter_library.py`
- **`'custom'`** — 指定滤波器类型+截止频率+阶数，自动计算系数
- **`'coeffs'`** — 直接给 b/a 系数

---

## Auto Delay 自动延迟优化

### 概述

RL 控制器内置 1Hz 侧环，自动优化 `torque_delay_ms` 以最大化正功占比（`power_ratio`）。  
目标：`ratio ≥ 0.95` → 在满足后最大化 `pos_per_s`（每秒正功）。

### 开关方式

通过 GUI RL 面板的 **Auto Delay** 开关控制。  
`auto_delay_enable` 经 BLE passthrough 下发至 RPi（`rpi_passthru[20]` bit0）。

### 重要：Auto OFF 时功率指标仍然实时计算

**Auto Delay 关闭时，`power_ratio` / `pos_per_s` / `neg_per_s` 依然每秒更新并回传 GUI。**

- 1Hz 评估不依赖 `auto_delay_enable`，每轮都执行 `evaluate_delay_candidate_leg`
- 只有"扫描候选 + 写入新 delay"这一步才被 `if auto_delay_enable` 守护
- 因此 GUI 的 Power Sign overlay 数字在 Auto OFF 时仍然有效，可用于人工判断助力质量

### 左右腿独立

v3.1 起，`runtime_delay_ms_L/R` 各自独立漂移：

- Auto OFF 或 cfg 下发：两腿同步到 GUI 的单一 `delay_ms` 值
- Auto ON：两腿各自扫描、各自 dwell（当前默认 0.5s）、各自 best delay

### 功率指标定义

```
P(t) = tau(t) * vel(t) * π/180        # tau: Nm, vel: deg/s
W+   = ∫P>0 dt / T_window             # 正功 (W·s 归一化)
ratio = W+ / (W+ + |W-|)              # 正功占比 [0,1]
pos_per_s = W+ / T_window             # 每秒正功 (W)
neg_per_s = W- / T_window             # 每秒负功 (W, ≤0)
```

评估时内部固定 `scale=1.0`（与 `runtime_scale` 解耦），回传 GUI 前乘 `runtime_scale`。

### 评估速度源（硬开关，不走 GUI）

用于 `power_ratio` / `pos_per_s` / `neg_per_s` 评估的速度 `vel` 可通过顶部常量切换：

- `AUTO_PWR_USE_ANGLE_DIFF_VEL = True`（默认）：
  - 使用 `d(hip_angle)/dt`（deg/s）
  - 左右腿独立差分状态
  - 带角度回绕保护（`AUTO_PWR_ANGLE_WRAP_DEG`，默认 ±180°）
- `AUTO_PWR_USE_ANGLE_DIFF_VEL = False`：
  - 使用 IMU 原始角速度 `LTAVx/RTAVx`

> 注意：该开关仅影响 Auto Delay 评估链路（`hist_vel`），不改变 NN 主控制输入速度链路。

### 关键参数（`RL_controller_torch.py` 顶部）

| 常量 | 默认值 | 说明 |
|------|--------|------|
| `AUTO_TARGET_RATIO` | 0.95 | 主目标：正功占比阈值 |
| `AUTO_DWELL_S` | 0.5s | 变化后的冷却时间 |
| `AUTO_DWELL_ADAPTIVE` | False | True=自适应 dwell≥窗口长度 |
| `AUTO_SCAN_HALF_RANGE_MS` | 100ms | 扫描范围 ±100ms |
| `AUTO_SCAN_STEP_MS` | 10ms | 扫描步长 |
| `AUTO_MAX_STEP_MS` | 40ms | 单次最大变化量 |
| `AUTO_PWR_USE_ANGLE_DIFF_VEL` | True | AutoDelay 评估速度源（角度差分/IMU原始） |
| `AUTO_PWR_ANGLE_WRAP_DEG` | 180° | 角度差分回绕阈值 |
| 窗口 | 8~8s | 当前默认固定 8s 窗口 |

---

## 功率显示链路对齐（计划 / 未实施）

**状态**：设计已冻结，代码未改。记录在此是为了下次接手时可直接照做。

### 背景 —— 现在 GUI 实时功率波形是偏相的

GUI 面板上和"功率"相关的显示分两类：

| 元件 | 数据来源 | 是否正确 |
|---|---|---|
| `+Ratio xx %` / `+P / −P` 文本 overlay | RPi passthru (`rpi_blob[24..39]`) | ✅ RPi 权威，τ·ω 同瞬 |
| 紫色 Power 折线 (`L_pwr_line`/`R_pwr_line`) | GUI 本地 `L_vel_BLE × L_tau_d_BLE × π/180` (`GUI.py:3311-3312`) | ❌ 偏相 60–80ms |
| 绿/红 Power 条带 (`_update_power_strip`) | 同上 `L_pwr_buf` | ❌ 偏相 |
| CSV 里的 `L_pwr_W`/`R_pwr_W` | 同上 `_append_data_point` 写入 | ❌ 偏相 |

**偏相根因**（详见 `Code Debug/BUG_ANALYSIS.md`）：BLE 上行单帧里的 `L_cmd_Nm` 是 RPi 几十毫秒前发的（经 Serial8→Teensy→20 Hz BLE 打包→BLE radio），而 `L_vel_dps` 是 Teensy 同帧现采的。两者相乘得到的 (τ·ω) 在 0.65 Hz 步态下有 23–47° 相移，足以让符号统计翻号。RPi 侧逐行 (cmd, vel) 是因果配对 (vel→NN→cmd)，所以 RPi CSV 的正功占比 ≈ 83 %，GUI CSV 的 ≈ 76 %。**数据本身没 bug，是 GUI 本地拼对造成的物理偏相。**

### 方案 B：RPi 直接透传 `P_inst` —— 选定方案

让 RPi 在每个 NN tick 算出瞬时功率 `P_inst = L_cmd_final × Lvel × (π/180)`（τ 和 ω 已经在同一行同一时刻配对），通过 40 B status passthru 发给 Teensy，Teensy **不改代码**，原样转发到 BLE 上行，GUI 直接取用。

- RPi 侧：`RL_controller_torch.py:1463` 后一行加 `L_pwr_inst = L_cmd_final * Lvel * (π/180)`（右腿同理）；把这两个值塞进 `send_status`。
- 频率：现在 `STATUS_SEND_INTERVAL = 50` 帧（2 Hz）不够，必须改成 `5`（20 Hz），和 BLE 上行同步。Serial8 占用从 ~23 % 升到 ~30 %，仍宽裕。
- GUI 侧：`_append_data_point`（`GUI.py:3298-3322`）已经有 `power_override` 入口，原本只给 replay 用；live 模式当 `_rpi_online && algo==RL && _rpi_pwr_inst_valid` 时改走 `power_override=(pwr_inst_L, pwr_inst_R)`。非 RL 或 RPi 掉线时回退到本地算（保 DNN 等旧模式可用）。
- Teensy 零改动，40 B passthru 透明转发。

### 废弃方案 A（为什么不做）

Option A 曾计划：Serial8 `PI_TORQUE_SIZE` 26→42 B，RPi 把自己用作 NN 输入的 (Lpos, Rpos, Lvel, Rvel) echo 回 Teensy，Teensy RL 模式下用 echo 值覆盖 BLE 上行里的 angle/vel 字段，这样 GUI 本地 `L_vel × L_tau_d` 自然对齐。**放弃原因**：同样的效果，B 只改两侧各几行 + 不动 Teensy，A 要改三侧而且扩协议，多此一举。

### 40 B status passthru 布局变更（v3 → v4）

当前 v3 布局（`RL_controller_torch.py:942` `send_status` 内注释，**勿重复改注释**）已用满 40 B，`[21..23]` 只有 3 B 空档，不够塞两个 int16。方案 B 借助"把两个精度过剩的 float32 降级成 int16"腾出 4 B：

```
v4 布局（差异字段加 ★）
[0..1]   magic 'RL'
[2]      version = 4          ★ 原 3
[3]      nn_type
[4..7]   filter_source / type_code / order / enable_mask
[8..9]   cutoff_hz   int16 ×10      ★ 原 float32 @ 8..11；0.1 Hz 精度足够
[10..11] scale       int16 ×1000    ★ 原 float32 @ 12..15；0.001 精度足够
[12..13] P_inst_L    int16 ×100     ★ 新增，±327.68 W 量程
[14..15] P_inst_R    int16 ×100     ★ 新增
[16..17] delay_ms_L  int16 ×10
[18..19] delay_ms_R  int16 ×10
[20]     auto_flags
[21..23] reserved (3 B)
[24..25] ratio_L     int16 ×10000
[26..27] ratio_R     int16 ×10000
[28..29] pos_per_s_L int16 ×100
[30..31] pos_per_s_R int16 ×100
[32..33] neg_per_s_L int16 ×100
[34..35] neg_per_s_R int16 ×100
[36..37] best_delay_L int16 ×10
[38..39] best_delay_R int16 ×10
```

量程核算：hip exo 峰值扭矩 15 Nm × 峰值角速度 ~400 °/s × π/180 ≈ 105 W，±327.68 W 充裕；cutoff 常用 20–40 Hz，int16 ×10 量程 6553 Hz；scale 常用 0.4–1.0，int16 ×1000 量程 ±32.767。

### 改动清单（按顺序）

1. `RL_controller_torch.py`
   - `STATUS_SEND_INTERVAL = 50 → 5`
   - `send_status` 加 `pwr_inst_L`, `pwr_inst_R` 入参；`buf[2] = 0x04`；按 v4 布局 `_pack_i16`
   - 主循环 `L_cmd_final` / `R_cmd_final` 之后算 `L_pwr_inst = L_cmd_final * Lvel * (π/180)`，右腿同理；传进 `send_status`
2. `GUI.py`
   - 新增常量 `RPI_STATUS_VERSION_PWR_INST = 4`；`self._rpi_pwr_inst_L/_R` 状态 + `_rpi_pwr_inst_valid` + 同步超时机制（参考现有 `_rpi_status_valid` 写法）
   - `_handle_uplink` 解析器（`GUI.py:5430` 附近）加 v4 分支：新偏移读 cutoff/scale/P_inst；v3 分支保留兼容
   - `_append_data_point`（`GUI.py:3298-3312`）在本地计算分支前插一个条件：live+RL+valid 时走 `power_override`
   - RPi 掉线 / status 过时 → `_rpi_pwr_inst_valid = False`，自动回退本地算
3. Teensy：**零改动**
4. 文档：`Docs/SYSTEM_ARCHITECTURE.md` §10.6 更新 v4 布局；`Docs/CHANGELOG.md` `[Unreleased]` 加一条

### 向前兼容

- 旧 RPi (v3) + 新 GUI：GUI parser 按 `version == 3` 走老分支 → `_rpi_pwr_inst_valid` 恒 False → live 模式回退本地算 → 和现在行为一致，不会崩。
- 新 RPi (v4) + 旧 GUI：旧 parser 检查 `version >= 3` 放行后按 v3 偏移读，会把 int16 cutoff/scale 解释为错误的 float32 → 要么 NaN 要么乱码 → 旧 GUI 会因 `isfinite` 检查 `valid_sample=False`，退回到显示 0 / HOLD。不会崩但会退化。部署时应同步更新 GUI。

---

## 通讯协议

```
Pi TX (GPIO14) → Teensy RX8 (pin 34)
Pi RX (GPIO15) ← Teensy TX8 (pin 35)
共地连接
波特率: 115200
```

### 接收 (Teensy → Pi)

二进制帧: `A5 5A [LEN] [TYPE] [payload] [CHKSUM]`

- TYPE=0x01 (IMU): 4×float32 (Lpos, Rpos, Lvel, Rvel) + 11字节 logtag
- TYPE=0x02 (GUI透传): 40字节 passthrough payload

### 发送 (Pi → Teensy)

`AA 55` + 6×float32 (tau_L, tau_R, L_p, L_d, R_p, R_d)

---

## 来源

| 本文件夹 | 原始来源 |
|---------|---------|
| `networks/dnn.py` | `Deployment_4NN/DNN_torch_end2end.py` 中的 DNN class |
| `networks/lstm_network.py` | 同上文件中的 LSTMNetwork class |
| `networks/lstm_leg_dcp.py` | 同上文件中的 LSTMNetworkLegDcp class |
| `networks/lstm_pd.py` | `Deployment/DNN_torch_end2end.py` 中的 LSTMNetworkPD class |
| `networks/base_network.py` | 同上文件中的 Network class |
| `filter_library.py` | `Deployment_4NN/filter_library.py` |
| `RL_controller_torch.py` | 基于 `Deployment_2NN/RL_controller_torch.py` 扩展 |
| `models/dnn/` | `Deployment_4NN/*.pt` |
| `models/lstm/` | `Deployment_2NN/nn_para/` |
