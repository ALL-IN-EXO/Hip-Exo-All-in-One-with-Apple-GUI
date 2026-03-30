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

## 通讯协议

```
Pi TX (GPIO14) → Teensy RX8 (pin 34)
Pi RX (GPIO15) ← Teensy TX8 (pin 35)
共地连接
波特率: 115200
```

### 接收 (Teensy → Pi)

二进制帧: `A5 5A [LEN] [TYPE] [payload] [CHKSUM]`

- TYPE=0x01 (IMU): 5×float32 (Lpos, Rpos, Lvel, Rvel, exo_delay) + 11字节 logtag
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
