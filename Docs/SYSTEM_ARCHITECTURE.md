# Hip Exoskeleton System Architecture

## 1. Devices

Teensy 4.1, SIG / T-Motor (x2), IM948 IMU (x6), BLE Transceiver for IMU, Adafruit ItsyBitsy nRF52840 (x2, GUI link), Raspberry Pi 5, Host PC, SD Card (on Teensy)

---

## 2. Teensy Serial Ports & Communication Protocols

| Serial Port | Baud Rate | Connected Device | Protocol | Frequency | Direction |
|---|---|---|---|---|---|
| **Serial (USB)** | 115200 | PC debug console | ASCII text | 10 Hz print | TX |
| **Serial1** | 460800 | IMU P1 (via BLE transceiver) | IM948 binary | 100 Hz | RX/TX |
| **Serial2** | 460800 | IMU P2 (via BLE transceiver) | IM948 binary | 100 Hz | RX/TX |
| **Serial3** | 460800 | IMU P3 (via BLE transceiver) | IM948 binary | 100 Hz | RX/TX |
| **Serial4** | 460800 | IMU P4 (via BLE transceiver) | IM948 binary | 100 Hz | RX/TX |
| **Serial5** | 115200 | BLE Transceiver (GUI link) | **128-byte** binary frames | TX: 20 Hz, RX: event-driven | RX/TX |
| **Serial6** | 460800 | IMU Right Thigh (via BLE transceiver) | IM948 binary | 100 Hz | RX/TX |
| **Serial7** | 460800 | IMU Left Thigh (via BLE transceiver) | IM948 binary | 100 Hz | RX/TX |
| **Serial8** | 115200 | Raspberry Pi 5 | Binary frames (36B TX / 26B RX) | 100 Hz | RX/TX |
| **CAN3** | 1 Mbps | Motors (x2) | CAN 2.0 (SIG/T-Motor) | 100 Hz | RX/TX |

---

## 3. Data Flow

### 3.1 System Diagram

```
                                    +-----------+
                                    |  Host PC  |
                                    |  (GUI.py) |
                                    +-----+-----+
                                          |
                                     USB Serial
                                     115200 baud
                                          |
                              +-----------+-----------+
                              | BLE Central           |
                              | (ItsyBitsy nRF52840)  |
                              +-----------+-----------+
                                          |
                                    BLE UART
                                 128-byte frames
                                 MTU=247, 2Mbps PHY
                                          |
                              +-----------+-----------+
                              | BLE Peripheral        |
                              | (ItsyBitsy nRF52840)  |
                              +-----------+-----------+
                                          |
                                    Serial5 (UART)
                                    115200 baud
                                          |
+------------------+              +-------+--------+              +-----------------+
|   Raspberry Pi 5 | -- Serial8 --| Teensy 4.1     |-- CAN3 1Mbps--|  Motors (x2)   |
|   (RL Neural Net)|   115200     | (Main Control) |              | SIG / T-Motor   |
+------------------+              +-------+--------+              +-----------------+
                                          |
                              Serial1,2,3,4,6,7
                              460800 baud each
                                          |
                              +-----------+-----------+
                              | BLE Transceiver (IMU) |
                              +-----------+-----------+
                                          |
                                    Wireless
                                          |
                         +----------------+----------------+
                         |        |        |        |      |        |
                       IMU_L   IMU_R   IMU_1   IMU_2   IMU_3   IMU_4
                      (Ser7)  (Ser6)  (Ser1)  (Ser2)  (Ser3)  (Ser4)
                       Left    Right   Pos1    Pos2    Pos3    Pos4
                       Thigh   Thigh
```


---

## 4. Key Code Locations

### 4.1 当前使用的代码 (Active)

| Component | Path | Description |
|---|---|---|
| **Teensy 主固件** | `All_in_one_hip_controller_RL_update/All_in_one_hip_controller_RL_update.ino` | 多算法主循环 |
| Motor driver 抽象层 | `All_in_one_hip_controller_RL_update/MotorDriver.h` | SIG/TMOTOR 运行时切换 |
| BLE 协议 (128B) | `All_in_one_hip_controller_RL_update/BleProtocol.h` | 上下行帧打包/解析 |
| Controller 基类 | `All_in_one_hip_controller_RL_update/Controller.h` | 算法抽象接口 |
| EG 算法 | `All_in_one_hip_controller_RL_update/Controller_EG.h/.cpp` | Energy Gate (~12 params) |
| Samsung 算法 | `All_in_one_hip_controller_RL_update/Controller_Samsung.h/.cpp` | Samsung (kappa, delay) |
| RL 算法 | `All_in_one_hip_controller_RL_update/Controller_RL.h/.cpp` | Serial8 ↔ RPi 透传 |
| Test 模式 | `All_in_one_hip_controller_RL_update/Controller_Test.h` | 固定/正弦力矩 |
| Motor SIG | `All_in_one_hip_controller_RL_update/Motor_Control_Sig.h/.cpp` | ODrive CAN 驱动 |
| Motor TMOTOR | `All_in_one_hip_controller_RL_update/Motor_Control_Tmotor.h/.cpp` | VESC CAN 驱动 |
| IMU 驱动 | `All_in_one_hip_controller_RL_update/im948_CMD.h`, `im948_CMD.ino` | IM948 协议 |
| IMU 适配 | `All_in_one_hip_controller_RL_update/IMU_Adapter.h/.cpp` | 6路IMU统一接口 |
| SD 日志 | `All_in_one_hip_controller_RL_update/sdlogger.h/.cpp` | Teensy SD 卡日志 |
| **GUI** | `GUI_RL_update/GUI.py` | 参数调节 + 实时绘图 |
| **RPi RL 控制器** | `RPi_Unified/RL_controller_torch.py` | 唯一入口, CLI 选网络 |
| RPi 神经网络 | `RPi_Unified/networks/dnn.py`, `lstm_network.py`, `lstm_leg_dcp.py`, `lstm_pd.py` | 4种网络定义 |
| RPi 网络基类 | `RPi_Unified/networks/base_network.py` | Network (nn.Module) |
| RPi 滤波器库 | `RPi_Unified/filter_library.py` | IIR/EMA/Kalman 滤波器 |
| RPi DNN 模型 | `RPi_Unified/models/dnn/Trained_model*.pt` | DNN 权重文件 |
| RPi LSTM 模型 | `RPi_Unified/models/lstm/end2end/` | LSTM 权重 (按 motion_type 分目录) |

### 4.2 历史/参考代码 (Archive)

| Component | Path | Description |
|---|---|---|
| 旧版 Teensy (无算法切换) | `All_in_one_hip_controller/` | 不支持多算法切换的旧固件 |
| 旧版 GUI (无 RL) | `GUI/` | 不含 RL 面板的旧版 GUI |
| 旧部署代码 (参考) | `Deployment/RL_controller_torch.py` | 单 LSTM_PD 部署脚本 (已验证可用) |
| 旧部署 NN 定义 (参考) | `Deployment/DNN_torch_end2end.py` | 所有 NN class 的原始定义 |
| 其他归档 | `Archive_Old_Reference/` | 历史版本 |

---

## 5. Multi-Algorithm Architecture (v3.0 Update)

### 5.1 Overview

The system now supports **runtime-switchable control algorithms** via GUI, without recompilation. All algorithms share a common `Controller` base class interface.

```
All_in_one_hip_controller_RL_update/
├── All_in_one_hip_controller_RL_update.ino  ← 主循环（精简）
├── MotorDriver.h                      ← 电机抽象层 (SIG/TMOTOR 运行时切换)
├── BleProtocol.h                      ← 128字节帧打包/解析
├── Controller.h                       ← 算法抽象基类
├── Controller_EG.h / .cpp             ← Energy Gate 算法 (~12 params)
├── Controller_Samsung.h / .cpp        ← Samsung 算法 (2 params: kappa, delay)
├── Controller_RL.h / .cpp             ← RL 神经网络 (Serial8 ↔ RPi)
├── Controller_Test.h                  ← 测试模式 (固定力矩)
├── Motor_Control_Sig.h / .cpp         ← SIG (ODrive) 底层驱动
├── Motor_Control_Tmotor.h / .cpp      ← TMOTOR (VESC) 底层驱动
├── IMU_Adapter.h / .cpp               ← 6路IMU适配
├── sdlogger.h / .cpp                  ← SD 日志
└── im948_CMD.h / .ino                 ← IMU 协议
```

### 5.2 Controller Interface

所有算法实现以下接口（定义在 `Controller.h`）:

```cpp
class Controller {
  virtual void compute(const CtrlInput& in, CtrlOutput& out) = 0;
  virtual void parse_params(const BleDownlinkData& dl) = 0;
  virtual void reset() = 0;
  virtual const char* name() const = 0;
  virtual AlgoID id() const = 0;
};
```

- `CtrlInput`: IMU 角度、滤波值、步态频率、控制周期等
- `CtrlOutput`: 左/右腿输出扭矩 (Nm, 输出轴)
- 切换算法时自动调用 `reset()` 清零内部状态

### 5.3 算法实例 (静态分配)

```cpp
static Controller_EG      ctrl_eg;
static Controller_Samsung ctrl_samsung;
static Controller_RL      ctrl_rl;
static Controller_Test    ctrl_test;
Controller* active_ctrl = &ctrl_eg;  // 运行时指针
```

**没有 `new`/`delete`**，所有算法实例在编译时静态分配。

### 5.4 主循环流程

```
loop() {
  1. 处理异步命令 (IMU reinit / Motor reinit / Brand switch)
  2. 读取 IMU
  3. 安全检测 (角度 > 80° 则禁用)
  4. [100Hz 节拍]:
     a. 接收 CAN 电机反馈
     b. 接收 BLE 下行 → 解析 → 算法参数更新 / 算法切换
     c. IMU 滤波 + 步态频率估计
     d. 填充 CtrlInput
     e. active_ctrl->compute(input, output)
     f. 下发电机命令 (通过 MotorDriver 抽象层)
     g. SD 日志
  5. [20Hz 节拍]: BLE 上行发送
  6. [10Hz]: 串口调试打印
}
```

---

## 6. BLE Protocol (128 bytes)

### 6.1 帧格式

BLE 上下行均为 128 字节帧: `[0xA5] [0x5A] [0x80] [125 bytes payload]`

BLE 中间板 (ItsyBitsy) 的 `rs232_datalength` 和 `ble_datalength` 均已设为 128。

### 6.2 上行帧 (Teensy → GUI)

| Byte | Content | Type | Description |
|------|---------|------|-------------|
| 0-2 | Header | - | 0xA5 0x5A 0x80 |
| 3-4 | t_cs | uint16 | 时间戳 (厘秒) |
| 5-6 | L_ang | int16 | 左腿角度 ×100 |
| 7-8 | R_ang | int16 | 右腿角度 ×100 |
| 9-10 | L_tau | int16 | 左实测扭矩 ×100 |
| 11-12 | R_tau | int16 | 右实测扭矩 ×100 |
| 13-14 | L_cmd | int16 | 左命令扭矩 ×100 |
| 15-16 | R_cmd | int16 | 右命令扭矩 ×100 |
| 17 | imu_ok | uint8 | IMU 状态 |
| 18-19 | mt | int16 | max_torque ×100 |
| 20 | sd_ok | uint8 | SD 状态 |
| 23-24 | gf | int16 | 步态频率 ×100 |
| 25 | tag_ok | uint8 | logtag 是否有效 |
| 26 | tag_ch | uint8 | logtag 首字母 |
| 27 | imu_bits | uint8 | 6路IMU位图 |
| 28 | brand | uint8 | 当前电机品牌 |
| 29-30 | temp | int8×2 | 电机温度 L/R |
| 31 | algo | uint8 | 当前活动算法 ID |
| 32-60 | reserved | - | Teensy 预留 |
| **61-100** | **RPi passthru** | **raw** | **RPi 透传区 (40B)** |
| 101-127 | reserved | - | 预留 |

### 6.3 下行帧 (GUI → Teensy)

| Byte (payload) | Content | Type | Description |
|------|---------|------|-------------|
| 0 | algo_select | uint8 | 0=EG, 1=Samsung, 2=RL, 3=Test |
| 1 | brand_req | uint8 | 0=不变, 1=SIG, 2=TMOTOR |
| 2 | ctrl_flags | uint8 | bit0=imu_reinit, bit1=motor_reinit, bit2-3=dir |
| 3-4 | max_torque | int16 | max_torque_cfg ×100 |
| 5-57 | algo_params | varies | 算法专用参数 (53B, 根据 algo 解析) |
| **58-97** | **RPi passthru** | **raw** | **RPi 透传区 (40B)** |
| 98-124 | reserved | - | 预留 |

**算法参数区 [5..57] 布局因算法不同**，详见 `BleProtocol.h`。

---

## 7. Motor Driver Abstraction

### 7.1 品牌与 ID 映射

| 品牌 | 左腿 (M2) | 右腿 (M1) |
|------|-----------|-----------|
| SIG (ODrive) | node_id = 1 | node_id = 2 |
| TMOTOR (VESC) | drive_id = 104 | drive_id = 105 |

### 7.2 CAN 初始化策略

**真机验证结论（不可改动）**:
- 单纯 `extern` 共享全局 `Can3` → 持续发送后 `Can3.write()` 失稳/卡死
- 正确做法: **全局 Can3 + 类内 Can3 + 二次初始化**
  - `.ino` 保留全局 `Can3`, `initial_CAN()` 做全局 `begin()` + `setBaudRate()`
  - 每个电机类内有自己的 `Can3` 成员, `init()` 里调 `hw_.initial_CAN()` 做类内初始化
  - 电机类用类内 `Can3.write()` 发送
  - `.ino` 用全局 `Can3.read()` 接收反馈

### 7.3 运行时切换

GUI 发送 `brand_request` 字段 → Teensy `switch_motor_brand()`:
1. 停当前电机 (`stop()`)
2. 清空 CAN 缓冲
3. 切换 `motor_L` / `motor_R` 指针
4. 重新 `initial_CAN()` + `init()`
5. 归零控制状态

---

## 8. RPi ↔ GUI 透传

BLE 帧中保留了 **RPi 透传区** (上行 [61..100], 下行 [58..97], 各 40 字节)。Teensy 不解析这些字节，原样透传:

```
GUI → BLE → Teensy → Serial8 → RPi   (下行透传)
RPi → Serial8 → Teensy → BLE → GUI   (上行透传)
```

用途: 在不修改 Teensy 固件的前提下，GUI 和 RPi 之间传递 RL 超参数、调试信息等。

---

## 9. GUI (v4.0+)

### 9.1 核心功能

- **128 字节帧**: 上下行均 128B，与 BLE 板对齐
- **算法"选择+确认"模式**: ComboBox 选择算法后需点击 CONFIRM 才发送到 Teensy，Active 标签显示实际运行算法
- **双重方向控制**: "Motor" 按钮控制真实电机方向 (发送到 Teensy)；"Plot" 按钮仅翻转可视化显示
- **连接健康监测**: 2 秒无数据自动标记断连 (黄色)，自动 Power OFF
- **速度独立右轴**: 角速度有独立的第三Y轴
- **缓冲控制**: Clear / Win / Auto 控件
- **IMU 1-4 角度显示**: OK 状态下显示角度数值
- **正弦波测试模式**: Test 面板支持 Constant / Sin Wave 波形
- **暗色/亮色主题**: Dark/Light 切换
- **电机品牌切换**: SIG / TMOTOR 下拉菜单
- **IMU / Motor reinit 按钮**

### 9.2 RL 面板功能

- **RPi 在线/离线状态**: 根据 RPi 状态帧判断 (2s 超时变红色 "RPi: Offline")
- **当前 NN 类型显示**: 从 RPi 状态帧读取并显示 (DNN / LSTM / LSTM-LegDcp / LSTM-PD)
- **滤波器控件**: 所有网络类型显示 Filter Type + Cutoff + Torque 开关; DNN 额外显示 Vel+Ref 开关
- **参数寄存 + Apply 按钮**: RL 参数 (Scale / Delay / Cutoff 等) 不实时下发，需点 "Apply RL Settings" 才发送
- **DNN 预设参数**: 切换到 DNN 时自动填充推荐值 (Scale=0.50, Delay=200ms, Cutoff=2.5Hz, Vel+Ref+Torque 全开)
- **动态参数面板**: QStackedWidget 根据算法显示不同参数控件
  - EG: ~9 个旋钮 (gate_k, scale_all, ext_gain 等)
  - Samsung: 2 个旋钮 (kappa, delay_ms)
  - RL: Scale + Delay + Filter Type/Cutoff/Order + Enable + Apply 按钮
  - Test: 波形选择 + 幅值 + 频率

### 9.3 下行打包

```python
payload[0] = algo_select    # 0=EG, 1=Samsung, 2=RL, 3=Test
payload[1] = brand_request  # 0=不变, 1=SIG, 2=TMOTOR
payload[2] = ctrl_flags     # bit0=imu_reinit, bit1=motor_reinit, bit2-3=dir
payload[3:5] = max_torque_cfg × 100  # int16
payload[5:58] = 算法专用参数 (根据 algo 填充)
# Test algo: [7]=waveform(0=const,1=sin), [8..9]=freq_hz×100
payload[58:98] = RPi 透传区 (仅 algo=RL 时打包, 见 §10.4)
```

### 9.4 上行解析

```
[31]     algo: 当前活动算法 ID
[32..39] IMU 1-4 角度 (TX1..TX4 ×100, int16)
[61..100] RPi 透传区 (40B, 见 §10.5)
```

---

## 10. RPi_Unified (树莓派统一 RL 控制器)

### 10.1 目录结构

```
RPi_Unified/
├── RL_controller_torch.py        # 唯一入口 (CLI选网络)
├── filter_library.py             # 滤波器工具库 (IIR/EMA/Kalman)
├── models/                       # 神经网络权重文件
│   ├── dnn/                      # DNN 前馈网络 (.pt)
│   │   ├── Trained_model.pt
│   │   ├── Trained_model1.pt
│   │   ├── Trained_model2.pt
│   │   └── Trained_model3.pt
│   └── lstm/                     # LSTM 网络 (.pt)
│       └── end2end/
│           ├── run/
│           ├── walk/
│           ├── walkv2/
│           ├── walkv2_legdecp/
│           └── walkv2_pd/
├── networks/                     # 网络定义 (每个 class 一个文件)
│   ├── __init__.py
│   ├── base_network.py           # Network (nn.Module) 基础前馈网络
│   ├── dnn.py                    # DNN class (18→128→64→2)
│   ├── lstm_network.py           # LSTMNetwork class (4→256→2)
│   ├── lstm_leg_dcp.py           # LSTMNetworkLegDcp class (每腿 2→256→1)
│   └── lstm_pd.py                # LSTMNetworkPD class (每腿 2→256→1, PD控制)
└── output/                       # 运行时 CSV 日志输出
```

### 10.2 用法

```bash
source ~/venvs/pytorch-env/bin/activate
cd RPi_Unified
python RL_controller_torch.py --nn <type> [--tag <label>]
```

| `--nn` 参数 | 网络 | 输入维度 | torque 公式 | 推荐 kp/kd |
|---|---|---|---|---|
| `dnn` (默认) | DNN (18→128→64→2) | 18 (含历史帧) | `((qHr*0.1-qTd)*50 - dqTd_filtered*14.142)*0.008` | kp=50, kd=14.142 |
| `lstm` | LSTMNetwork (4→256→2) | 4 | `0.1*action*kp + dqTd*kd` | kp=50, kd≈3.536 |
| `lstm_leg_dcp` | LSTMNetworkLegDcp (2→256→1 ×2) | 4 (拆2×2) | `0.2*action*kp + dqTd*kd` | kp=50, kd≈3.536 |
| `lstm_pd` | LSTMNetworkPD (2→256→1 ×2) | 4 (拆2×2) | `(action-qTd)*kp - dqTd*kd` | kp=50, kd≈3.536 |

- `--tag`: 自定义日志文件名标签 (默认按 `--nn` 类型命名)
- DNN 模式自动取反 IMU 输入 (flexion 正方向不同)
- LSTM 系列默认 20Hz Butterworth 2阶 torque 前滤波

### 10.3 网络对比

| | DNN | LSTMNetwork | LSTMNetworkLegDcp | LSTMNetworkPD |
|---|---|---|---|---|
| **NN_TYPE_CODE** | 0 | 1 | 2 | 3 |
| **结构** | 18→128→64→2 前馈 | 4→256(×2层)→2 LSTM | 每腿 2→256(×2层)→1 LSTM | 每腿 2→256(×2层)→1 LSTM |
| **滤波器** | 5个接口 (input_pos, input_vel, vel, ref, torque) | 仅 exo_filter (torque前) | 仅 exo_filter (torque前) | 仅 exo_filter (torque前) |
| **IMU符号** | 取反 (`INVERT_IMU_SIGN=True`) | 不取反 | 不取反 | 不取反 |
| **模型路径** | `models/dnn/Trained_model3.pt` | `models/lstm/end2end/...` | `models/lstm/end2end/walkv2_legdecp/` | `models/lstm/end2end/walkv2_pd/` |

### 10.4 运行时滤波器调节 (GUI → RPi)

通过 BLE 透传区 (40B) 从 GUI 发送滤波器配置:
- **DNN**: 可调 Vel/Ref/Torque 三组滤波器的 type、cutoff、order、enable
- **LSTM 系列**: 可调 Torque 前滤波器的 type、cutoff、order、enable

GUI 下行透传协议 (40B payload):
```
[0-1]  magic: 'R' 'L' (0x52 0x4C)
[2]    version: 0x01
[3]    command: 0x01 (APPLY)
[4]    filter_type_code: 1=Butterworth, 2=Bessel, 3=Chebyshev2
[5]    filter_order: 2 (default)
[6]    enable_mask: bit0=vel, bit1=ref, bit2=torque
[7]    reserved
[8..11]   scale (float32 LE)
[12..15]  delay_ms (float32 LE)
[16..19]  cutoff_hz (float32 LE)
```

> **Teensy 固件无需修改**: RL 的滤波器参数和网络选择变更完全在 GUI ↔ RPi 之间通过 BLE 透传区完成，Teensy 侧 (`Controller_RL` / `BleProtocol`) 只做原样转发 (`rpi_passthru[40]`)，不解析这些字节。因此增减 RL 滤波器选项、切换网络类型等改动只需修改 `GUI.py` 和 `RL_controller_torch.py`，无需重新编译烧录 Teensy。

### 10.5 RPi 状态上行 (RPi → GUI)

RPi 每 50 帧 (~0.5s @100Hz) 发送状态帧 (header `AA 56` + 40B):
```
[0-1]  magic: 'R' 'L' (0x52 0x4C)
[2]    version: 0x01
[3]    nn_type: 0=DNN, 1=LSTM, 2=LSTMLegDcp, 3=LSTMNetworkPD
[4]    filter_source: 0=base (初始配置), 1=runtime_override (GUI已更新)
[5]    filter_type_code: 1=Butterworth, 2=Bessel, 3=Chebyshev2
[6]    filter_order
[7]    enable_mask: bit0=vel, bit1=ref, bit2=torque
[8..11]   cutoff_hz (float32 LE)
[12..15]  scale (float32 LE)
[16..19]  delay_ms (float32 LE)
[20..39]  reserved
```

### 10.6 Serial8 通讯协议 (Teensy ↔ RPi)

**Teensy → RPi (下行)**:
- IMU 数据帧: `A5 5A [LEN=32] [TYPE=0x01] [5×float32 + 11B logtag] [CHKSUM]`
- GUI 透传帧: `A5 5A [LEN=41] [TYPE=0x02] [40B payload] [CHKSUM]`

**RPi → Teensy (上行)**:
- Torque 命令: `AA 55` + 6×float32 (tau_L, tau_R, L_p, L_d, R_p, R_d)
- 状态帧: `AA 56` + 40B (Teensy 转存到 `rpi_uplink_buf` → BLE → GUI)
