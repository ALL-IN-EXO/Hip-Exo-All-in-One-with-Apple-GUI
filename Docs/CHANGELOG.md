# Changelog

All notable changes to this project will be documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

## [v6.0] - 2026-05-01

> **Phase 1 release**：三 bug（Apply Algorithm 多点、Power OFF/ON 力矩不恢复、RL 参数下发不可见）全部上机复测通过。详细诊断过程见 `Docs/PHASE1_DEBUG_PROGRESS.md`。

### Fixed

- **RPi cfg 下发可视化（Bug 3 取证 + 验证通过）**（`RPi_Unified/RL_controller_torch.py`）：
  - **背景**：用户报告 GUI 改 RL 参数 → 点 Apply RL Settings → "RPi console 没有任何收到信号的提示"。静态分析发现 RPi 的 cfg 解析路径其实完整正确（magic/version/cmd 全部对得上 GUI 发的字节），但**整条 cfg 应用流程没有任何 `print` 调用**，所以即使 RPi 完美收到、解析、应用，用户也看不到。
  - 加 `DBG_PHASE1_RPI` 主开关（环境变量 `HIPEXO_DBG=0` 可关）。
  - `read_packet`：cfg 帧到达即打印 `[RPi DBG read_packet] CFG #N len=... payload_head=...`；bad_len/bad_cksum/unknown_type 也打印分类原因，分别计数 `_DBG_RPI_BAD_LEN/_BAD_CKSUM/_OTHER_FRAMES`。
  - `parse_runtime_cfg`：accept 时打印 `[RPi DBG parse_cfg] ACCEPT scale=... delay=... cutoff=...`；reject 时按具体失败原因（短 payload、bad magic、bad version/cmd、非 finite floats、cutoff<=0）分别打印，方便定位。
  - 主循环 cfg 分支：解析后打印 `[RPi DBG cfg APPLIED] NN_TYPE=... -> scale=... delay=... cutoff=...`。
  - 主循环 1Hz 心跳 `[RPi DBG hb] in_waiting=N imu=A cfg=B other=C bad_len=... bad_cksum=... parse_reject=... no_hdr_bytes=...`：即使没收到 cfg 也能持续看见 Serial8 RX 链路状况。
  - **Test C 上机复测通过**（2026-05-01）：cfg 包路径全程畅通，只是之前不可见。
- **`main()` 内给模块级 `_DBG_RPI_LAST_HEARTBEAT_TS` 赋值漏 `global` 声明导致启动 `UnboundLocalError`**（`RPi_Unified/RL_controller_torch.py`）：在 `main()` 顶部加 `global _DBG_RPI_LAST_HEARTBEAT_TS`。

- **BLE 下行链路抗丢帧修复（Bug 1 / Bug 2 同根因，Phase 1 上机实测确认）**：
  - **背景**：Phase 1 实测发现，GUI 在 ~1ms 内连发两个 128B BLE 帧时（典型场景：`Apply Algorithm` 触发的 LOGTAG + params 双帧），第二帧在 BLE bridge (nRF52840) → Teensy Serial5 链路上**几乎必丢**。
    - Test 1 失败实测：GUI 发 9 帧 → Teensy 收 5 帧（LOGTAG 4/4，紧跟其后的 algo 0/4）
    - Test 1 成功实测：用户连点 3 次才能切成功，因为前几次的双帧第二帧总是丢
    - Test 2 (Power 混乱)：用户疯狂连点 Power ON/OFF (150-200ms 间隔)，约半数帧被丢，造成 GUI 按钮状态与 Teensy `max_torque_cfg` 长期错位
  - **修复 A** — `GUI_RL_update/GUI.py`：新增 `_throttled_ble_write()` 模块级 helper，强制相邻两次 `self.ser.write()` 间隔 ≥ 50ms。`_tx_params` 和 `_send_logtag_text` 全部改用此 helper。单次操作多 50ms 延迟（用户感知不到），但彻底消灭"双帧第二帧丢失"模式。
  - **修复 B** — `All_in_one_hip_controller_RL_update.ino`：在 `setup()` 里给 Serial5 加 `addMemoryForRead(buf, 512)`（默认 RX 缓冲只有 64B，单帧 128B 直接超出 budget，主循环抖动 >5.5ms 就丢字节）。同 Serial8 (RPi 链路) 的现成做法。即使 GUI 没修，Teensy 也能容忍 ~45ms 主循环抖动。
  - **不动控制逻辑**：`switch_algorithm()`、`_on_power_toggled()`、`_tx_params()` 等控制函数本身没问题，只是它们想发的帧之前没全部到达 Teensy。
  - 详细诊断与原始日志见 `Docs/PHASE1_DEBUG_PROGRESS.md`。

- **Phase 1 调试基础设施**（不影响控制逻辑）：
  - Teensy: `Controller_RL.h` 新增 `DBG_PHASE1` 宏开关；`.ino` + `Controller_RL.cpp` 在算法切换、BLE 每帧、10Hz 状态、RPi 链路边沿处加 `DBG1(...)` 打印 + 帧/字节计数器（`DBG_PHASE1=0` 编译期消去）。
  - GUI: `GUI.py` 加模块级 `DBG_PHASE1` 开关（环境变量 `HIPEXO_DBG=0` 可关）+ `_dbg_log()` helper 自动落盘到 `GUI_RL_update/debug_logs/phase1_<时间戳>.log`。打点：`_on_algo_confirm` / `_on_power_toggled` / `_on_apply_rl_clicked` / `_tx_params -> SERIAL` / 上行 `active_algo` 变化 / 上行 ACK。
  - `.gitignore` 加 `GUI_RL_update/debug_logs/`。

- **Apply Algorithm pending 状态可重试修复**（`GUI_RL_update/GUI.py`）：
  - 当 GUI 选中算法与 Teensy 实际活动算法不一致时，`Apply Algorithm *` 持续可点击（不再被锁灰）
  - 解决“首包未生效后必须改参数/切 Power 才能再次 Apply”的问题
  - `Apply Algorithm` 按钮新增 tooltip，明确提示“切换算法时需要点击两次（request + ACK确认）”

- **TMOTOR 电机遥测 `waiting/0` 修复（请求链路）**（`Motor_Control_Tmotor.*`, `MotorDriver.h`, `All_in_one_hip_controller_RL_update.ino`）：
  - 新增 TMOTOR `request_status()` 主动请求状态帧（`CAN_PACKET_STATUS`）
  - 主循环对 `SIG/TMOTOR` 均执行 `request_feedback()`（左右腿都请求）
  - 遥测有效位判据改为“任一侧有效即上报”，避免单侧 NAN 导致整帧无效
  - `unpack_servo_telemetry()` 的 CAN ID 匹配改为兼容式（低字节/高字节 drive_id 均可识别），提升不同驱动固件 ID 打包差异下的回读成功率

## [v5.1] - 2026-04-30

### Changed

- **BLE 上行字段压缩（回收 12B 预留位）**（`BleProtocol.h`, `All_in_one_hip_controller_RL_update.ino`, `GUI.py`, `SYSTEM_ARCHITECTURE.md`）：
  - 去除 `IMU1..4` 角速度上行字段（原 `data_ble[44..51]`），改为 `reserved[8B]`
  - 将 `imu_ok / sd_ok / tag_ok / brand / algo` 五个状态打包为 `status0`（`data_ble[17]`）
  - 原单独状态字节 `data_ble[20]/[25]/[28]/[31]` 改为 `reserved`
  - GUI 上行解包改为读取 `status0` 位域；L/R 角速度仍保留在 `data_ble[40..43]`，控制主链路字段不变

- **GUI 左侧参数下发改为“显式 Apply”**（`GUI_RL_update/GUI.py`）：
  - 左侧参数控件（含 Teensy 输入/力矩滤波、EG/Samsung/SOGI/Test 参数）不再 `valueChanged/stateChanged` 自动下发；
    改为仅标记为 pending（`Apply Algorithm *` 变橙可点）。
  - `Apply Algorithm` 现在同时处理两类变更：算法切换 + 左侧参数变更；
    若仅参数变更（算法未变），不再重置 RPi 在线状态。
  - `Apply RL Settings` 仍只在 RL 参数变更时可点（Scale/Delay/Cutoff/Filter/Auto Method）。
  - `Power` 仍保留即时下发（安全优先）：`OFF => max_torque=0`，`ON => 恢复非零值/默认15`；
    并在成功发送后刷新“已应用签名”，避免按钮脏状态假阳性。

- **新增电机遥测显示页（Current/RPM）+ CSV 记录**（`GUI_RL_update/GUI.py`, `All_in_one_hip_controller_RL_update/BleProtocol.h`, `All_in_one_hip_controller_RL_update/MotorDriver.h`, `All_in_one_hip_controller_RL_update/All_in_one_hip_controller_RL_update.ino`）：
  - GUI 右侧顶栏新增页面切换按钮：`Main Plots` / `Motor Data`
  - `Motor Data` 页新增两张实时曲线：左右电机电流（A）与左右电机转速（RPM）
  - Teensy 在 BLE 上行预留字节中增加紧凑电机遥测量化字段（`0.5A/LSB`, `50rpm/LSB` + valid flags）
  - GUI 端新增解码与状态显示，并写入 GUI CSV：`motor_cur_L/R_A`, `motor_rpm_L/R`, `motor_*_valid`

## [v5.0] - 2026-04-30


### Fixed

- **RL 状态回显偶发异常 `45.0Hz` / 旧值残留问题修复**（`Controller_RL.cpp`, `All_in_one_hip_controller_RL_update.ino`, `GUI.py`, `RL_controller_torch.py`）：
  - Teensy `AA56` 状态帧新增健壮性校验（`magic/version/source/order/enable_mask/cutoff/scale`），异常帧直接丢弃，不再污染 GUI
  - RL 串口超时 (`>500ms`) 时清空 `rpi_status_valid + rpi_status_buf`，避免 GUI 继续显示过期状态
  - RL 模式下若本周期无有效 RPi 状态，BLE 上行 `rpi_uplink_buf` 置零（不沿用历史缓存）
  - GUI 端新增 RPi 状态元信息 sanity guard，拒收异常 cutoff/scale/order/status 元数据
  - RPi 端 runtime cfg 取消 `30Hz` 硬上限，不再做范围夹紧；仅对非正值/非数值拒绝，避免“上限掩盖链路问题”

- **Teensy 端 Live Power 可能全零导致 GUI `Live --` 的回归修复**（`All_in_one_hip_controller_RL_update/All_in_one_hip_controller_RL_update.ino`）：
  - `phys_pwr` 计算口径统一为“当前下发命令扭矩 × 当前速度”（不再使用 `get_torque_meas()`）
  - Teensy SD 日志中的 `L_pwr_W/R_pwr_W` 同步为同一口径，避免 GUI Live 与本地日志定义不一致
- **Teensy 统一滤波函数调用名修复**（`All_in_one_hip_controller_RL_update/All_in_one_hip_controller_RL_update.ino`）：
  - 将主循环中的 `reset_torque_filter_state()` / `update_torque_filter_if_needed()` 调用改为已存在的 `reset_filter_states()` / `update_filters_if_needed()`
  - 修复函数未声明导致的编译失败

- **`Controller_SOGI.h` 编译错误修复**（`All_in_one_hip_controller_RL_update/Controller_SOGI.h`）：
  - 补回缺失的三个成员变量 `vel_lpf_fc_`、`vel_filt_L_`、`vel_filt_R_`（`.cpp` 中已使用但 `.h` 被回退丢失）
  - `ZcTracker` 结构体补回 `hold_elapsed_s` 字段（`.cpp` 的 `update_zc_tracker()` 依赖此字段）
  - 移除废弃的 `zc_fake_hold_s_` 类成员（新实现已将 hold 时间内置到 `ZcTracker` 中）

- **Teensy 主循环滤波函数名不一致修复**（`All_in_one_hip_controller_RL_update/All_in_one_hip_controller_RL_update.ino`）：
  - 主循环调用统一为已定义函数：`reset_torque_filter_state()`、`update_torque_filter_if_needed()`
  - 消除历史重构后 `reset_filter_states()/update_filters_if_needed()` 命名混用导致的编译风险

- **GUI Power 源语义与方向按钮解耦修复**（`GUI_RL_update/GUI.py`, `Docs/SYSTEM_ARCHITECTURE.md`）：
  - `Power` 下拉改为显式标注：`Physical (Teensy)` / `Control (Pi)`（内部仍用 canonical token `Physical/Control`）
  - `Power:Auto` 逻辑收敛为：仅 `RL + sync_from_pi + ctrl_valid` 选 `Control (Pi)`；其余场景选 `Physical (Teensy)`，否则 `None`
  - `Motor L+/R+` 不再参与 GUI power 的符号补偿；仅影响执行器下发方向，避免污染 `Power Sign` 条带、`Live/Eval` 文本与 GUI CSV 的 `L_pwr_W/R_pwr_W`

### Changed

- **GUI 开机默认曲线显示调整**（`GUI_RL_update/GUI.py`）：
  - 启动时仅默认勾选 `Angle` 与 `Cmd`
  - `Est` / `Vel` / `Pwr` 默认关闭（可手动开启）

- **GUI 参数区紧凑化与 Legacy 交互约束**（`GUI_RL_update/GUI.py`）：
  - `Input Filter` 改为单行布局：`Enable + Type + (fc/alpha)`，节省纵向空间
  - `Torque Filter (Teensy)` 改为单行布局：`Enable + Type + (fc/alpha)`，并统一标题文案
  - 移除参数区 `Plot: Raw Angle / Raw Velocity` 两个显示切换控件（Raw 数据显示保持默认直通）
  - Legacy 路径（EG/Samsung）激活时，`Input Filter` 继续灰显禁用，避免路径语义冲突

- **RPi README 滤波链路与启动方式文档同步**（`RPi_Unified/README.md`）：
  - 明确 `run.sh` 的交互/直启入口（`dnn/lstm_leg_dcp/lstm_pd/pf_imu/myoassist_*`）
  - 明确 RPi 统一 torque 输出滤波默认值：`2阶 IIR Butterworth, 5Hz, ON`
  - 明确 DNN / PF-IMU 输入滤波位置与开关，及可选滤波器类型：Butterworth / Bessel / Chebyshev II

- **SYSTEM_ARCHITECTURE 数据结构补全（RPi cfg/status + power 字段来源）**（`Docs/SYSTEM_ARCHITECTURE.md`）：
  - 更新 `§10.4` GUI→RPi 40B cfg 结构（含 `auto_flags` 与保留区）
  - 更新 `§10.5` RPi→GUI `AA56 v3` 40B 状态结构（每腿 delay/ratio/+P/-P/best_delay）
  - 新增 `§10.5.1`，明确 `AA56`(Eval 指标) 与 `telemetry_ext`(Physical/Control 实时功率) 的职责边界

- **Teensy BLE reassembly 修复**（`All_in_one_hip_controller_RL_update/All_in_one_hip_controller_RL_update.ino`）：
  - `Receive_ble_Data()` 从阻塞式 `Serial5.readBytes()` 改为非阻塞字节累积状态机（`collecting` + `payload_buf`）
  - 修复 128 字节 BLE 帧分多包到达时 `readBytes()` 超时丢弃的问题，彻底解决 GUI 切算法后 badge 常驻橙色的现象

- **SOGI 速度低通滤波 + 自动相位补偿**（`All_in_one_hip_controller_RL_update/Controller_SOGI.*`, `BleProtocol.h`, `GUI_RL_update/GUI.py`）：
  - `compute()` 中新增一阶 IIR 速度 LPF，截止频率 `vel_lpf_fc_` 由 GUI 可调（默认 6 Hz）
  - LPF 引入的相位滞后通过 `atan2(wn, wc)` 精确计算并自动叠加到 `phi_lead`，无需手动补偿
  - BLE 下行新增 `sogi_vel_lpf_fc`（payload `[19..20]`，×10 编码），GUI SOGI 面板新增 `vel LPF (Hz)` 控件

- **SOGI 过零频率门控重构**（`All_in_one_hip_controller_RL_update/Controller_SOGI.*`）：
  - `ZcTracker` 新增 `hold_elapsed_s` 字段，hold 时间内置到 tracker 结构体，不再依赖类级别的 `zc_fake_hold_s_`
  - 过零检测对象改为 LPF 后速度（`velL/velR`），避免对原始速度的高频假过零误触发
  - 默认参数：过零间隔下限 0.12s（~4 Hz 半周期），0.5s 窗口内 ≥2 次触发，关断保持 0.3s

- **SOGI 深蹲/STS 双侧模式自动检测**（`All_in_one_hip_controller_RL_update/Controller_SOGI.*`）：
  - 通过 `cos(φ_L - φ_R) = (x1_L·x1_R + x2_L·x2_R) / (ampL·ampR)` 实时计算双腿相位差
  - IIR 平滑时间常数 0.5s，迟滞阈值 +0.4（→双侧）/ −0.1（→交替）
  - 深蹲/STS 模式下改用升正弦 `(1 + sin φ) / 2` 输出（代替半波整流），覆盖完整周期、两端平滑无硬截断

- **SOGI 速度幅值自适应增益**（`All_in_one_hip_controller_RL_update/Controller_SOGI.cpp`）：
  - 步行模式力矩与速度幅值线性正相关：`scale = amp / SOGI_AMP_REF`（默认 80 deg/s），由 `max_torque_cfg` 最终限幅
  - 深蹲/STS 模式不做幅值自适应，使用固定 `A_gain`

- **SOGI GUI 默认参数更新**（`GUI_RL_update/GUI.py`）：
  - `A_gain` 5→10 Nm，`phi_lead` 20→0°，`move_on` 0.15→0.00s，`move_off` 0.20→0.10s，`vel LPF` 10→6 Hz

- **Teensy 可切换滤波框架演进**（`All_in_one_hip_controller_RL_update/All_in_one_hip_controller_RL_update.ino`, `BleProtocol.h`, `GUI_RL_update/GUI.py`）：
  - Teensy 引入 `SignalFilter` 统一封装，支持一阶 IIR LPF 与二阶 Butterworth
  - GUI 支持运行时下发滤波类型、截止频率/alpha 与使能位
  - 功率计算仍基于原始 IMU 速度（`imu.LTAVx/RTAVx`），不被输入滤波路径改写

- **Teensy 滤波链路重构为“输入滤波 + 力矩前滤波”双级架构**（`All_in_one_hip_controller_RL_update/All_in_one_hip_controller_RL_update.ino`, `All_in_one_hip_controller_RL_update/BleProtocol.h`, `GUI_RL_update/GUI.py`, `Docs/SYSTEM_ARCHITECTURE.md`）：
  - BLE 下行字段更新：`[31..32]=input_filter_fc_hz`、`[34..35]=torque_filter_fc_hz`、`[98]=filter_flags(bit0..bit3)`、`[99..100]=input_filter_alpha`、`[101..102]=torque_filter_alpha`
  - GUI Teensy 过滤控件改为两组独立设置：
    - `Input Filter`（角度/速度，LPF alpha 或 Butterworth fc）
    - `Torque Filter [Teensy-local only]`（电机前最终滤波，LPF alpha 或 Butterworth fc）
  - RL 模式下自动灰显 Teensy 输入滤波与力矩前滤波控件；EG/Samsung legacy 内部 LPF 开关开启时灰显输入滤波控件
  - EG legacy 与 Samsung legacy 对输入滤波生效为旁路；**不再旁路** Teensy 最终 torque prefilter（仅 RL 路径旁路）
  - Samsung 新增 `Legacy Internal LPF` 占位开关（协议 bit3 与 GUI 按钮先打通，算法内部实现待后续）

- **`parse_params` 调试打印**（`All_in_one_hip_controller_RL_update/Controller_SOGI.cpp`）：
  - `Controller_SOGI::parse_params()` 末尾新增串口打印，每次 GUI 下发 SOGI 参数时输出一行 `[SOGI] A=... lead=... ...`，方便验证 BLE 传参是否成功

### Changed

- **RL 默认 delay 与 scale 范围调整**（`RPi_Unified/RL_controller_torch.py`, `GUI_RL_update/GUI.py`）：
  - `pf_imu`、`myoassist_1966080`、`myoassist_2293760` 三个算法的默认 runtime delay 统一为 `0ms`（Pi 默认 + GUI 预设 + GUI 远程启动预填一致）
  - `RL Scale (L/R)` 取值范围从 `0~3` 扩展到 `0~20`（GUI 输入范围 + Pi 端解析限幅一致）
  - Samsung 的 `Kappa` 范围已保持 `0~20`（本次复核确认，无需额外修改）

- **PF-IMU 输入滤波接入 + Teensy-only 滤波 UI 约束**（`RPi_Unified/networks/pf_imu.py`, `RPi_Unified/RL_controller_torch.py`, `GUI_RL_update/GUI.py`）：
  - `PF-IMU` 新增运行时输入滤波链路（输入角度 + 输入速度），复用 RL 面板 `Filter Type / Filter Cutoff / Input Filter` 配置
  - `PF-IMU` 在启动阶段即按默认配置应用输入滤波（默认 5Hz, 2nd-order Butterworth，`Input Filter=ON`）
  - GUI 顶部 `Filter Before Torque (Hz)` 明确标注为 `Teensy-local only`，并在 `RL` 算法激活时自动灰显禁用

- **接入 NJIT MyoAssist 双 checkpoint 到 Pi RL 统一入口**（`RPi_Unified/RL_controller_torch.py`, `RPi_Unified/networks/myoassist.py`, `RPi_Unified/networks/__init__.py`, `RPi_Unified/run.sh`, `GUI_RL_update/GUI.py`, `RPi_Unified/README.md`, `Docs/NJIT_MYOASSIST_DEPLOYMENT.md`, `tools/myoassist_consistency_eval.py`）：
  - 新增 `--nn myoassist_1966080` 与 `--nn myoassist_2293760`，并分配 `NN_TYPE_CODE=5/6`
  - GUI RL 面板新增 `Start Myo-1966` / `Start Myo-2293` 一键远程启动按钮与 `RPi nn_type` 显示映射
  - MyoAssist 运行时默认参数：`scale=1.00`、`delay=0ms`、统一 torque 前滤波继续由主循环管理
  - 新增 `Docs/NJIT_MYOASSIST_DEPLOYMENT.md`，记录 NJIT 源码算法清单、迁移范围、接口差异、部署路径与验收方法
  - 新增一致性评估脚本 `tools/myoassist_consistency_eval.py`；在 `PI5_lstm_pd-20260424-221451.csv` 的 `50–150s` 区间对两个 checkpoint 做逐点对比，结果左右腿均为 `corr=1.0 / RMSE=0 / lag=0ms / amp_ratio=1.0`

- **新增 Pi 侧 `PF-IMU` 控制器接入（`--nn pf_imu`）并保持 RL 协议不变**（`RPi_Unified/networks/pf_imu.py`, `RPi_Unified/RL_controller_torch.py`, `GUI_RL_update/GUI.py`, `RPi_Unified/run.sh`, `RPi_Unified/README.md`, `Docs/PF_IMU_DEPLOYMENT.md`）：
  - Pi 主入口新增 `--nn pf_imu` 分支，沿用现有 RL 收发链路（`AA59` 控制帧 / `AA56` 状态帧格式不变）
  - GUI 新增 `Start PF-IMU` 远程按钮，并支持 `nn_type=4 -> RPi: PF-IMU` 模式识别显示
  - PF-IMU 默认 RL 运行时参数：`scale=1.00`、`delay=0ms`、统一 torque filter 继续有效
  - Pi CSV 新增 PF 运行诊断字段（`pf_compute_ms/p95`, `pf_overrun_count`, `pf_exception_count`, `pf_ess_*`, `pf_conf_*`）
  - 新增部署文档 `Docs/PF_IMU_DEPLOYMENT.md`，说明性能保护策略、接口兼容原因与后续演进路线

- **新增 PF-IMU 一致性评估工具与结果归档**（`tools/pf_imu_consistency_eval.py`, `Docs/PF_IMU_DEPLOYMENT.md`）：
  - 用同一 CSV（`PI5_lstm_pd-20260424-221451.csv`）在 `50–150s` 对比 MATLAB 参考逻辑与 RPi 移植逻辑
  - 评估口径固定为 `delay=0`（仅比较算法裸输出，不走 runtime delay）
  - 输出 `single` 与 `MC-mean` 两类结果，文档记录 `lag/corr/RMSE/amp_ratio`；MC 均值结果两腿相关性约 `0.94`，lag 为 `0ms`

- **PF-IMU 切换 MATLAB-v12 profile 并复验**（`RPi_Unified/networks/pf_imu.py`, `RPi_Unified/RL_controller_torch.py`, `tools/pf_imu_consistency_eval.py`, `Algorithm Reference/Zhemin Reference/pf_imu_matlab_local_compare.py`, `Algorithm Reference/Zhemin Reference/PF_IMU_MATLAB_LOCAL_COMPARISON.md`, `Docs/PF_IMU_DEPLOYMENT.md`）：
  - Pi 默认 `pf_imu` 更新为：`900 particles`、`5-point smoothing`、`auto qstar prior ON`
  - 控制器新增在线 auto-prior 与输入平滑（并暴露 prior 诊断字段）
  - 用同一数据重跑 50–150s 对比后，`matlab vs local_ref_mcmean` 提升到 `corr≈0.838/0.810 (L/R)`；部署链路受因果 5 点平滑影响出现约 `-20ms` 固定相位差（best-lag 相关性更高）
  - 详细图表与结论见 `Algorithm Reference/Zhemin Reference/PF_IMU_MATLAB_LOCAL_COMPARISON.md`

- **PF-IMU 新增 5 秒分窗证据并明确“部署难于仿真”结论**（`Algorithm Reference/Zhemin Reference/pf_imu_windowed_5s_analysis.py`, `Algorithm Reference/Zhemin Reference/pf_imu_compare_results_50_150s/*`, `Algorithm Reference/Zhemin Reference/PF_IMU_MATLAB_LOCAL_COMPARISON.md`, `Docs/PF_IMU_DEPLOYMENT.md`）：
  - 新增 `50–150s` 的 `5s × 20 窗` 分窗分析与网格图（左右腿）
  - 结果显示部署链路在全部窗口中均劣于参考仿真链路（两腿 `20/20` 窗：`corr` 更低、`RMSE` 更高）
  - 两腿窗口级最佳滞后均稳定在 `-20ms`，支持“因果实时链路导致固定相位差”的判断
  - 文档主结论更新为：真实部署受因果、实时预算、随机路径与安全保护链路约束，目标应为统计可控与稳定，而非逐点复刻离线曲线

- **PF-IMU 切换到 MATLAB v17 guided-raw profile，并重跑一致性对比**（`RPi_Unified/networks/pf_imu.py`, `RPi_Unified/RL_controller_torch.py`, `tools/pf_imu_consistency_eval.py`, `Algorithm Reference/Zhemin Reference/PF_IMU_MATLAB_LOCAL_COMPARISON.md`）：
  - Pi `pf_imu` 默认更新为 v17 口径：RAW 输入、固定先验、`900 particles`、robust likelihood、direction-side penalty、guided injection、confidence-gated torque、rate limit
  - `pf_imu_consistency_eval.py` 统一到 v17 并补充 `imu_LTx/imu_RTx/imu_Lvel/imu_Rvel` 到输出 CSV
  - 新增输出：`pf_imu_consistency_50_150s_v17_single.csv`、`pf_imu_consistency_50_150s_v17_mcmean.csv`
  - v17 内部一致性结果（50–150s）：`local_ref_mcmean vs local_deploy_mcmean` 达到 `corr≈0.999`（L/R），`lag=0ms`

- **补充 PF-IMU v17 对比实现说明文档**（`Algorithm Reference/Zhemin Reference/PF_IMU_V17_COMPARISON_IMPLEMENTATION.md`）：
  - 记录完整对比流程：输入数据、脚本、执行命令、输出文件、指标口径
  - 明确给出两层结论：`local ref vs local deploy` 高一致（移植正确），`MATLAB vs local` 仍有系统差异（当前主要是口径/流程差异）

- **版本号来源统一为 `Docs/CHANGELOG.md`（GUI 启动 + macOS 打包）**（`GUI_RL_update/GUI.py`, `scripts/build_mac_JZ.sh`, `README.md`）：
  - GUI 启动时不再硬编码标题版本，改为读取最新发布节（`## [vX.Y[.Z]] - YYYY-MM-DD`）并显示为 `Hip-Exo Controller vX.Y`
  - `build_mac_JZ.sh` 的 `APP_VERSION` 默认值改为从 `Docs/CHANGELOG.md` 解析（仍可用环境变量覆盖）
  - 打包默认软件名改为 `HipExoControllerGUI_v<version>`，`Info.plist` 同步写入 `CFBundleShortVersionString`、`CFBundleDisplayName`、`CFBundleName`
  - 打包时将 `Docs/CHANGELOG.md` 一并打入 `.app`（PyInstaller `--add-data`），保证发布包运行时也可按同口径解析版本
  - README 移除硬编码“Current Version: vX.Y”文案，避免与 changelog 双维护

- **EG Legacy A/B 调试链路 + gait period 上报**（`Controller_EG.*`, `All_in_one_hip_controller_RL_update.ino`, `BleProtocol.h`, `GUI.py`, `Docs/EG_DEBUG.md`）：
  - 新增 `payload[33] = eg_legacy_flags`（EG 专用）：`bit0=legacy delay scaling`、`bit1=gate uses x_prev`、`bit2=internal LPF`
  - GUI EG 面板新增 `Legacy EG Path` 开关，开启后一次性下发 `0x07`，用于复刻旧链路做 A/B 验证
  - EG 控制器新增 legacy 行为：
    - delay 缩放改为“仅 `gait_freq>0.7Hz` 才缩短 + 最小 5 samples”
    - gate 输入改为 `x_prev` 路径
    - 恢复 EG 内部 LPF（`0.85/0.15`）
  - 当 EG internal LPF 打开时，Teensy 主循环统一 pre-motor LPF 对 EG 自动旁路，避免双滤波
  - Teensy SD CSV 新增 `gait_freq_Hz`、`gait_period_ms`
  - GUI CSV 新增 `gait_period_ms`（由上行 `gait_freq_Hz` 同公式换算），状态栏同步显示 `gait=...Hz (T=...ms)`

- **README 补充“新增 Pi 算法接入 GUI”标准清单**（`README.md`）：
  - 新增 Pi→GUI 接入步骤：`--nn` 注册、`nn_type` 显示映射、`Start <AlgoName>` 远程按钮、enable/disable 统一纳管
  - 明确“无显式延迟算法”的 RL 运行时默认：`scale=1.00`、`delay=0ms`、`Butterworth 5Hz order=2 Torque ON`
  - 补充快捷启动要求：`run.sh` 支持 `<new_type>` + GUI 一键远程启动

## [v4.1] - 2026-04-27

### Fixed

- 详细问题说明与证据链见：`Code Debug/README_RL_LATENCY_AND_SYNC_SPIKE.md`

- **Teensy-native 场景右腿 Live power 符号反向修复**（`GUI_RL_update/GUI.py`）：
  - 根因：GUI 对 `Cmd/Est torque` 做了 `dir_bits`（L+/R+）符号补偿，但 `power_override` 写入 `L_pwr/R_pwr` 时未做同补偿
  - 现象：当右腿方向配置为 `R-` 时，`Power Sign` 条带、`L/R pwr` 文本与 `Live ratio` 右腿口径出现反号
  - 修复：在 `_append_data_point(...)` 中对 `L_pwr/R_pwr` 同步应用 `mL/mR`（仅 `dir_bits`，不受 `VL+/VR+` 视觉开关影响）

- **RPi 下 GUI 角度/速度/力矩/功率“卡顿十几秒”修复**（`RPi_Unified/RL_controller_torch.py`）：
  - 症状：`Data Source=Auto` 或 `Raw` 下，开启 Pi 后 GUI 数据实时性严重丧失（约 10 秒延迟），即使快速晃动 IMU，Pi 端 CSV 的 `imu_LTx/Lvel` 也几乎不变，说明 Pi 读到的是串口 FIFO 里的旧帧
  - 根因：Teensy 以 ~1kHz 发送 IMU 帧，Pi 主循环名义 100Hz 消费；v4.0 后 Pi 每周期新增的开销（AA59 40B payload、统一 torque 滤波、`lstm_pd` zero-mean `np.mean`、sync buffers、更宽 CSV row、100Hz `print`）一旦令消费略慢于生产，Linux 串口 RX FIFO 即永久堆积旧帧，且原先没有任何排空机制
  - 修复 1：新增 `read_freshest_packet(ser)`，当 `ser.in_waiting` 中仍有完整 IMU 帧时，丢弃旧 IMU 帧只保留最新一帧（`cfg` 帧永远按序透传，GUI 运行时配置不丢）；主循环由 `read_packet(ser)` 切换到 `read_freshest_packet(ser)`
  - 修复 2：移除 `send_torque()` 和 `send_status()` 末尾的 `ser.flush()`。在 115200 波特率下 `ser.flush()=tcdrain()` 每次阻塞 ~3.6ms（42B 帧），每周期直接吃掉主循环 ~36% 时间预算，把消费端从“足够快”推到“追不上”；内核 FIFO + Teensy 端无背压，不需要 drain
  - 效果：Pi 端恢复对最新 IMU 的实时响应，GUI 中 Pi 同步路径（Auto/Sync/Control 功率）不再滞后；IMU 生产速率仍是 Teensy 侧（`~1kHz`），Pi 控制周期仍是 100Hz，均未改动

- **GUI `POWER OFF` 按钮偶发需要点两次才关机修复**（`GUI_RL_update/GUI.py`）：
  - 症状：某些情况下点一次 `POWER OFF` 按钮只把按钮文字/颜色翻了一下但力矩没归零；再点第二次才真的把 `Max Torque` 拉到 0
  - 根因：`btn_power.isChecked()` 与 `sb_max_torque_cfg.value() > 0` 是隐含不变量，但只有 `_on_power_toggled` 维护它；一旦用户在 Parameters 面板手动改 Max Torque（敲数字或点 spinbox 箭头），`sb_max_torque_cfg` 变了但按钮 `checked` 没变 → 下一次点击按钮，Qt 先把 `checked` 从 unchecked 翻到 checked → 进入 `_on_power_toggled(True)`，`setValue(_maxT_before_off)` 与当前值相同是 no-op，只把按钮刷成绿 "POWER ON"；用户得再点一次才触发 `_on_power_toggled(False)` 真把力矩设为 0
  - 修复：给 `sb_max_torque_cfg.valueChanged` 新增 `_sync_power_btn_from_torque`，任何路径改 torque 值都会自动对齐按钮 checked/text/颜色（`>0 → POWER ON 绿`，`=0 → POWER OFF 红`）；同步过程用 `btn_power.blockSignals(True)` 保护，避免重新进入 `_on_power_toggled` 造成回环
  - 效果：`POWER OFF` 永远一次点击到位；直接编辑 Max Torque 数字/箭头后按钮即时跟随；不动 BLE、不动 `_on_power_toggled`、不动掉线自动关机、不动主题刷新

- **GUI `Data Source=Sync` 下偶发 ±300° / ±3000 dps “爆帧”修复**（`All_in_one_hip_controller_RL_update/Controller_RL.h`, `Controller_RL.cpp`）：
  - 症状：Pi backlog 修好后出现的二次问题 —— Sync 路径（GUI Auto/Sync 数据源）约 `2.1%` 帧出现角度 ±300°、速度 ±3000 dps 的刺尖；Raw 路径干净
  - 诊断：对 `GUI_RL_update/data/20260422_213207.csv` 与 `Data from PI/PI5_lstm_pd-20260423-023206.csv` 做同 `sample_id` 交叉比对，发现 Pi 送上线前的 `sync_LTx/sync_Lvel` 始终在物理量程内；GUI 收到的却是 int16 / 100 和 int16 / 10 的近饱和值（`±32500/100=±325°`、`±32767/10=±3275 dps`）。`sync_sample_id`（AA59 payload 前 2 字节）以及 `sync_cmd_L`（走 Teensy 本地 `ud.L_cmd100`，不经 AA59 覆盖）都始终干净 → 错位只发生在 AA59 payload 偏移 `26..37` 的 6 个 int16 字段
  - 根因：Teensy `Serial8` 默认 RX 缓冲仅 64 字节；当主循环被 BLE/SD/算法拖 ≳6 ms 时，Pi 100Hz 连续送达的 42B AA59 帧发生 RX overrun 丢字节。AA59 无校验和 → 状态机照常凑满 42 字节、尾部解到下一帧的头几个字节，int16 字段解成近饱和随机值
  - 为什么以前没暴露：Pi 读旧 backlog 时实际 AA59 发送速率被消费速率压低，Teensy RX 压力小、极少 overrun；修好 backlog 后 100Hz 稳态，overrun 概率上升
  - 修复 1：`Controller_RL::init_serial()` 增加 `Serial8.addMemoryForRead(PI_SERIAL_RX_EXTRA, 512)`，RX 缓冲从 64B 扩到 >512B，可容忍 ~45ms 的主循环停顿（`512B / 11520 B/s`），实测足够覆盖所有已知抖动源
  - 修复 2：`READING_SYNC` 改为先解到临时变量，通过物理量健壮性检查（`|ang|≤20000`、`|vel|≤25000`、`|pwr|≤30000`）后才提交到成员；坏帧直接丢弃、沿用上一帧 sync 值。新增 `Controller_RL::bad_sync_frames` 计数供观察 RX 抖动频率
  - 效果：两条改动叠加，overrun 几乎不发生、即便发生坏帧也不会传到 BLE → GUI，彻底消除 Sync 路径刺尖。未改动 AA59 协议（未加 checksum，留待后续观察 `bad_sync_frames` 是否仍增长再决定是否协议升级）

### Changed

- **SOGI 新增 STOPPED/MOVING 运动状态机（`All_in_one_hip_controller_RL_update/Controller_SOGI.*`）**：
  - 在 SOGI 控制链路加入 `STOPPED`/`MOVING` 状态机，使用幅值迟滞和持续时间判据切换状态
  - 新增阈值与时间参数：`AMP_ON=22`、`AMP_OFF=14`、`MOVE_ON=0.15s`、`MOVE_OFF=0.30s`、`STOP_HOLD=0.40s`
  - 最终力矩输出新增 `gate_motion`（仅 `MOVING` 状态允许输出），用于抑制停稳阶段的误助力触发

- **SOGI 状态机参数接入 GUI 实时调节（`BleProtocol.h`, `Controller_SOGI.*`, `GUI_RL_update/GUI.py`）**：
  - SOGI 下行参数扩展新增 `amp_on`、`amp_off`、`move_on_sec`、`move_off_sec`（payload `[11..18]`）
  - GUI 的 SOGI 面板新增 4 个参数控件，并沿用 `valueChanged -> _tx_params()` 机制实时下发到 Teensy
  - Teensy `Controller_SOGI::parse_params()` 改为优先使用 GUI 下发值，旧 GUI 未发送扩展字段时自动回退默认值（兼容旧版本）

- **SOGI 新增“过零频率判假”防抖（`All_in_one_hip_controller_RL_update/Controller_SOGI.*`）**：
  - 使用 `SOGI x1` 检测左右腿过零，按相邻过零间隔估算频率 `f_cross = 0.5 / dt_cross`
  - 当 `f_cross` 超过上限（默认 `3.0 Hz`）时判定为虚假过零，并在短窗内累计计数
  - 若短窗内连续触发异常（默认 `0.30s` 内 `>=2` 次），进入 `0.35s` 输出关断（`gate_zc=0`），抑制过零风暴导致的高频扭矩翻转

- **SOGI 新增站立角度门控（`All_in_one_hip_controller_RL_update/Controller_SOGI.*`）**：
  - 增加硬编码站立判定：当左右髋角度同时落在小角度窗内并持续一段时间后，强制左右力矩置 `0`
  - 目的为抑制站立阶段低频速度噪声导致的虚假助力输出，同时避免步态过零瞬间的短时误触发
- **RL+Pi 场景功率口径与 Overlay 文本统一**（`RPi_Unified/RL_controller_torch.py`, `GUI_RL_update/GUI.py`）：
  - Pi `ctrl_pwr` 口径更新为当前功率：`P_live(t) = tau_out(t) * vel_current(t)`（替代旧 `*sync_vel` 口径）
  - `Power Sign` 条带按 Live 实时流显示；右下角文本同时显示 `Live` 与 `Eval` 两种口径
  - 详细定义（公式、三套 vel 含义、更新频率、为何两者可不同）迁移至 `Docs/SYSTEM_ARCHITECTURE.md` §11.4

- **RPi CSV 补齐 auto-delay 功率评估字段**（`RPi_Unified/RL_controller_torch.py`）：
  - 新增每帧写出：`auto_ratio_L/R`、`auto_pos_per_s_L/R`、`auto_neg_per_s_L/R`
  - 同步新增：`auto_motion_valid_L/R`、`auto_best_delay_ms_L/R`、`auto_window_s`、`auto_gait_freq_hz`、`auto_delay_enable`
  - 目的：离线分析可直接复现 PPR / 正负功评估，不再依赖 GUI 透传状态或二次重算

- **Visual 极性开关仅作用于 Torque 显示**（`GUI_RL_update/GUI.py`）：
  - `VL+/VR+` 现在只翻转 `Cmd/Est torque` 曲线符号
  - 角度、速度、功率曲线不再受 `Visual` 开关影响
  - 更新按钮 tooltip，明确“torque sign only”

- **恢复“新树莓派首次配置与注册”文档**（`Docs/RPI_NEW_PI_SETUP.md`）：
  - 补回 Pi4B / Pi5 串口差异、SSH优先、macOS 免密别名、同步部署、GUI 注册、常见故障排查
  - 包含已验证问题：`remote_dir missing`（`~/` 展开）、`ModuleNotFoundError: scipy`、host key 变更告警

- **Power Sign overlay 在 SOGI/Samsung 下的显示修复**（`All_in_one_hip_controller_RL_update/All_in_one_hip_controller_RL_update.ino`, `GUI_RL_update/GUI.py`）：
  - Teensy `Transmit_ble_Data()` 补齐 `ALGO_SOGI` 的 `fill_ble_status()` 上报，SOGI 不再写零状态槽
  - GUI 算法切换时清空上一算法状态缓存，避免出现旧状态粘连导致的 `WAIT`/0 值
  - GUI `Power: Auto` 逻辑调整为优先 `Control` 功率源（控制同步样本），减少 `Power Sign` 条带在非 RL 算法下“全红/近零”的误显示

- **左侧参数面板滚动修正（`GUI_RL_update/GUI.py`）**：
  - 修复 `QScrollArea` 在部分平台出现左侧内容空白/不可见的问题（恢复 `setWidgetResizable(True)`）
  - 保留纵向与横向滚动条 `AsNeeded`，满足窄屏/高缩放场景可平移查看

- **GUI 远程启动 RL 时的 Delay 默认显示对齐**（`GUI_RL_update/GUI.py`）：
  - 点击 `Start LSTM-PD` 时，RL 面板 `Torque Delay (ms)` 预填为 `200ms`
  - 点击 `Start LegDcp` 时，RL 面板 `Torque Delay (ms)` 预填为 `100ms`
  - 作用是等待首帧 RPi 状态回传期间，GUI 显示不再与 Pi 运行时默认值错位

- **架构文档补充滤波器位置（`Docs/SYSTEM_ARCHITECTURE.md`）**：
  - `5.4 主循环流程` 明确新增“compute 后、下发前”的统一电机前滤波步骤
  - `11.2` 长条数据流图补画 Teensy-native 的统一 LPF 节点
  - 明确 RL 回传扭矩路径旁路 Teensy 该 LPF，保持 Pi 扭矩透明下发

- **RPi LSTM-PD Grid AutoDelay 起点修正**（`RPi_Unified/RL_controller_torch.py`）：
  - `lstm_pd` 运行时默认 delay 基线改为 `200ms`（此前会从 `0ms` 起）
  - AutoDelay 从 OFF 切到 ON 且方法为 `grid` 时，若当前 baseline 仍接近 `0ms`，自动回填到 `200ms` 后再开始扫描

- **RPi 环境文档补全（`RPi_Unified/README.md`）**：
  - 新增“按当前项目配置”的完整环境准备章节（系统依赖、UART 开启、Python venv、必需 pip 包、快速自检）
  - 明确 `tmux` 为 GUI 远程启动 Pi RL 的必需依赖
  - 明确默认虚拟环境路径 `~/venvs/pytorch-env` 与 `run.sh`/GUI 远程控制一致

- **Teensy 非RL算法统一“电机前滤波”并开放 cutoff 调节**（`All_in_one_hip_controller_RL_update/*`, `GUI_RL_update/GUI.py`）：
  - 在 Teensy 主循环新增统一 2阶 Butterworth IIR（100Hz）扭矩滤波，位置固定在 `active_ctrl->compute(...)` 之后、下发电机之前
  - RL 路径保持透明：`Controller_RL` 输出不经过 Teensy 该滤波器
  - EG 内部原 `tau_cmd_*_filt_` 独立扭矩低通移除，避免与统一电机前滤波重复
  - GUI 在 `Max Torque` 下新增 `Filter Before Torque (Hz)`（默认 5.0Hz，可调），通过 BLE 下发到 Teensy payload `[31..32]`
  - Teensy 下行解析新增 `torque_filter_fc_hz` 字段（旧 GUI 未下发该字段时默认回退 5.0Hz）

- **系统架构文档补充“算法×Pi在线状态”运行时数据流总览**（`Docs/SYSTEM_ARCHITECTURE.md`）：
  - 新增按算法（EG/Samsung/SOGI/Test/RL）和 Pi 在线状态划分的统一矩阵
  - 明确每个场景下的扭矩计算归属、auto-delay 归属、滤波器位置、GUI Auto 显示来源、overlay 指标来源
  - 新增两条主链路 Mermaid 长条图与 GUI/RPi/Teensy 三端 CSV 对齐图

- **非 Pi 模式功率 overlay 兜底修复**（`GUI_RL_update/GUI.py`）：
  - `Right/Left Leg Power Sign` 右下角 `+Ratio / +P / -P` 叠字不再仅依赖 RPi 状态帧
  - 当无 Pi 状态（`_rpi_status_valid = False`）时，改为从当前 Teensy 功率条带窗口实时计算并显示指标（ratio、+P、-P）
  - 有 Pi 状态时保持原行为：继续优先使用 Pi/状态帧上报值

- **Hardware 品牌交互可视化增强**（`GUI_RL_update/GUI.py`）：
  - Motor 品牌下拉框改为高对比样式（粗边框 + 粗体），当 pending 与 current 不一致时高亮为橙色，便于一眼识别“待应用”状态
  - `Apply` 按钮在目标品牌已生效时自动切为绿色并显示 `SIG Active` / `TMOTOR Active`
  - 右侧品牌徽标改为 `Current:SIG/TMOTOR`，当前生效品牌使用绿色显示
- **Motor Init 行为优化（品牌一致性）**（`GUI_RL_update/GUI.py`）：
  - 当用户点击 `Motor Init` 且 pending 品牌与 current 品牌不一致时，GUI 先下发品牌切换，再自动触发一次 `Motor Init`
  - `Motor Init` / 品牌选择 / Apply 提示文案与 tooltip 同步更新，减少“按钮显示与当前品牌不一致”的误解
- **默认电机品牌切换为 TMOTOR**（`GUI_RL_update/GUI.py`, `All_in_one_hip_controller_RL_update.ino`）：
  - GUI 品牌下拉默认项改为 `TMOTOR`（初始 pending=TMOTOR）
  - Teensy 上电默认品牌改为 `TMOTOR`（`motor_L/motor_R/current_brand` 初值同步）

## [v4.0] - 2026-04-21

### Engineering Rules (铁规则 — 长期有效，不随版本失效)

> **2026-04-21 确立：关于功率 / 正功比例 / auto-delay 的数据来源约定**
>
> 背景：实测发现 GUI CSV (`20260421_131729.csv`) 和 RPi CSV (`PI5_lstm_pd-*.csv`) 在同一次实验下，正功比例分别为 76 % / 83 %，差异源于 GUI 帧内 `(τ, ω)` 之间存在 **~60–80 ms 的年龄差**（Teensy 20 Hz BLE 打包窗口 + BLE radio；见 `Code Debug/BUG_ANALYSIS.md`）。
>
> 约定（所有新老代码必须遵守）：
>
> 1. **功率的定义永远是 `P = τ(t) · ω(t)`，且 τ 和 ω 必须来自同一物理瞬间。** 任何跨时间戳的乘法（例如 BLE 帧里的 `L_cmd_Nm × L_vel_dps`）都不是功率，不能作为物理量使用、不能用来驱动控制决策、不能作为"正功比例"发表或绘图的 ground truth。
> 2. **RPi 侧是唯一权威来源**：`RPi_Unified/RL_controller_torch.py` 同一控制周期内的 `(imu_*vel, *_command_actuator)` 是因果配对 (vel 是 NN 输入，cmd 是 NN 输出)，其计算出的 `P`、正功比例、auto-delay 搜索都是正确的。日后若新增"真功率"指标，**一律在 RPi 侧算**，经由 Serial8 status 帧或 echo 路径发给 Teensy / GUI。
> 3. **GUI 侧 (含 replay) 只做监视 / 示意**：`GUI.py` 里 `_apply_replay_sample` / `_update_power_strip` / power overlay 文本 **只允许用来显示波形和提示用户**。禁止把它们的计算结果反馈给控制决策 (如 auto-delay)、禁止作为实验分析的数据源、禁止在论文 / 报告里引用 GUI CSV 算出的 ratio。GUI replay 的功率条带 "仅供玩耍"。
> 4. **Teensy 侧也不得基于 BLE-uplink-level `(τ, ω)` 派生任何物理量**。BLE 上行帧字段是打包后的快照流，不是同步的 `(τ, ω)` 对。
> 5. **新增或修改任何功率 / 相关 / 相位 / ratio 计算**时，PR 描述里必须说明所用 `(τ, ω)` 来自哪一对同步源，并在 `Code Debug/BUG_ANALYSIS.md` 或等价文档补一行说明。违反规则 1–4 的代码不得合入 `main`。
> 6. **auto-delay 必须在 RPi 侧运行**，因为只有 RPi 持有真同步的 `(τ, ω)`。GUI 透传层不得自己估 delay。
>
> 相关根因与延迟账见 `Code Debug/BUG_ANALYSIS.md` §3 / §4；本规则在 Option A (RPi 把用过的 `angle/vel` echo 回 Teensy，RL 模式下 Teensy 用 echo 值打 BLE) 落地后，可将 GUI RL 模式也纳入"权威来源"范围，但规则本身（τ 与 ω 必须同瞬间）永远有效。

### Added

- **RL 同步真值链路（Pi→Teensy `AA 59` + BLE 扩展区）**：
  - RPi 控制帧由 legacy `AA 55` 升级为同步帧 `AA 59`（40B payload），同包下发：
    - `sample_id`
    - `tauL/tauR/Lp/Ld/Rp/Rd`
    - 控制对齐输入 `sync_ang/sync_vel`
    - 控制功率 `ctrl_pwr`
  - Teensy `Controller_RL` 新增 `0x59` 解析与同步缓存，保留 `0x55`/`0x56` 兼容解析
  - BLE 上行新增同步扩展区 `data_ble[101..127]`（payload `[98..124]`）：
    - `phys_pwr`（执行器物理功率）
    - `sync_ang/sync_vel/sync_cmd`（控制对齐流）
    - `ctrl_pwr`（控制功率）
    - `sync_from_pi` 等 flags
  - RL+Pi 时同步流优先来自 Pi；无 Pi 同步时 Teensy 自动回退本地同采样 sync 流

- **三端日志统一 Teensy 时间轴（便于 GUI/RPi/SD 对齐）**：
  - Teensy→RPi IMU 二进制帧升级为 v2：新增 `t_cs(uint16)`（保持 v1 兼容解析）
  - RPi CSV 新增 `teensy_t_cs_u16`、`teensy_t_s_unwrapped`
  - GUI CSV 新增 `teensy_t_cs_u16`、`teensy_t_s_unwrapped`（与既有 `t_unwrapped_s` 并存）
  - Teensy SD CSV 新增统一时间与动力学字段：`teensy_t_cs_u16`、`teensy_t_s_unwrapped`、`imu_L/Rvel`、`L/R_pwr_W`
  - GUI Replay 扩展为自动识别 RPi/Teensy/GUI 三类 CSV 常见字段映射；速度/功率列缺失时自动回退 `0`

- **GUI 一键远程启动/停止 Pi RL（本地 SSH，不经 Teensy）**（`GUI_RL_update/GUI.py`）：
  - RL 面板新增 `Start LegDcp` / `Start LSTM-PD` / `Stop Pi RL` 按钮
  - 新增 `Pi Profile` 下拉（active profile），支持在 GUI 内切换当前远端目标
  - 新增 `Configure Pi...` 对话框：可创建/编辑/删除 SSH profile，并设置 active profile
  - Pi profile 配置迁移到用户配置目录（不再依赖仓库内文件）：
    - macOS: `~/Library/Application Support/HipExoController/rpi_profiles.conf`
    - Windows: `%APPDATA%/HipExoController/rpi_profiles.conf`
    - Linux: `~/.config/HipExoController/rpi_profiles.conf`
  - 启动 Pi RL 时若未配置 profile，GUI 自动弹出配置引导对话框
  - 远端 RL 进程采用 `tmux` 会话托管（默认会话名 `hip_rl`），避免终端关闭导致进程退出
  - 新增 `Pi RL Remote` 状态行：显示启动/停止结果、运行状态（含 NN 类型）与常见错误（如 `sshpass/tmux/venv` 缺失）
  - 新增 1Hz 远端状态轮询（仅 RL 场景或远端运行时启用），通过 `tmux capture-pane` 获取运行日志状态
- **6 路 IMU 电量正式上报链路**（`im948_CMD.*` / `IMU_Adapter.*` / `BleProtocol.h` / `GUI_RL_update/GUI.py`）：
  - Teensy 侧新增 6 路 IMU 电量百分比状态变量（L/R/1/2/3/4），由 IM948 `0x10` 状态回复解析填充
  - `IMU_Adapter` 新增低频 round-robin 状态查询（每次查询一路，约每 3s 更新每个 IMU 电量）
  - BLE 上行 payload 新增 `data_ble[52..57]`（对应 GUI payload `[49..54]`）承载 6 路 IMU 电量
  - GUI Hardware 卡新增 `Battery` 行，实时显示 6 路电量百分比并按阈值着色（绿/橙/红）
  - 电量未知时显示 `--`（协议值 `255`）
- **RPi 文档补充 tmux 依赖说明**（`RPi_Unified/README.md`）：
  - 新增 GUI 远程启动模式的 Pi 端依赖提示与安装命令（`apt install tmux`）
- **GUI↔RPi 对齐锚点（CSV 标签化）**（`GUI_RL_update/GUI.py`）：
  - GUI 采集 CSV 新增对齐列：`t_unwrapped_s`、`wall_time_s`、`csv_sample_idx`、`align_event` 及 RPi 状态快照（nn/delay/ratio/+P/-P）
  - 新增自动事件标注（如 `algo_apply`、`rl_apply`、`pi_rl_started/stopped`、`serial_connected/lost`）
  - 新增手动 `Align Mark` 按钮：可人工插入对齐事件（`manual_align:xxx`），并同步短 tag 到 Pi
  - 关键事件可同步发送短 `logtag` 到 Pi（例如 `RLAPPLY`、`RLONPD`、`RLOFF`），用于与 Pi 侧分段日志做时间对齐
  - 备注：`t_s` 基于 Teensy 上行 `uint16` 厘秒计数，存在约 `655.36s`（约 10 分 55 秒）回绕；长时段分析请优先使用 `t_unwrapped_s`

- **SOGI-FLL 相位同步控制器**（`All_in_one_hip_controller_RL_update/`, `GUI_RL_update/GUI.py`）：
  - Teensy 侧新增 `Controller_SOGI.h/.cpp`，基于 SOGI-FLL 提取左右髋角速度瞬时相位 φ，输出 `τ = A·ramp·gate·sin(φ+lead)` 与 ω 同相助力（算法原理见 `Docs/SOGI-FLL-CONTROLLER.md`）
  - `BleProtocol.h` 新增 `ALGO_SOGI = 4` 及 3 个可调参数：`sogi_A_gain` (×100, Nm)、`sogi_phi_lead_deg` (×100, °)、`sogi_amp_min` (×10, deg/s)，占用下行 payload `[5..10]`
  - 其他 SOGI 参数硬编码（f0=1.0 Hz, k=1.41, γ=40, f_min/f_max=0.3/3.5 Hz, ramp=1.5 s），减少调参负担
  - `.ino` 主循环接入：静态实例 `ctrl_sogi`、`switch_algorithm` 新增 `ALGO_SOGI` 分支；复用现有 IMU 安全门与 dir_bits 镜像逻辑
  - GUI 侧 `SegmentedControl` 新增 "SOGI" 选项（位于 Test 之前）；参数面板含 A_gain / Phi lead / amp_min 三个 QDoubleSpinBox，`_tx_params` 按协议写入 payload `[5..10]`
  - SOGI 复用 `AutoDelayOptimizer`（`enabled` 恒 false，不改 delay）以计算 ratio / pos_per_s / neg_per_s，填入 BLE 上行 40B 透传槽，使 GUI 的 `+Ratio` 正功占比指标对 SOGI 生效
- **PyInstaller 打包脚本**（`scripts/build_win.ps1`, `scripts/build_mac.sh`）:
  - Windows PowerShell 脚本产出 `.exe` 目录 + `.zip`，保存到 `GUI_RL_update/release/windows/<时间戳_gitsha>/`
  - macOS Bash 脚本产出 `.app` bundle + `.zip`（用 `ditto` 保留 bundle symlink），保存到 `GUI_RL_update/release/macos/<时间戳_gitsha>/`
  - 两份脚本默认入口 `GUI_RL_update/GUI.py`，可通过 `-Entry/-AppName/-SourceDir`（Win）或 `--entry/--name/--source`（Mac）覆盖
  - 依赖自动装入独立 venv（`.venv-build-win/`, `.venv-build-mac/`）：`pyqt5 pyqtgraph pyserial numpy pyinstaller`
  - 提供 `FullBuild`/`--full` 模式（`--collect-all PyQt5 pyqtgraph`）和默认 SLIM 模式（针对 pyqtgraph 保留 QtSvg/QtPrintSupport/QtOpenGL 等常用依赖，仅排除 Qt3D/QtWebEngine/QtQml 等明显无关模块）
- `.gitignore` 新增 PyInstaller 产物忽略规则：`.venv-build-*/`、`release/`、`*.spec`

### Changed

- **GUI 功率来源改为“仅控制端上报”，移除本地 `vel*cmd` 真值路径**（`GUI_RL_update/GUI.py`）：
  - `L_pwr/R_pwr` 不再由 GUI 本地推导；live 显示仅使用 Teensy/Pi 上报功率
  - Plot 控制区新增 `Data Source`（`Auto/Raw/Sync`）与 `Power Source`（`Auto/Physical/Control`）
  - `Auto` 规则：
    - RL+Pi 同步有效：优先 `Sync + Control`
    - 其他场景：回退 `Raw + Physical`
  - GUI CSV 新增 raw/sync/physical/control 全链路字段与 source 标记，便于离线对齐分析
  - Replay 模式若 CSV 缺失功率列，显示回退为 `0`（不再本地 `vel*cmd` 估算）

- **RL torque 滤波通路统一到主循环**（`RPi_Unified/RL_controller_torch.py`）：
  - 统一执行链改为：`raw torque -> unified torque filter -> delay -> scale -> send_torque`
  - LSTM / LSTM-LegDcp / LSTM-PD 网络内部 `exo_filter` 旁路为 identity（不再各自预滤波）
  - DNN 内部 `torque_filter` 关闭，避免与主循环滤波重复
  - GUI 下发的 `Filter Type / Cutoff / Order / Torque Enable` 统一重建主循环 torque filter（所有 NN 共用）
  - 默认统一 torque filter 设为 `Butterworth 5.0Hz 2nd`（上电即生效）

- **RL Filter Cutoff GUI 初始值统一为 5.0 Hz**（`GUI_RL_update/GUI.py`）：
  - RL 面板 `Filter Cutoff (Hz)` 默认值保持 `5.0`
  - 不同 NN 预设切换时的 cutoff 也统一为 `5.0 Hz`（DNN / LSTM / LegDcp / LSTM-PD）

- **Pi 远程启动配置链路 tooltip 补齐**（`GUI_RL_update/GUI.py`）：
  - RL 面板 `Pi Profile / Pi RL Remote / 状态行` 增加悬停引导
  - `Configure Pi...` 对话框中 profile 列表、输入字段、标签、操作按钮全部补充 tooltip
  - 统一“鼠标悬停即可读到用途/约束”的交互，降低首次配置误操作

- **GUI tooltip 风格统一（全局单一样式）**（`GUI_RL_update/GUI.py`）：
  - 全局 `QToolTip` 统一为深色底 + 浅色字（与 `Disp/Filtered` note 风格一致）
  - 主题切换时同步将样式应用到 `QApplication`，并强制 `QToolTip` palette/font，避免出现黑底黑字或白底黑字混用

- **Hip GUI 连接栏交互调整**（`GUI_RL_update/GUI.py`）：
  - 左上角 `Eco` 开关替换为手动 `Refresh` 按钮，用于即时刷新串口列表
  - 渲染刷新率改为固定使用 Normal 帧率（不再由 Eco 开关切换）
- **Hip GUI 串口自动连接（Adafruit 优先）**（`GUI_RL_update/GUI.py`）：
  - 串口扫描由“仅设备名”升级为带 `VID/PID/Manufacturer/Description` 元信息缓存
  - 断开状态下每秒刷新端口列表，并按评分自动尝试连接（默认 2s 冷却）
  - 自动连接优先规则：`Adafruit VID(0x239A)` / `manufacturer|description` 含 `adafruit` / 最近成功端口
  - 对 `Bluetooth` 类串口降权，减少误连概率；自动连接失败只更新状态栏，不弹窗打断
- **Hip GUI 录屏稳定性改进**（`GUI_RL_update/GUI.py`）：
  - 录屏帧从 PNG 改为 JPG（默认质量 90），显著降低实时落盘开销
  - 录屏状态文本限频更新（约 5 Hz），减少高频 UI 文本重绘
  - 录屏写盘迁移为后台线程 + 有界队列（主线程仅抓帧入队），降低录制过程主线程阻塞导致的卡顿
  - 队列拥塞时启用丢最旧帧策略，优先保证实时交互与当前画面
  - ffmpeg 查找逻辑新增“打包内 bin + 环境变量 + 系统路径”多级解析
  - ffmpeg 编码参数改为 `libx264 + ultrafast + even-dimension padding`，降低编码耗时并避免奇数分辨率失败
- **Hip GUI Replay 交互增强（进度条 + End-Hold）**（`GUI_RL_update/GUI.py`）：
  - Plot 控制区新增 replay 时间进度条（当前时刻/总时长 + 样本索引），支持拖动 seek
  - `Load CSV` 在 replay mode 下高亮显示，作为当前模式指示
  - 播放到末尾不再自动退出 replay mode；改为停留并提示“Replay finished...Press Stop to exit”
  - 仅 `Stop` 按钮退出 replay mode，避免“播完自动切回 live”导致状态不明确
- **打包脚本补充 ffmpeg 内置**（`scripts/build_mac_JZ.sh`, `scripts/build_win.ps1`）：
  - 打包时自动探测并内嵌 ffmpeg 到 `bin/`，减少用户机器缺少 ffmpeg 导致仅保存图片序列的问题
  - 同时加入 `imageio-ffmpeg` 兜底（依赖安装 + PyInstaller collect），即使构建机未安装系统 ffmpeg 也可打包出可录制 mp4 的版本
  - 支持显式覆盖路径：mac 用 `FFMPEG_BIN`，Windows 用 `FFMPEG_EXE`
- **mac 打包脚本增加签名/公证流程**（`scripts/build_mac_JZ.sh`）：
  - 默认支持 Developer ID 签名（`SIGN_ENABLED=1`，默认身份 `Developer ID Application: Formideep`）
  - 新增可选 notarization 流程（`NOTARIZE_ENABLED=1` + `NOTARY_PROFILE`）与 stapler（`STAPLE_ENABLED`）
  - 打包顺序升级为：`build -> sign -> (optional notarize+staple) -> zip`
  - 脚本支持 `sh build_mac_JZ.sh` 直接调用（自动 re-exec 到 bash）
  - 默认公证关闭（`NOTARIZE_ENABLED=0`）；需要时手动开启并提供 profile（默认值 `exo-notary`）
- **RL Auto Delay 功率评估速度源切换（硬开关，默认角度差分）**（`RPi_Unified/RL_controller_torch.py`）：
  - 新增 `AUTO_PWR_USE_ANGLE_DIFF_VEL=True`（不走 GUI）
  - Auto Delay 的功率/门控评估链路改为可选 `d(angle)/dt` 速度源（仅影响 auto-delay 评估，不影响 NN 主输入）
  - 角度差分实现采用左右腿独立前值状态，加入回绕保护（`AUTO_PWR_ANGLE_WRAP_DEG`）
  - cfg 切换时同步清空评估历史并重置差分前值，避免跨配置速度尖峰污染窗口

---

## [v3.3] - 2026-04-15

### Added

- **GUI CSV 导入重播功能**（`GUI_RL_update/GUI.py`）:
  - 在 Plot 控制区新增 `Load CSV` / `Pause` / 速度选择（`1x/2x/4x/8x`）/ `>>5s` / `Stop`
  - 支持从本地导入历史 CSV 并按时间轴重播
  - 支持暂停/恢复与快进（跳过 5 秒）
  - 重播模式下复用现有曲线与读数刷新逻辑，且不会将重播数据再次写入实时日志 CSV
- 若 CSV 缺失绘图关键词（角度/指令扭矩/估计扭矩/速度/功率），GUI 弹出映射对话框让用户选择对应列，并将结果保存到本地 `GUI_RL_update/mapping.json`
- 功率列支持“自动计算（vel*cmd）”映射：当 CSV 无功率列时，重播阶段实时计算功率
- `GUI_RL_update/mapping.json` 已加入 `.gitignore`，默认作为本地配置文件
- `mapping.json` 支持多对一候选映射（每个目标字段可存多个候选列名），导入时按优先级自动匹配当前 CSV 的可用列
- Replay 倍速新增 `0.2x` / `0.5x` 选项（保留 `1x/2x/4x/8x`）
- 为重播与截图/录制相关按钮新增鼠标悬停提示（tooltip）
- Replay 控件新增 `<<5s` 按钮，支持回退 5 秒

### Fixed

- **EG right torque sign** (`Controller_EG.cpp`): `out.tau_R` 补乘 `-1`，修正右腿助力方向；同步将 `ado_.push_sample` 的 tau_R 参数改为 `-S_R`，使 ADO 功率计算与实际输出方向一致

### Changed

- **Samsung Auto Delay 显示**：auto ON 时 `delay_ms` spinbox 灰掉；下方状态行改为显示相对 base 的有符号 offset（`L:+30ms(V)  R:-20ms(-)`）；加 Reset 按钮（发 falling+rising edge 触发 ADO 冷启动）
- **EG Auto Delay 显示**：auto ON 时同时锁住 `Assist_delay_gain`（delay index）spinbox；状态行由 ms 换算为 idx offset（`÷10`，`L:+2.0idx(V)  R:-1.0idx(-)`）；加 Reset 按钮；auto OFF 时显示当前绝对 idx 值
- **RL Auto Delay 新增双优化器开关（Grid / BO）**：
  - RPi 侧在保留原有局部网格扫描（legacy）基础上，新增 1D Bayesian Optimization（GP + UCB/EI）分支
  - 优化变量保持 `delay_ms`（每腿独立），目标函数为 `J = pos_per_s - λ*max(0, 0.95-ratio)^2 - μ*|neg_per_s|`
  - 现有安全壳不变：`motion_valid`、`dwell`、`AUTO_MAX_STEP_MS`、delay bounds 全部保留
  - GUI RL 面板新增 `Auto Method` 下拉（`Grid (Legacy)` / `Bayes (BO)`），通过 `rpi_passthru[20]` bit3 下发
  - RPi 上行状态 `auto_flags` bit3 回传当前方法，GUI 状态行显示 `Auto=ON/OFF + Method`
- **RL Auto Delay 默认参数按 grid_optimization 最优组更新**（`RPi_Unified/RL_controller_torch.py`）：
  - `AUTO_DWELL_S`：`3.0s -> 0.5s`
  - `AUTO_MAX_STEP_MS`：`20ms -> 40ms`
  - 窗口默认改为固定 `8s`（`AUTO_WINDOW_MIN_S = AUTO_WINDOW_MAX_S = 8.0`）
- **GUI 性能优化（实时与 Replay）**（`GUI_RL_update/GUI.py`）：
  - 绘图刷新与数据接收解耦：接收路径仅入缓冲，绘图按固定帧率刷新（Normal 24 FPS / Eco 16 FPS）
  - 高频 RX 路径中 `_update_rl_filter_state_label` 限频至约 4Hz，降低文本更新抖动
  - Power overlay `setHtml(...)` 改为“内容变化 + 限频”更新，减少富文本重绘成本
  - IMU `setStyleSheet(...)` 改为仅在 OK/FAIL 状态变化时更新
  - Replay 改为时间预算批处理（默认 5ms/tick）+ 单次重绘；`lbl_status` 限频约 8Hz，缓解回放卡顿
  - 新增诊断与执行清单文档：`Docs/GUI_PERFORMANCE_TODO.md`
  - 默认 `Win` 维持为 `200`（可在 GUI 中按需调小），在流畅性与历史可视范围之间保持平衡
  - 默认渲染帧率微调为 `Normal 24 FPS / Eco 16 FPS`
  - 曲线启用 `clip-to-view + auto downsampling`，并关闭默认 antialias，减少 pyqtgraph 渲染成本
- **Max Torque GUI 默认值更新**（`GUI_RL_update/GUI.py`）：
  - `Max Torque` 上限保持 `30 Nm`
  - 默认值由 `12 Nm` 调整为 `15 Nm`
  - POWER OFF 后再次开启时的回退默认值同步改为 `15 Nm`
- **Hip 参数区 tooltip 补齐**（`GUI_RL_update/GUI.py`）：
  - 为新增/常用参数补充说明：`Max Torque`、EG 参数、Samsung `Kappa/Delay`、RL `Scale/Delay/Filter/Auto Method`
  - 同步给对应标签与控件添加 tooltip，便于现场调参与新用户理解
- **新增跨平台打包脚本**（`scripts/build_mac.sh`, `scripts/build_win.ps1`）：
  - 支持一键打包 Hip GUI 为 macOS `.app` 与 Windows `.exe`
  - 输出目录统一落在 `GUI_RL_update/release/{mac|windows}/<timestamp_gitsha>/`
  - 默认启用图标与版本号标注（图标源：`scripts/assets/app_icon.jpeg`）
  - 支持 `APP_VERSION`、`ICON_SOURCE`、`ICON_ENABLED`、`VERSION_ENABLED`、`SKIP_DEP_INSTALL`、`FULL_BUILD` 等可选参数
- **打包大文件忽略规则补充**（`.gitignore`）：
  - 新增忽略 `GUI_RL_update/release/`, `GUI_RL_update/dist/`, `GUI_RL_update/build/`, `GUI_RL_update/*.spec`, `GUI_RL_update/.venv-build-win/`
  - 新增通用忽略 `*.app`, `*.zip`，避免误提交打包产物到 GitHub

---

## [v3.2] - 2026-04-14

Multi-user workflow fixes: personal configuration (BLE pairing name, RPi connection info) moved out of source files into gitignored local config files, eliminating merge conflicts for multi-developer setups.

### Added

- **`ble_device_config.h` per-user BLE device name**:
  - Removed hardcoded `"Shutong"` from both BLE Bridge sketches (central + peripheral)
  - Both now `#include "ble_device_config.h"` (gitignored) and reference `BLE_DEVICE_NAME`
  - `ble_device_config.h.example` committed in each sketch folder as template
  - `#error` guard gives a clear compile-time message if the file is missing
  - Eliminates merge conflicts when multiple users push different BLE names

- **`tools/rpi_profiles.conf` multi-user Pi profile management**:
  - `tools/rpi_sync.py` now reads connection info (host / user / remote_dir / password) from a gitignored `tools/rpi_profiles.conf` (INI format)
  - `[active] profile = <name>` in the conf file selects the default profile; override at runtime with `--profile <name>`
  - `tools/rpi_profiles.conf.example` committed as template (documents both `biodyn-pi5` and `aboutberlin` profiles)
  - Removed the per-user commented-out `DEFAULT_*` blocks from `rpi_sync.py` that previously caused merge conflicts
  - Clear error message if no conf file is found and required params are missing

---

## [v3.1] - 2026-04-09 ~ 2026-04-13

### Added

- **RL Auto Delay 自动延迟优化**（RPi 侧，RL 模式）:
  - RPi 新增 1Hz 侧环实时功率指标计算（`power_ratio` / `pos_per_s` / `neg_per_s`）
  - 双目标优化：`ratio ≥ 0.95` 优先，满足后最大化 `pos_per_s`
  - 仅在”有效运动”窗口内（速度 + 绝对功率门控）执行 delay 调参
  - 自适应窗口（按步态周期估计 4~12s）；固定 dwell 3s（可切换自适应模式 `AUTO_DWELL_ADAPTIVE`）
  - GUI RL 面板新增 `Auto Delay` 开关；`Power Sign` 子图右侧新增 overlay（`+Ratio%` / `+P / -P` / delay）
  - **Auto Delay OFF 时功率指标仍持续计算**：1Hz 评估不依赖 `auto_delay_enable`，仅”扫描+apply”步骤被门控；GUI overlay 数字始终有效
- **RL Auto Delay 左右腿独立 (per-leg L/R)**:
  - `runtime_delay_ms` 拆为 `runtime_delay_ms_L/R`；auto ON 后两腿独立漂移
  - `_auto_step_leg` 对 L/R 各调一次，各自维护 dwell / best_delay
  - 推理 delay buffer 按 `delay_frames_L/R` 分别移相
- **Serial8 `AA 56` RPi→Teensy 状态帧升级为 v3**（40B 不变，内容重组为 per-leg int16 packed）:
  - `[16..19]` delay_ms_L/R ×10；`[20]` auto_flags (bit0=enable, bit1=mvL, bit2=mvR)
  - `[24..35]` ratio/pos_per_s/neg_per_s per-leg (×10000 / ×100 / ×100)；`[36..39]` best_delay L/R ×10
  - GUI v3/v2 版本分发解析；老 `_rpi_*` 字段作为 L/R 均值/总和别名保留
- **Teensy-native Auto Delay（Samsung / EG 无 RPi 时）**:
  - 新建 `AutoDelayOptimizer.h/.cpp`：Teensy C++ 工具类，功能与 RPi 侧对等
  - `Controller_Samsung` 集成：`ado_` 直接控制 `delay_ms_L/R`，驱动内部延迟环形缓冲
  - `Controller_EG` 集成：新增后处理输出延迟层（`EG_POST_DELAY_BUF=200`），`ado_` 控制 `post_delay_ms_L/R`；现有 `Assist_delay_gain`（相位延迟）不变
  - `BleProtocol.h` 下行 `payload[28]` bit0 = `auto_delay_enable`（EG/Samsung）；`[29..30]` = `eg_post_delay_ms`
  - `.ino` 1Hz 侧环调 `tick_auto_delay()`；`Transmit_ble_Data` 非 RL 模式填充 v3 blob；算法切换重置 ADO 时间戳
  - GUI Samsung/EG 面板各新增 Auto Delay 开关 + 实时状态行；`_tx_params` 打包新字段

### Changed

- Serial8 IMU 帧去除 `exo_delay` 字段：Teensy→RPi 由 `5×float32 + 11B logtag`（36B）改为 `4×float32 + 11B logtag`（32B）；RPi 解析逻辑同步更新（TYPE=0x01 payload 27B）
- RL 延迟改为运行时参数 `runtime_delay_ms`（GUI passthrough 下发），不再读取包内 `exo_delay`
- LSTM-PD 默认扭矩倍率下调为 `scale=0.40`（GUI 预设与 RPi 启动默认对齐）
- RPi `lstm_pd` 新增可选 zero-mean 输出平衡（开关在 `RL_controller_torch.py` 顶部），作用于”滤波后、delay 前”
- GUI `Power Sign` overlay 文字升级：字号 15pt，上移至子图内；左右 strip 各显示本腿独立指标（`+Ratio` / `+P·-P (total)` / `Delay=Xms`）
- `RPi_Unified/run.sh` 支持交互菜单或命令行参数选择 NN 类型（`dnn` / `lstm` / `lstm_leg_dcp` / `lstm_pd`）；脚本自身定位取代硬编码路径 *(2026-04-10, Shutong)*

### Fixed

- RL auto delay motion 门控解耦 `runtime_scale`：评估内部固定 scale=1.0，避免 scale 调小时门控永远不触发；回传 GUI 前乘 scale，保持功率语义
- RL auto delay 冷启动 dwell：`auto_delay_enable` False→True 上升沿时重置 `auto_last_change_ts_L/R`，首轮等满 dwell 再扫描
- GUI cfg 下发时清空 `hist_tau_src/vel/ang` 缓冲并重置 `auto_last_eval_ts`，避免切换配置后旧数据污染评估窗口
- `dwell` 改为固定 3.0s（`AUTO_DWELL_ADAPTIVE=False` 默认）；`AUTO_DWELL_ADAPTIVE=True` 时走 `max(AUTO_DWELL_S, window_s + 1.0)`
- **Auto Delay OFF 时功率指标停更（修复）**：1Hz 触发条件移除 `auto_delay_enable` 守护；cfg handler 不再在 auto 关闭时清零 ratio/pos_per_s/neg_per_s（仅清 motion_valid）
- `RPi_Unified/RL_controller_torch.py` 首次启动若 `./output/` 不存在自动创建（`os.makedirs` exist_ok） *(2026-04-10, Shutong)*
- GUI `Power Sign` overlay：修复左腿文本被裁切；统一隐藏底轴数值；RPi 状态解析新增范围与跳变校验，丢弃异常帧

### Documentation

- 更新 `Docs/SYSTEM_ARCHITECTURE.md`：Serial8 帧长与 IMU 帧定义
- 更新 `RPi_Unified/README.md`：新增 Auto Delay 功能说明（开关、指标定义、OFF 时仍计算的行为、关键参数表）
- 新增 `Docs/RL_AUTO_DELAY_OPTIMIZATION.md`：完整设计文档（功率指标定义、窗口策略、门控条件、优化流程、per-leg 独立机制、AUTO_DWELL_ADAPTIVE 开关、Auto OFF 时指标仍更新说明）

---

## [v3.0] - 2026-03-20

### Added
- **Multi-Algorithm Architecture**: Runtime-switchable control algorithms via GUI without recompilation
  - `Controller` abstract base class (`Controller.h`) with `compute()`, `parse_params()`, `reset()` interface
  - `Controller_EG`: Energy Gate algorithm (~12 parameters)
  - `Controller_Samsung`: Samsung algorithm (kappa, delay)
  - `Controller_RL`: RL neural network — Serial8 passthrough to RPi
  - `Controller_Test`: Fixed/sine torque test mode
- **MotorDriver abstraction layer** (`MotorDriver.h`): SIG (ODrive) / T-Motor (VESC) runtime switch via GUI
- **RPi_Unified**: Unified RL controller, single entry point for all 4 network types
  - `dnn`: DNN feedforward (18→128→64→2)
  - `lstm`: LSTMNetwork (4→256→2)
  - `lstm_leg_dcp`: Per-leg decoupled LSTM (2→256→1 × 2)
  - `lstm_pd`: LSTM-PD (2→256→1 × 2, position error control)
- **GUI v4.0**: RL panel with RPi online/offline status, dynamic parameter panel per algorithm, algo select + confirm mode, dual direction control (motor vs plot), Dark/Light theme

### Changed
- BLE frame expanded to **128 bytes** (up from previous size), with RPi passthrough zone (40B each direction)
- Serial8 protocol: IMU frame (TYPE=0x01, 32B) + GUI passthrough frame (TYPE=0x02, 41B)
- Project restructured into `All_in_one_hip_controller_RL_update/`, `RPi_Unified/`, `GUI_RL_update/`

---

## [v2.x] - Archive

Refer to `Archive_Old_Reference/` and `Deployment/` for prior versions:
- `All_in_one_hip_controller/`: Single-algorithm Teensy firmware (no runtime switching)
- `GUI/`: GUI without RL panel
- `Deployment/RL_controller_torch.py`: Single LSTM-PD deployment script (verified working)
