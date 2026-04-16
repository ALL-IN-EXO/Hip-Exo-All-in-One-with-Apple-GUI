# Changelog

All notable changes to this project will be documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

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
