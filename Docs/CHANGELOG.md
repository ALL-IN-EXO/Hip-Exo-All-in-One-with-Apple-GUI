# Changelog

All notable changes to this project will be documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added

- **PyInstaller 打包脚本**（`scripts/build_win.ps1`, `scripts/build_mac.sh`）:
  - Windows PowerShell 脚本产出 `.exe` 目录 + `.zip`，保存到 `GUI_RL_update/release/windows/<时间戳_gitsha>/`
  - macOS Bash 脚本产出 `.app` bundle + `.zip`（用 `ditto` 保留 bundle symlink），保存到 `GUI_RL_update/release/macos/<时间戳_gitsha>/`
  - 两份脚本默认入口 `GUI_RL_update/GUI.py`，可通过 `-Entry/-AppName/-SourceDir`（Win）或 `--entry/--name/--source`（Mac）覆盖
  - 依赖自动装入独立 venv（`.venv-build-win/`, `.venv-build-mac/`）：`pyqt5 pyqtgraph pyserial numpy pyinstaller`
  - 提供 `FullBuild`/`--full` 模式（`--collect-all PyQt5 pyqtgraph`）和默认 SLIM 模式（针对 pyqtgraph 保留 QtSvg/QtPrintSupport/QtOpenGL 等常用依赖，仅排除 Qt3D/QtWebEngine/QtQml 等明显无关模块）
- `.gitignore` 新增 PyInstaller 产物忽略规则：`.venv-build-*/`、`release/`、`*.spec`

### Changed

- **Hip GUI 连接栏交互调整**（`GUI_RL_update/GUI.py`）：
  - 左上角 `Eco` 开关替换为手动 `Refresh` 按钮，用于即时刷新串口列表
  - 渲染刷新率改为固定使用 Normal 帧率（不再由 Eco 开关切换）
- **Hip GUI 录屏稳定性改进**（`GUI_RL_update/GUI.py`）：
  - 录屏帧从 PNG 改为 JPG（默认质量 90），显著降低实时落盘开销
  - 录屏状态文本限频更新（约 5 Hz），减少高频 UI 文本重绘
  - 录屏写盘迁移为后台线程 + 有界队列（主线程仅抓帧入队），降低录制过程主线程阻塞导致的卡顿
  - 队列拥塞时启用丢最旧帧策略，优先保证实时交互与当前画面
  - ffmpeg 查找逻辑新增“打包内 bin + 环境变量 + 系统路径”多级解析
  - ffmpeg 编码参数改为 `libx264 + ultrafast + even-dimension padding`，降低编码耗时并避免奇数分辨率失败
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
