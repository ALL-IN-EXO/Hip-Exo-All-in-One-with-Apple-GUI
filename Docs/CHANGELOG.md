# Changelog

All notable changes to this project will be documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [2026-04-10] Shutong

### Changed
- `RPi_Unified/run.sh` 支持选择神经网络类型：无参数时进入交互菜单（dnn / lstm / lstm_leg_dcp / lstm_pd），也可命令行直接指定 `./run.sh <nn>`；脚本自身定位取代硬编码 `cd RPi_Unified`，并新增 `#!/bin/bash` shebang 与可执行权限

### Fixed
- `RPi_Unified/RL_controller_torch.py` 首次启动时若 `./output/` 目录不存在会在写 CSV 前报错；新增 `os.makedirs('./output', exist_ok=True)` 自动创建目录

---


### Changed
- Serial8 IMU 帧去除 `exo_delay` 字段: Teensy→RPi 从 `5×float32 + 11B logtag` 改为 `4×float32 + 11B logtag`，总帧长由 36B 改为 32B
- `RPi_Unified/RL_controller_torch.py` 同步更新 IMU 解析逻辑 (TYPE=0x01 payload 从 31B 改为 27B)
- RL 延迟统一使用运行时参数 `runtime_delay_ms`（GUI passthrough 下发），不再读取包内 `exo_delay`
- RL 自动延迟优化（可开关）:
  - RPi 新增实时功率指标计算（`power_ratio` / `pos_per_s` / `neg_per_s`）
  - 仅在“有效运动”窗口内执行 1Hz 延迟调参（局部扫描 + 步长限制 + dwell 防抖）
  - GUI RL 面板新增 `Auto Delay` 开关，并显示 RPi 回传的实时指标与推荐延迟
  - GUI 的左右 `Power Sign` 子图标题增加 RPi 回传的 `+Ratio(%)` 与 `Auto/motion` 状态
  - GUI 优化：`Auto Delay` 打开后锁定 `Torque Delay` 输入框，并实时同步显示 RPi 当前生效的 delay
  - GUI 优化：`Power Sign` 子图右侧新增图内 overlay 指标（`+Ratio(%)` + `Auto/motion`），替代标题塞字
  - GUI 细化：`Auto Delay` 打开时，`Torque Delay (ms)` 行改为灰色只读 `L/R` 显示；`Power Sign` overlay 改为两行并上移，避免左腿文本被裁切
  - GUI 细化：功率显示增加“仅显示路径”的速度去毛刺（绝对值/跳变阈值守卫），抑制 IMU 单帧速度尖峰导致的瞬时异常大负功率
  - GUI 稳定性：`Power Sign` 左腿 strip 与右腿统一隐藏底轴数值，修复大字 overlay 偶发被挤压/不可见；RPi 状态解析新增范围与跳变校验，丢弃异常帧以避免 delay/power 瞬时跳变显示
- RL 自动延迟改为左右腿独立 (per-leg L/R delay)：
  - RPi `runtime_delay_ms` 拆成 `runtime_delay_ms_L/R`，auto 关闭或 cfg 下发时两腿同步到 GUI 基础 delay；auto 打开后两腿独立扫描、独立 dwell、独立推荐
  - `evaluate_delay_candidate` 改为 `evaluate_delay_candidate_leg`，每侧只用本腿 `tau/vel` 做评估
  - auto 状态机内部 `_auto_step_leg(...)` 对 L/R 各调一次，各自返回更新后的 delay / best / metrics
  - 推理输出的 delay buffer 按 `delay_frames_L/R` 分别移相
- LSTM-PD 默认扭矩倍率下调为 `scale=0.40`（GUI 预设与 RPi 启动默认对齐）
- RPi 新增 `lstm_pd` 专用可选 zero-mean 输出平衡（开关在 `RL_controller_torch.py` 顶部参数区）：
  - 仅作用于 `--nn lstm_pd`
  - 作用位置为“滤波后、delay 前”
  - 通过滑窗均值去偏置，目标是让扭矩围绕 0 更对称
  - CSV 日志 `torque_delay_ms` 拆为 `torque_delay_ms_L / torque_delay_ms_R`
- Serial8 `AA 56` RPi→Teensy 状态帧版本号由 `0x02` 升为 `0x03`，仍保持 40B 载荷不变：
  - `[16..17] delay_ms_L (int16 ×10)`、`[18..19] delay_ms_R (int16 ×10)`
  - `[20]` auto flags: bit0=auto_enable, bit1=motion_valid_L, bit2=motion_valid_R
  - `[24..27] ratio_L/R (int16 ×10000)`
  - `[28..31] pos_per_s_L/R (int16 ×100)`
  - `[32..35] neg_per_s_L/R (int16 ×100)`
  - `[36..39] best_delay_L/R (int16 ×10)`
  - GUI 解析走版本分发：v3 读 int16 对、v2 (legacy float) 回退时把单值镜像到 L/R，老 `_rpi_*` 字段作为 L/R 的均值/总和别名保留
- GUI RL 面板显示升级为左右分栏：
  - 左右 `Power Sign` overlay 各自显示本腿 `+Ratio` / `+P · -P` / `Delay=Xms · auto/motion` 三行
  - RL 面板底部 `Auto Delay(RPi)` 展开为 `L: ... / R: ...` 两行
  - RL 滤波状态行在 L/R delay 不同时显示 `Delay L/R=X/Y ms`，相同时退化为单值

### Fixed
- RL 自动延迟 motion 门控解耦 `runtime_scale`：评估内部固定用 scale=1.0，避免 GUI 把 scale 调小时 `abs_power_per_s` 永远达不到阈值导致 auto 失效；显示的 `pos_per_s`/`neg_per_s` 在回传前再乘 scale，保持实际功率语义
- RL 自动延迟新增冷启动 dwell：检测到 `auto_delay_enable` False→True 上升沿时重置 `auto_last_change_ts`，首轮扫描需等满 `AUTO_DWELL_S` 让 history 先填充
- GUI cfg 到达时清空 `hist_tau_src/vel/ang` 缓冲并重置 `auto_last_eval_ts`，避免滤波器/scale/delay 切换后窗口混入旧配置数据做评估
- RL 自动延迟 dwell 增加 `AUTO_DWELL_ADAPTIVE` 开关：`True` 走自适应 `max(AUTO_DWELL_S, auto_window_s + 1.0)` 等窗口 100% 新数据，`False`（默认）固定 `AUTO_DWELL_S`；常量默认值从 2.0s 提高到 3.0s
- GUI `Power Sign` overlay 明确标注：`+Ratio` 标为 `(L+R avg)`、功率数值新增第二行 `+P / -P (L+R total)`，消除"两条 strip 显示同一个均值"时的歧义；底部 `Auto Delay(RPi)` 状态行同步补齐标注

### Documentation
- 更新 `Docs/SYSTEM_ARCHITECTURE.md` 中 Serial8 帧长与 IMU 帧定义
- 更新 `RPi_Unified/README.md` 中 Teensy→Pi IMU 帧字段说明
- 新增 `Docs/RL_AUTO_DELAY_OPTIMIZATION.md`：自动 delay 调参的设计思路、功率指标定义、门控条件与优化流程

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
