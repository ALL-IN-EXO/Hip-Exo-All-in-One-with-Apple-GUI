# Changelog

All notable changes to this project will be documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Changed

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
