# RL Real-Time Latency & Sync Spike Fix Notes

**Scope:** `## [Unreleased] -> ### Fixed` in `Docs/CHANGELOG.md`  
**Date:** 2026-04-22/24  
**Related files:**  
- `RPi_Unified/RL_controller_torch.py`  
- `All_in_one_hip_controller_RL_update/Controller_RL.h`  
- `All_in_one_hip_controller_RL_update/Controller_RL.cpp`  
- `GUI_RL_update/GUI.py` (display path behavior + POWER OFF double-click fix)

---

## 0. TL;DR

本次修复对应两个连续暴露的问题：

1. **Pi backlog（高优先）**：不是 Serial8 本身慢，而是 Pi 主循环偶发跑不到 100Hz，消费速度小于 Teensy IMU 生产速度，导致 UART RX FIFO 累积老包，Pi 算的是几秒前的 IMU。  
2. **Sync spike（二次暴露）**：backlog 修复后，Pi→Teensy AA59 100Hz 稳态发送让 Teensy `Serial8` 默认 64B RX 缓冲更容易 overrun；AA59 无 checksum，偶发错位后把 int16 解成近饱和值，GUI Sync 看到 ±300° / ±3000 dps。

对应修复：
- Pi 端：**每轮丢弃过期 IMU，只保留最新包** + 去掉 `ser.flush()` 阻塞。  
- Teensy 端：`Serial8` RX 缓冲扩到 512B + AA59 parse 后做物理量健壮性检查，坏帧丢弃并沿用上一帧。

---

## 1. 问题 A：Pi backlog 导致“秒级陈旧”

### 1.1 现象

- RL 模式下，GUI 显示明显慢于真实动作。  
- 典型表现：晃动 IMU 时，Pi CSV (`imu_LTx/imu_Lvel`) 在很长时间内几乎不变；Teensy 仍在持续产生新 IMU。  
- `Data Source=Auto` 体感更差，`Raw` 看起来相对正常（因为 Raw 角/速来自 Teensy 直上报）。

### 1.2 高置信根因

不是 Serial8 线本身卡，而是：

- Teensy 在高频持续送 IMU 到 Pi（生产端）；
- Pi 主循环是 100Hz 消费端，但 v4.0 后每周期开销叠加，一旦短暂掉速到 60–80Hz，
- Linux UART FIFO 没有 catch-up 机制，旧包持续堆积；
- Pi 每轮只取一个包，系统就会长期在“读老包 → 算老 torque → 下发老命令”状态。

### 1.3 证据链（按截图整理）

1. Pi 端时间在走，但 IMU 数值长时间基本不动，说明读到的是 FIFO 旧包。  
2. 生产/消费失衡：Teensy 生产 100Hz（甚至更快），Pi 消费端一旦低于生产端，积压单调增加。  
3. v4.0 周期开销叠加（AA59 更大 payload、统一 torque IIR、lstm_pd zero-mean 统计、sync buffer 操作、CSV 写入字段增多、日志输出等），把系统从“有余量”推到“无余量”。  
4. Auto 模式更慢是因为显示源切换到 sync/control 链路，而该链路同样受 backlog 影响。  
5. “加快晃动 IMU”不会解决，因为 FIFO 是队列，读头依然是历史样本。

### 1.4 已实施修复

1. **只保留最新 IMU 包**：在 Pi 侧循环读取时，主动丢掉 backlog 中旧 IMU。  
2. **移除 `ser.flush()`**：避免每周期 `tcdrain` 阻塞，恢复主循环预算。

### 1.5 预期结果

- Pi 计算输入回到“最新 IMU”；  
- torque 不再表现为秒级滞后；  
- GUI 的 Pi 相关显示（尤其 sync/control 路径）恢复实时性。

---

## 2. 问题 B：Sync 路径偶发爆帧（±300° / ±3000 dps）

### 2.1 现象

- backlog 修复后，`Data Source=Sync` 偶发尖峰；`Raw` 路径干净。  
- 典型异常值接近 int16 饱和换算：  
  - 角度：`±32500 / 100 ≈ ±325°`  
  - 速度：`±32767 / 10 ≈ ±3276.7 dps`

### 2.2 关键诊断结论

- 不是“显示问题”，而是 Teensy 解析到的 sync 字段本身错。  
- 同 `sample_id` 对齐比对显示：Pi 侧 `sync_LTx/sync_Lvel` 在正常量程，但 GUI 侧 sync 字段偶发近饱和。  
- `sync_sample_id`（AA59 前 2 字节）与 `sync_cmd_L`（本地 cmd 通道）经常是干净的，坏的集中在 AA59 payload 某段 int16 字段，符合“丢字节后错位解析”特征。

### 2.3 高置信根因

- Teensy `Serial8` 默认 RX 缓冲仅 64B。  
- Pi→Teensy AA59 帧 42B，100Hz 连续发送；Teensy 主循环若被 BLE/SD/算法短暂占用，会发生 RX overrun。  
- AA59 无 checksum，状态机仍按 42B 凑包，导致部分字段读到跨帧随机字节并解成近饱和值。

### 2.4 已实施修复

1. **扩 RX 缓冲到 512B**：`Serial8.addMemoryForRead(..., 512)`，显著提升抗抖能力。  
2. **AA59 健壮性校验**：先解到临时变量，检查物理量范围（angle/vel/pwr），坏帧直接丢弃并沿用上一帧。  
3. **坏帧计数器**：`bad_sync_frames` 便于后续现场观测。

### 2.5 预期结果

- Sync 路径不再出现大尖峰；  
- 即便偶发坏帧，也不会污染 GUI 显示或控制输出。

---

## 3. 为什么这两个问题会连续暴露

- 修 backlog 前：Pi 实际输出 AA59 速率被 backlog 间接拉低，Teensy RX 压力小，overrun 不明显。  
- 修 backlog 后：Pi 恢复稳态高频发送，RX 压力上来，原先潜在的 Serial8 overrun 暴露出来。  

所以这是**先后触发的两层问题**，不是互相矛盾。

---

## 4. 验证建议（现场）

1. 刷新 Teensy + Pi 最新代码后，固定 RL（如 `lstm_leg_dcp`）运行。  
2. 先看 backlog：晃动 IMU，Pi CSV 的 `imu_LTx/imu_Lvel` 应实时变化，不再长时间平。  
3. 再看 sync：切到 `Data Source=Sync`，观察是否还有 ±300° / ±3000 dps 尖峰。  
4. 若仍偶发异常，记录 `bad_sync_frames` 增长速度与现场负载（BLE/SD/打印）一起分析。  

---

## 5. 说明

- 本 README 是 `Unreleased/Fixed` 的工程化补充说明。  
- 更早的功率定义与 GUI/RPi 对齐背景，见：`Code Debug/BUG_ANALYSIS.md`。  

---

## 6. 问题 C：POWER OFF 按钮偶发需要点两次

### 6.1 现象

- GUI 左上 `POWER OFF / POWER ON` 大按钮，**关机时有时需要点 2 次才真正关掉**（第一次点击按钮只是颜色文字变化，力矩没归零；第二次才真把 `sb_max_torque_cfg` 拉到 0）。
- 问题不是 Qt click 没注册（visual 状态每次都响应），而是**逻辑上点了一次才生效**。

### 6.2 高置信根因

代码里存在一个隐含不变量：  
> `btn_power.isChecked()`  ⇔  `sb_max_torque_cfg.value() > 0`

但这个不变量 **只在 `_on_power_toggled` 路径里维持**。以下路径会让两者失同步：

- 用户直接在 Parameters 面板里手动编辑 `Max Torque` 数字；  
- 用户点 `QDoubleSpinBox` 上下箭头把 Max Torque 从 0 调到非 0；  
- 任何把 torque 值改了、但没过 `_on_power_toggled` 的路径。

一旦失同步（`btn_power` 仍 unchecked / 红色 "POWER OFF"，而 `sb_max_torque_cfg > 0`），用户点一下按钮会：

1. Qt 把 checkable 按钮从 unchecked 翻到 checked → 触发 `_on_power_toggled(True)`；  
2. `_on_power_toggled(True)` 里 `setValue(val)`，其中 `val = _maxT_before_off`，而当前 spinbox 已经是 15，属于 no-op；  
3. `_set_power_ui(True)` 把按钮刷成绿 "POWER ON"；
4. **此时力矩其实一直是 15，没变过**。

用户眼里：我想关，按了一次 → 按钮反而变绿、motor 还在转。再点一次 → `_on_power_toggled(False)` → `setValue(0)` → 绿变红，motor 停。**2 次点击才关掉**。

这个路径和 BLE / AA59 / Pi backlog 都无关，是纯 GUI 内部状态不一致。

### 6.3 已实施修复

`GUI_RL_update/GUI.py`：给 `sb_max_torque_cfg.valueChanged` 新接一个 handler `_sync_power_btn_from_torque(value)`，任何路径改了 spinbox 值都会被它看见：

- `value > 0` 且按钮当前 unchecked → 自动把按钮刷成 "POWER ON" 绿；  
- `value == 0` 且按钮当前 checked → 自动把按钮刷成 "POWER OFF" 红；  
- 已同步则立即 return，避免无意义的 stylesheet 重建。

核心 3 行保护措施：

1. 进入前 `want_checked == isChecked()` 快速 return，避免在 `_on_power_toggled` → `setValue()` → `valueChanged` → `_sync_power_btn_from_torque` 再回 `_set_power_ui` 的链条里多绕一圈。  
2. 调用 `_set_power_ui` 前用 `btn_power.blockSignals(True)` 抑制 `setChecked` 引起的 `toggled` 回调；否则会重新进入 `_on_power_toggled`，把 spinbox 值又刷一次，形成回环。  
3. 改完立刻 `blockSignals(False)` 恢复，不影响后续用户点击。

代码位置：  
- 连接点在 `sb_max_torque_cfg` 创建处（`_build_layout` 里的 `make_dspin(15.0, ...)` 之后）。  
- handler 方法定义紧跟在 `_on_power_toggled` 下面。

### 6.4 预期结果

- 直接编辑 Max Torque 数字或点箭头 → 按钮实时跟随颜色/文字，不变量恢复。  
- POWER OFF 永远一次点击到位（按钮 checked 状态总是跟实际 torque 状态一致，点击 `toggled` 总是携带真正的意图方向）。  
- 其他路径（`_on_power_toggled`、连接时 `_set_power_ui(sb_max_torque_cfg > 0)`、掉线时强制关机、主题刷新）都不变，安全。

### 6.5 验证建议

1. GUI 启动后，先不操作，直接 POWER OFF 按钮点一次 → 无反应（已经是红的，value=0）。正常。  
2. 点按钮变绿 "POWER ON"（torque=15），再点一次 → 红 "POWER OFF"（torque=0）。一次到位。  
3. 手动在 Max Torque 里敲 `15` 回车 → 按钮**自动变绿**。此时点一次按钮 → 直接红。**不再需要两次**。  
4. 手动把 Max Torque 从 15 敲到 `0` → 按钮**自动变红**，与不变量一致。
