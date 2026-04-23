# RL Real-Time Latency & Sync Spike Fix Notes

**Scope:** `## [Unreleased] -> ### Fixed` in `Docs/CHANGELOG.md`  
**Date:** 2026-04-22/23  
**Related files:**  
- `RPi_Unified/RL_controller_torch.py`  
- `All_in_one_hip_controller_RL_update/Controller_RL.h`  
- `All_in_one_hip_controller_RL_update/Controller_RL.cpp`  
- `GUI_RL_update/GUI.py` (display path behavior)

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
