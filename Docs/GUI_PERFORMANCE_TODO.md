# GUI 性能优化 TODO（卡顿问题定位与执行）

更新时间：2026-04-15  
范围：`GUI_RL_update/GUI.py`

## 1. 已确认的卡顿来源

1. 主线程高频重绘负担过高
- 之前每个新样本都会触发整套 `setData(...)`、power strip 更新、可选 `setXRange(...)`。
- 高频 `deque -> list` 转换带来明显 CPU 和内存抖动。

2. Replay 单帧处理量过大
- 之前 replay 在一个 tick 内可处理大量样本，并且“每个样本都重绘 + 更新状态文本”。
- 导致快放或大 CSV 时出现明显卡顿/顿挫。

3. Power overlay 富文本更新过于频繁
- `setHtml(...)` 是相对昂贵操作，之前更新过于频繁。

4. IMU 标签样式重复刷新
- IMU 状态没变时仍反复 `setStyleSheet(...)`。

5. RL 状态面板刷新频率过高
- `_update_rl_filter_state_label()` 在高频 RX 路径里被频繁触发，文本刷新开销大。

## 2. 本轮已执行（并已完成打勾）

### A. 实时绘图路径（低成本 / 高收益）

- [x] 绘图刷新与数据接收解耦
  - 接收路径只入缓冲，不在每个样本点直接重绘。

- [x] 固定绘图帧率（20~30 FPS）
  - Normal：24 FPS。
  - Eco：16 FPS。
  - 目的：进一步降低长期运行时主线程负载。

- [x] 默认窗口点数调整
  - 默认值保持为 `Win=200`（用户可按需要改小以进一步减负）。

- [x] `_update_rl_filter_state_label` 限频到约 4 Hz
  - 在高频 RX 更新中增加最小刷新间隔。

- [x] Power overlay 改为“数值变化 + 限频”才更新 `setHtml`
  - 仅当文本变化且达到最小间隔才刷新 HTML。

- [x] IMU `setStyleSheet` 仅在状态变化时更新
  - OK/FAIL 没变化时不再重复改样式。

预期收益：
- 全曲线开启时，UI 流畅度明显提升。
- CPU 占用通常可下降约 30%~60%（与机器/窗口大小相关）。

### B. Replay 专项（低成本 / 超高收益）

- [x] replay 每 tick 可批量处理样本，但只重绘一次
  - 批量入缓冲后统一刷新画面。

- [x] 固定大步长上限改为“时间预算”
  - 默认每 tick 预算约 5ms，避免单帧阻塞。
  - `max_steps` 仅保留给少数启动场景。

- [x] `lbl_status` 限频到约 8 Hz
  - 防止 replay 期间状态文本过度刷新造成主线程阻塞。

- [x] pyqtgraph 轻量性能参数
  - 对曲线启用 `setClipToView(True)` 与自动 downsampling。
  - 关闭全局抗锯齿（`antialias=False`）。

预期收益：
- replay 的“突然卡死/大顿挫”会显著减少。

## 3. 后续可选优化（本轮未做）

- [ ] 给渲染帧率增加 GUI 可调开关（例如 15/24/30 FPS）
- [ ] 加一个“高画质模式”开关（开启 antialias）
- [ ] 将热路径缓冲升级为 numpy ring buffer，进一步减少转换开销
