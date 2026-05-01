# GUI 性能优化 TODO（卡顿问题定位与执行）

更新时间：2026-05-01  
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

6. 实时 CSV 写盘仍在主线程
- 虽然单次 `writerow(...)` 不大，但长时间运行时会叠加磁盘 flush、杀毒扫描、慢盘抖动。
- 这类问题在不同电脑上差异很大，是“有些电脑更卡”的典型来源之一。

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

- [x] 实时 CSV 记录改为后台线程写盘
  - 主线程只负责组装一行数据并入队，不再直接 `writerow/flush`。
  - 目的：降低慢磁盘、杀毒软件、系统 flush 对 UI tick 的阻塞。

- [x] 高频状态文本改为“限频 + 仅变化时更新”
  - `lbl_status` 实时状态降到约 10 Hz。
  - Tag 面板计时/步频降到约 10 Hz。
  - 多个 badge / value label 仅在文本或颜色变化时才真正调用 Qt 刷新。

预期收益：
- 全曲线开启时，UI 流畅度明显提升。
- CPU 占用通常可下降约 30%~60%（与机器/窗口大小相关）。
- 在慢盘或配置一般的电脑上，长时间运行时的“偶发顿一下”会明显减少。

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

## 3. 当前线程现状（截至 2026-05-01）

### A. 仍在主线程的路径

- `QTimer -> _update_everything()` 主循环仍在 GUI 主线程内执行。
- 串口 `in_waiting/read`、BLE 帧拼包、`_handle_uplink_payload()` 解析目前仍在主线程。
- pyqtgraph 曲线刷新仍在主线程：
  - `_maybe_render_plot()`
  - `_render_plot_frame()`
  - `PlotDataItem.setData(...)`
  - power strip 的 `setData(...) / setHtml(...) / setXRange(...)`
- 结论：当前 GUI 不是“多线程绘图”；只是把部分非 UI 工作移出了主线程。

### B. 当前已经存在的后台线程

- [x] CSV 写盘线程
  - 主线程只组装 telemetry 行并入队。
  - 后台线程负责 `writerow(...)` 和周期性 `flush()`。
  - 作用：减少磁盘写入对 UI tick 的阻塞。

- [x] 录像写帧线程
  - 录屏时主线程抓取 `QImage` 后入队。
  - 后台线程负责把图片保存到临时目录。
  - 作用：避免逐帧写图文件拖慢界面。

- [x] Pi RL 远程命令异步线程
  - 启动 / 停止 / 轮询 tmux 状态时，`subprocess.run(...)` 放在线程里执行。
  - 作用：避免 SSH 命令阻塞 GUI。

### C. 为什么没有把“绘图本身”放进线程

- Qt / PyQt 的控件更新原则上应留在主线程。
- `pyqtgraph` 的 `setData(...)` 本质上也是 GUI 对象更新，不适合直接放到 Python 后台线程。
- 如果强行把真正的绘图更新放到后台线程，常见风险是：
  - 随机崩溃
  - 偶发卡死
  - 跨线程 UI 调用导致的未定义行为

### D. 下一步最推荐的线程化方向

- [ ] 将 `串口 RX + payload 解析 + 数据整理` 迁移到 `QThread/worker`
  - worker 线程负责：
    - 串口读取
    - BLE 帧拼包
    - payload 解析
    - 速度 / 滤波 / 功率等计算
    - 可选的降采样或数组整理
  - GUI 主线程只负责：
    - 接收 signal
    - 更新控件
    - 调用 `setData(...)`
- 这会比“强行多线程绘图”更安全，也更符合 Qt 的推荐模式。

## 4. 后续可选优化（本轮未做）

- [ ] 给渲染帧率增加 GUI 可调开关（例如 15/24/30 FPS）
- [ ] 加一个“高画质模式”开关（开启 antialias）
- [ ] 将热路径缓冲升级为 numpy ring buffer，进一步减少转换开销
- [ ] 将串口 RX + payload 解析迁移到 `QThread/worker`
  - GUI 主线程只接收解析后的轻量结果并刷新控件。
  - 这是下一步最有价值的“真线程化”方向，但改动面比 CSV 写盘大，建议在当前版本稳定后再做。
- [ ] Load Replay CSV 改为后台线程解析
  - 解决大 CSV 载入时窗口短暂无响应的问题。
