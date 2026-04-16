# RL Auto Delay Optimization Design

## 1. 目的与范围

本文档描述 `RPi_Unified/RL_controller_torch.py` 中 RL 自动 Delay 调参功能的设计思路与实现细节，目标是：

- 在不改网络结构的前提下，通过运行时 `delay_ms` 优化助力时序
- 优先提升正功占比（`power_ratio`）
- 在满足 `power_ratio >= 0.95` 后，继续提升单位时间正功（`pos_per_s`）
- 只在“有效运动”状态下调参，避免静止/噪声导致误调
- **左右腿独立优化**：auto 打开后 L/R 各自扫描、各自 dwell、各自 best delay，支持非对称 gait

本功能仅作用于 RL 通道（Teensy↔RPi↔GUI 的 RL 透传路径）。

---

## 2. 关键结论（先说结论）

1. `delay` 不直接改变扭矩幅值，只改变扭矩与关节速度的相位关系。  
2. 在固定 `runtime_scale` 下，`tau` 序列幅值本身不变，但 `P = tau * omega` 的正负分布会变化。  
3. 因此，调 `delay` 可以显著改变 `power_ratio` 与 `pos_per_s`。  
4. 仅最大化 `power_ratio` 有“假优解”风险（例如总体功率很小也能比值很高），所以采用双目标：
   - 主目标：`ratio >= 0.95`
   - 次目标：在满足主目标后最大化 `pos_per_s`

---

## 3. 实时功率指标定义

对每条腿在窗口内计算：

- 即时功率：`power(t) = tau(t) * vel(t) * pi/180`（`vel` 单位 deg/s，转 rad/s）
- 正功：`W+ = sum(power > 0) / CTRL_HZ`
- 负功绝对值：`|W-| = sum(abs(power < 0)) / CTRL_HZ`
- 正功占比：`ratio = W+ / (W+ + |W-|)`
- 每秒正功：`pos_per_s = W+ / T_total`
- 每秒负功：`neg_per_s = -|W-| / T_total`

左右腿独立评估与独立优化。GUI 的 overlay 仍展示双腿汇总（仅用于人类阅读）：

- `ratio_avg = 0.5 * (ratio_L + ratio_R)`
- `pos_per_s_total = pos_per_s_L + pos_per_s_R`
- `neg_per_s_total = neg_per_s_L + neg_per_s_R`

对应代码：

- `compute_leg_power_metrics(...)` — 单腿功率指标
- `evaluate_delay_candidate_leg(...)` — 单腿候选 delay 评估（取代早期的双腿合成版本）
- `_auto_step_leg(...)` — 单腿自动 delay 状态机（main 内嵌闭包，按 L/R 各调一次）

---

## 4. 窗口策略（按步态周期自适应）

为了兼顾稳定性与响应速度，窗口长度自适应：

1. 从左右髋角历史估计步频（自相关法）：`estimate_gait_freq_hz(...)`
2. 按 `AUTO_WINDOW_CYCLES / gait_freq_hz` 得到目标窗口
3. 限制在 `[AUTO_WINDOW_MIN_S, AUTO_WINDOW_MAX_S]`（当前默认锁定 8~8s）
4. 若步频不可用，回退到 `AUTO_WINDOW_FALLBACK_S`（默认 8s）

优点：

- 走慢时窗口自动更长，抗噪更好
- 走快时窗口自动变短，响应更快

---

## 5. 有效运动门控（只在有效运动时调参）

每次评估对 L/R 分别判定 `auto_motion_valid_L/R`：

- `mean_abs_vel >= AUTO_VALID_MIN_MEAN_ABS_VEL_DPS`
- `abs_power_per_s >= AUTO_VALID_MIN_ABS_POWER_W`

只有该侧 `motion_valid == True` 才允许改该侧 `delay`。  
评估统一使用 `scale = 1.0`（ratio / 候选排序对 `runtime_scale` 不变），回传给 GUI 的 `pos_per_s / neg_per_s` 再乘 `runtime_scale` 保留真实功率语义。  
这可避免静止、摆腿幅度过小、信号噪声主导、以及 GUI 调小 scale 导致 `abs_power_per_s` 永远不过阈值的错误门控。

速度源（用于功率/门控评估）支持硬开关（不走 GUI）：

- `AUTO_PWR_USE_ANGLE_DIFF_VEL = True`（默认）：用 `d(hip_angle)/dt`（deg/s）
- `False`：回退用 IMU 原始角速度 `LTAVx/RTAVx`

`d(angle)/dt` 实现为左右腿独立前值状态，并带回绕保护（`AUTO_PWR_ANGLE_WRAP_DEG`，默认 ±180°），用于避免角度跨界导致的差分尖峰。  
该速度源仅用于 auto-delay 的 `hist_vel`（功率评估与 motion-valid gate）；NN 推理输入速度链路不变。

---

## 6. Delay 优化策略

执行节奏：

- 控制主环保持 100Hz（推理与下发不变）
- 自动调参在 1Hz 侧环执行（`AUTO_UPDATE_INTERVAL_S = 1.0`）

每次 1Hz 评估流程（对 L/R 各独立执行一次）：

1. 计算本侧“当前 delay”指标（当前 `ratio/pos_per_s/...`）
2. 若本侧运动有效且满足 `dwell` 时间，扫描候选 delay：
   - 区间：`current_delay_side ± 100ms`
   - 步长：`10ms`
3. 对每个候选 delay 用**本侧** `tau/vel` 复算窗口指标
4. 选优规则 `pick_best_delay_candidate(...)`：
   - 优先候选集 `ratio >= 0.95`
   - 在该集合内最大化 `pos_per_s`
   - 若无人满足 0.95，则回退到全局最大 `ratio`
5. 为防抖与突变：
   - 每次实际改变量限制为 `<= 40ms`
   - 变化后重置本侧 dwell 计时（`auto_last_change_ts_L/R` 各自维护）

Dwell 策略由 `AUTO_DWELL_ADAPTIVE` 开关控制（默认 `False`）：
- `False`：固定 `AUTO_DWELL_S`（当前默认 0.5s）
- `True`：`max(AUTO_DWELL_S, auto_window_s + 1.0)`，等窗口 100% 由新 cfg 下的数据填满

冷启动：在 cfg 把 `auto_delay_enable` 由 False→True 上升沿时，`auto_last_change_ts_L/R` 被重置为当前时间，首轮扫描需等满 dwell；同时 `hist_tau_src/vel/ang` 三条历史缓冲被清空，避免评估窗口混入切换前的旧 scale / delay / filter 数据。

非 auto 模式：`runtime_delay_ms_L = runtime_delay_ms_R = cfg['delay_ms']`，两腿跟随 GUI 单值；auto 一旦打开，两腿从同一基准起独立漂移。

这是一种“局部扫描 + 约束步进”的在线优化，L/R 各自一份轻量状态机，不阻塞 100Hz 控制链路。

---

## 6.1 BO 版本（可选，保持同一安全壳）

在 `grid` 之外，新增 `bo`（Bayesian Optimization）模式，仍只优化单变量 `delay_ms`（每腿独立）：

- 观测目标：
  - `J = pos_per_s - λ·max(0, 0.95-ratio)^2 - μ·|neg_per_s|`
- BO 模型：
  - 1D Gaussian Process（RBF kernel）
  - 采集函数：`UCB`（默认）或 `EI`
- 执行节奏：
  - 仍是 1Hz 侧环，每次窗口得到一次噪声观测 `J(x)`
  - BO 仅决定“下一次建议 delay”，实际写入仍走 `max step` 与 `dwell`
- **重要**：
  - `motion_valid`、`dwell`、`delay bounds`、`AUTO_MAX_STEP_MS` 完全复用原安全壳
  - 100Hz 主控制链路不变，BO 只在慢速监督层运行

建议论文表述：BO 的收敛/低 regret 论述限定在“准稳态窗口近似平稳”条件下，不宣称全局无条件保证。

---

## 7. 通信与 GUI 交互

### 7.1 GUI -> RPi（40B passthrough）

- `payload[20]` bit0: `auto_delay_enable`
- `payload[20]` bit3: `auto_method_bo`（0=grid，1=bo）

### 7.2 RPi -> GUI（AA56 状态 40B）

新增字段：

- `[20]` bit0: auto enable, bit1/2: motion valid L/R, bit3: method bo
- `[24..27]` `power_ratio` (float32)
- `[28..31]` `pos_per_s` (float32)
- `[32..35]` `neg_per_s` (float32)
- `[36..39]` `best_delay_ms` (float32)

GUI RL 面板增加：

- `Auto Delay` 开关
- `Auto Method` 下拉（`Grid (Legacy)` / `Bayes (BO)`）
- 实时状态行：`ratio / +P/s / -P/s / best delay / motion valid`

---

## 8. 为什么这个方案可行

1. 物理上可解释：目标直接来自 `tau` 与 `omega` 的功率耦合。  
2. 计算量可控：1Hz 扫描约 21 个候选（±100ms, 10ms step），在 Pi5 上负担很小。  
3. 工程上安全：只在有效运动调参 + dwell + 限步长，不会高频抖动。  
4. 可观测：GUI 可实时看到当前指标和建议 delay，便于人工监督。

---

## 9. 参数建议（当前默认）

- `AUTO_TARGET_RATIO = 0.95`
- `AUTO_UPDATE_INTERVAL_S = 1.0`
- `AUTO_DWELL_S = 0.5`
- `AUTO_SCAN_HALF_RANGE_MS = 100.0`
- `AUTO_SCAN_STEP_MS = 10.0`
- `AUTO_MAX_STEP_MS = 40.0`
- 窗口：`8~8s`（fixed 8s）

如需更稳（精度优先）：

- 适当增大 `AUTO_DWELL_S`（例如 0.8~1.5s）
- 保持窗口较长（例如 fixed 8s）

如需更快（响应优先）：

- 缩小 dwell（例如 0.1~0.3s）
- 但会增加误调概率（尤其短窗口 + 短 dwell 组合）

---

## 10. 常见使用误区

### Auto Delay OFF 时 Power Ratio 是否仍然计算？

**是的，始终计算。**

`auto_delay_enable = False` 只关闭"自动调参（apply new delay）"，不关闭"实时功率指标计算"。

- RPi 侧：1Hz 侧环的触发条件不含 `auto_delay_enable`；`_auto_step_leg` 内部的 evaluate + scan 始终运行，仅 `apply step` 块（候选扫描并写入 `runtime_delay_ms`）被 `if auto_delay_enable` 守护。
- Teensy ADO 侧：`tick()` 函数不再在 `!enabled` 时提前返回；`scan_leg()` 始终运行以更新 `cur_ratio_L/R` 等，仅 `delay_ms = new_delay` 赋值行被 `if (enabled && ...)` 守护。

结果：GUI 的 Power Sign overlay 中 `+Ratio%` / `+P / -P` 数值在 Auto Delay OFF 时仍实时刷新，可用于人工监督当前助力质量，无需打开 Auto Delay 才能看到数字。

---

## 11. 已知边界与后续增强

当前边界：

- 仅优化 delay，不自动改 `runtime_scale`
- 指标基于近窗口统计，不是长期全局最优
- 门控阈值目前在代码常量中配置

可选增强：

1. 将门控阈值和调参参数暴露到 GUI
2. 增加“连续 N 次有效窗口才允许调参”
3. 增加滞回区：若 `ratio` 在 0.95 附近，减少来回切换
4. 将左右腿目标分开，支持非对称 gait

---

## 11. 代码定位

- 自动调参与指标核心：`RPi_Unified/RL_controller_torch.py`
- GUI 开关与显示：`GUI_RL_update/GUI.py`
- 协议透传：`All_in_one_hip_controller_RL_update/Controller_RL.h/.cpp`
