# Bug Analysis: RPi CSV vs GUI CSV Power-Ratio Discrepancy

**Date:** 2026-04-21
**Files in scope:**
- `PI5_lstm_pd-20260421-183238.csv` — RPi 100 Hz log (218 s)
- `20260421_131729.csv` — GUI 20 Hz log (wrap-corrected span ≈ 1152 s)
- `Pi1.jpg`, `Bug1.jpg`, `Bug2.jpg` — GUI screenshots

---

## 1. 关键时间轴修正 —— GUI CSV 的 `t_s` 会回绕

GUI CSV 里的 `t_s` 字段来源是 Teensy BLE 上行帧里的 `uint16` 厘秒戳（`BleProtocol.h` 上行帧 byte 3–4 `t_cs`）。它每 **655.36 s ≈ 10 分 55 秒** 回绕一次。

- 以为的跨度：503 s
- 真实跨度：**145.9 → 1298.0 s ≈ 1152 s（约 19 分钟）**
- 发生 1 次 wrap，位置在 row 10092

`GUI.py:_load_replay_samples`（`GUI.py:4322-4326`）已经做了 `dt += 655.36` 解包，因此 replay 面板上显示的 "1014 s / 1050 s" 这种时间戳是**解包后**的 GUI 时间，对应 `Bug1.jpg` / `Bug2.jpg` 看到的那一段。

---

## 2. 两份 CSV 对齐结果

用 **命令扭矩** 做互相关对齐最干净（角度因为周期性 + 未先解包时很容易误对）。解包并对齐后：

| 字段 | GUI − RPi 延迟 | 相关系数 |
|---|---|---|
| `L_cmd_Nm` ↔ `L_command_actuator` | **0 ms** | **0.999** |
| `R_cmd_Nm` ↔ `R_command_actuator` | −20 ms | 0.999 |
| `L_angle_deg` ↔ `imu_LTx` | −80 ms (GUI 领先) | 0.818 |
| `R_angle_deg` ↔ `imu_RTx` | −80 ms | 0.817 |
| `L_vel_dps` ↔ `imu_Lvel` | −60 ms | 0.757 |
| `R_vel_dps` ↔ `imu_Rvel` | −60 ms | 0.744 |

**最接近的一段**（GUI 解包时钟）：`[1047.74, 1265.74] s`，对应 **RPi 时钟** `[0, 218] s` —— 这就是两条记录物理重合的 218 秒窗口。

**幅值对比**（对齐窗口内）：

| | RPi rms | GUI rms | RPi peak | GUI peak |
|---|---|---|---|---|
| L cmd (Nm) | 8.57 | 8.48 | 17.38 | 15.00 |
| R cmd (Nm) | 8.08 | 8.00 | 16.67 | 15.00 |
| L angle (deg) | 22.58 | 22.48 | — | — |
| L vel (dps) | 121.5 | 119.2 | — | — |

GUI 峰值 15 Nm 是 Teensy 下发给电机前 clamp 的结果，其他差异在 1 % 以内。

**结论：原始数据本身是一致的**，并不是"两侧数据有差异"。

---

## 3. Bug 到底是什么

这里有两件被混在一起的事。

### Bug A — 同一行 (cmd, vel) 的时间对齐方式天然不同（物理现象）

用各自 CSV 的同行数据做 `P = cmd × ω` 的正功比例：

| 指标 | RPi CSV | GUI CSV |
|---|---|---|
| Left 正功比例 | **82.7 %** | **75.9 %** |
| Right 正功比例 | 82.8 % | 75.3 % |
| 平均瞬时功率 L / R | +15.1 / +14.5 W | +10.1 / +9.8 W |

**根源**：
- **RPi 每一行的 `(L_command_actuator, imu_Lvel)` 是因果配对**：`imu_Lvel` 是 NN 的输入，`L_command_actuator` 是 NN 的输出，同一帧、同一时刻写入。算出来就是 82 % 正功，这正是 LSTM-PD 的设计目标。
- **GUI 每一行的 `(L_cmd_Nm, L_vel_dps)` 不是因果配对**：两者来自同一个 Teensy BLE 上行帧，但：
  - `L_vel_dps` = Teensy 刚采的 IMU 速度（新鲜）
  - `L_cmd_Nm` = Teensy 最近从 Serial8 收到的 RPi 指令（几十毫秒前、基于更老的 vel 算出来的）
  - 两者之间隔着整条 **RPi → Serial8 → Teensy 缓冲 → BLE 20 Hz 打包 → BLE radio → GUI** 回路。

**延迟扫描**（把 RPi cmd 人为延迟 τ，再与 RPi vel 相乘）：

```
shift +  0 ms   L +% = 76.1 %    ← 和 GUI 实测接近
shift +100 ms   L +% = 64.1 %
shift +150 ms   L +% = 54.1 %
shift +200 ms   L +% = 47.4 %    ← 和 Pi1.jpg 的 46.5 % / 57.5 % 相符
shift +300 ms   L +% = 32.9 %    ← 负功开始占优
shift +500 ms   L +% = 23.8 %
```

0.65 Hz 步态下（周期 1.54 s），100–200 ms 延迟就是 23–47° 相位移，足以把 `(cmd · ω)` 的符号翻过来。

**"RPi → Teensy 回路有延迟"这个判断方向是对的**，但延迟源不是 Serial8：
- Serial8 115200 baud 传 26 B 只要 ~2 ms；实测 cmd 到 GUI 的净延迟 ≈ 0 ms（corr 0.999, 0 ms）。
- **主要延迟来自 Teensy BLE 上行 20 Hz 打包窗口（最差 50 ms）+ 两块 ItsyBitsy 的 BLE radio（几十 ms 级）**。
- 结果：GUI 看到的 cmd 是"过去的指令"；把它和同一帧里"现在的 vel" 相乘，正负功比例就会偏离 RPi 的本意。

### Bug B — GUI replay 显示 bug（Bug1 / Bug2 里 `0.0 % | OFF/HOLD` 的真正原因）

`Bug1.jpg` / `Bug2.jpg` 里 `+Ratio 0.0 % | OFF/HOLD`、`+P 0.00 W  −P 0.00 W` —— 这些文本**不是 GUI 从 CSV 当场算出来的**，而是从以下成员变量读取：

```python
self._rpi_power_ratio_L / _R          # GUI.py:678-679
self._rpi_pos_per_s_L / _R            # GUI.py:680-681
self._rpi_neg_per_s_L / _R            # GUI.py:682-683
self._rpi_auto_motion_valid_L / _R    # GUI.py:676-677
```

这些变量**只在 BLE 上行帧解析器里被写入**（`GUI.py:5395-5454`，解析 `rpi_blob[58..97]` 这 40 B `RPi passthru` 状态帧）。

在 **replay CSV 模式下**：
- CSV 里根本没有那 40 B 状态帧的内容
- 没有任何本地计算去填充这些变量
- 所以它们一直是 `_reset_rpi_status()` 的默认值：`ratio = 0.0, motion_valid = False`
- 面板就恒显 `0.0 % | OFF/HOLD`

**Pi1.jpg 显示 46.5 % / 57.5 %** 的合理解释是：那张截图是**现场实跑时截的**（或者 GUI 仍在和 RPi 连着接收实时状态帧），所以 `_rpi_power_ratio_L / _R` 有值。*需要确认三张图的捕获时 GUI 是否 `Connected` / 是否开着 replay。*

---

## 4. 干净的 Bug 描述（给人看的版本）

> **现象**：同一次实验存成两份 CSV —— RPi 100 Hz `PI5_lstm_pd-*.csv` 和 GUI 20 Hz `20260421_131729.csv`。对齐后逐点对比，角度 / 速度 / 命令扭矩几乎完全一致（cmd 相关性 0.999）；但在 GUI 面板上 "Power Sign Ratio" 文字显示的正功比例却和 RPi 侧差异明显：
>
> - `Pi1.jpg`（实跑，RPi 直送状态帧）：+Ratio ≈ **46–58 %**，OFF/VALID
> - `Bug1/Bug2.jpg`（replay GUI CSV）：+Ratio = **0.0 %**，OFF/HOLD，+P/−P 恒为 0
>
> **排查**：
> 1. GUI CSV 的 `t_s` 是 Teensy `uint16` 厘秒戳，每 655.36 s 回绕。本次实验发生了 1 次 wrap（row 10092），解包后真实跨度 1152 s。对齐前必须先解包。
> 2. 解包 + cmd 互相关对齐后：两份 CSV 上的角度 / 速度 / 命令**原始数据一致**（cmd corr=0.999, 0 ms offset）。
> 3. 即便如此，用各自 CSV 同行 `(cmd × vel)` 算出来的正功比例：RPi = 82.7 %，GUI = 75.9 %。差别源于：**RPi 每一行 `(cmd, vel)` 是因果配对**（vel 是 NN 输入，cmd 是 NN 输出），**GUI 每一行 `(cmd, vel)` 不是因果配对** —— cmd 是 RPi 几十毫秒前基于更老 vel 发来的，vel 是 Teensy 现采的。回路延迟（RPi→Serial8→Teensy 缓冲→20 Hz BLE 上行→BLE radio→GUI）把 cmd 相对同一帧的 vel 后推 ~50–200 ms，在 0.65 Hz 步态下就是 23–47° 相位，足以翻号。延迟扫描在 +200 ms 处复现了 `Pi1.jpg` 的 46 % / 57 %。
> 4. `Bug1` / `Bug2` 的 `0.0 % | OFF/HOLD` 还叠加了一个 GUI 显示 bug：`+Ratio` 文本读自 `self._rpi_power_ratio_L` 等成员，这些成员仅在 BLE 状态帧解析器 (`GUI.py:5395` 起) 里被写入。replay 路径下这些字段在 CSV 里不存在、也没有任何本地计算去填充，所以 replay 期间恒为 0 / `HOLD`。
>
> **结论**：
> - **数据侧没有 bug**：两份 CSV 物理上一致；"Teensy 读到的和 RPi 发的不一样"并不成立；cmd 到达延迟的主要来源是 Teensy 的 BLE 20 Hz 打包窗口 + BLE radio，不是 Serial8。
> - **显示侧有 bug**：replay 模式下 GUI 面板上的 `+Ratio` 文本和 `+P / −P` 数值不会基于 CSV 重算，一直显示 0 / HOLD，容易让人误以为"回放和实跑对不上"。

---

## 5. 建议的下一步（等确认后再动手）

1. **让 replay 显示和实跑一致**：在 `_apply_replay_sample` / `_consume_replay_samples` 里基于窗口内的 `(L_cmd_Nm, L_vel_dps)` 自己算一次 pos / neg / ratio，填进 `_rpi_power_ratio_L` 等变量，把 `_rpi_status_valid` 设为 `True`。这样 `Pi1` 和 `Bug1` 的 `+Ratio` 就会都变成基于 CSV 的可比数字。
2. **让 GUI 显示的功率符号反映 RPi 的实际控制意图**：GUI 端算 `(cmd × vel)` 时用 `vel_gui(t − Δ)` 做相位补偿（Δ 可由 `torque_delay_ms_L/R` 或现场估计得到）。
3. **真正吃掉 Teensy 那 ~50 ms 打包延迟**：
   - 方案 A：Teensy 收到 Serial8 RL 指令后立刻触发一次 BLE 上行（event-driven），而不是等到 20 Hz 定时器。
   - 方案 B：GUI 端按 cmd 到达时刻而不是 BLE TX 时刻打时间戳。

---

## 6. 参考数据位置（便于下次复现）

- 对齐脚本 & 输出（本次分析用）：`/tmp/align_v3.py`、`/tmp/aligned_v3.csv`
- GUI 功率条源码：`GUI_RL_update/GUI.py:4846-4929` (`_update_power_strip`, `_update_power_strip_titles`)
- GUI replay 核心：`GUI_RL_update/GUI.py:4305-4349` (`_load_replay_samples`)、`4584-4592` (`_apply_replay_sample`)
- RPi 状态帧解析：`GUI_RL_update/GUI.py:5395-5454`
- BLE 协议总览：`Docs/SYSTEM_ARCHITECTURE.md` §6 / §8 / §10
