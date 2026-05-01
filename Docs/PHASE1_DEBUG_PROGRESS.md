# Phase 1 Debug — 三个 Bug 的实测追踪记录

> 目的：把 Phase 1 加 debug 打印 + 上机三个 test 的结果完整记下来，方便接手与回看。
> 此文件由 AI 协同维护；改动控制逻辑前必须先在这里留痕。

---

## 状态总览（2026-05-01）

| Bug | 用户描述 | 实测结果 | 根因 | 修复状态 |
|---|---|---|---|---|
| 1 | Apply Algorithm 要点 3 次才生效 | 已复现，定位到 BLE 链路 | **BLE 下行帧大量丢失，特别是 GUI 短时间内连发两帧时第 2 帧几乎必丢** | ✅✅ **已修 + 上机复测通过 (2026-05-01)** |
| 2 | Power OFF/ON 之后力矩不恢复 | 已复现，状态混乱 | 与 Bug 1 同源（BLE 帧丢） + GUI 没等 Teensy ACK 就允许下次切换 | ✅✅ **已修 + 上机复测通过 (2026-05-01)** |
| 3 | RL 参数不下发 | "RPi console 没有任何信号" | **RPi cfg 解析路径完整正确，但全程没有 print → 用户看不见**（不是真正的功能 bug） | ✅✅ **加可视化打印后上机复测通过 (2026-05-01)** — cfg 包路径全程畅通 |

---

## 1. 改动范围（Phase 1）

仅加 debug 打印，**没有改任何控制逻辑**。所有打印都用 `DBG_PHASE1` / `if DBG_PHASE1` 包起，关掉就零开销。

- Teensy: `Controller_RL.h` 加宏开关；`.ino` 在 `switch_algorithm`、`Receive_ble_Data` 每帧、10Hz 状态、`RL_RX` 边沿处加 print + 帧/字节计数器。
- GUI: `GUI.py` 加 `DBG_PHASE1` 开关 + `_dbg_log()` helper（自动落盘到 `GUI_RL_update/debug_logs/phase1_<时间戳>.log`）。打点位置：`_on_algo_confirm`、`_on_power_toggled`、`_on_apply_rl_clicked`、`_tx_params` 写串口前、上行 active_algo 变化、上行 ACK。
- `.gitignore` 加 `GUI_RL_update/debug_logs/`。

---

## 2. 测试日志原始文件

| Test | 状态 | GUI log | Teensy log |
|---|---|---|---|
| 1-failed | 4 单击都失败 | `GUI_RL_update/debug_logs/phase1_20260501_014132.log` | `teensy_phasea.log` |
| 1-success | 3 连击才切成功 | `GUI_RL_update/debug_logs/phase1_20260501_014343.log` | `teensy_phasea_1.log` |
| 2-power chaos | 状态混乱 | `GUI_RL_update/debug_logs/phase1_20260501_014536.log` | `teensy_phaseb.log` |
| 3-RL params | RPi 无反应 | `GUI_RL_update/debug_logs/phase1_20260501_015146.log` | `teensy_phasec.log` |

RPi 运行时 CSV：`PI5_lstm_pd-20260501-065153.csv`（用户提供）。

---

## 3. Test 1 失败 / 成功 对比 — Bug 1 锁定

### 3.1 Test 1 失败：4 次单击 → 0 次切成功

**GUI 发送（按时间序）：**

| T | 事件 | 发送内容 |
|---|---|---|
| 01:41:32.742 | 启动初始 _tx_params | `algo_byte=0` (单帧) |
| 01:41:43.676 | Click 1 EG→Sam | LOGTAG + `algo_byte=1` (双帧) |
| 01:41:48.873 | Click 2 Sam→RL | LOGTAG + `algo_byte=2` (双帧) |
| 01:41:53.020 | Click 3 RL→SOGI | LOGTAG + `algo_byte=4` (双帧) |
| 01:41:57.676 | Click 4 SOGI→Test | LOGTAG + `algo_byte=3` (双帧) |

**Teensy 实际收到：**

```
ble[algo=1 lg=4 B=1035]
[BLE_RX #1] algo=0 maxT=0.00 ...
```

- **算法帧 1/5（仅初始那次到了）**
- **LOGTAG 帧 4/4（全到）**
- 0 次 `switch_algorithm` 调用

**关键观察**：
- 凡是用户点 Apply Algorithm 触发的算法帧，**全部丢失**（0/4）
- 凡是用户点击带的 LOGTAG 帧，**全部到达**（4/4）

LOGTAG 和算法帧的发送间隔只有约 1ms（同一 click 内 `_send_logtag_text()` 后立即 `_tx_params()`，两次 `self.ser.write(128B)`）。**第 1 帧到，第 2 帧丢**——是非常规整的"成对发送时第二帧必丢"模式。

### 3.2 Test 1 成功：每次切都点 3 下

逐次（EG → Sam）click 时序：

```
T+0     click 1 select=0→1, algo_changed=True → 发 LOGTAG+algo_byte=1（双帧）
T+463ms click 2 select=1, pending=1, algo_changed=False → 仅发 algo_byte=1（单帧）
T+1047ms click 3 select=1, pending=1, algo_changed=False → 仅发 algo_byte=1（单帧）
T+1173ms uplink 报告 active_algo: 0→1（成功切到 Sam）
```

6 次切换全部走完这个 3 连击模式才成功。Teensy 总共收到 **11 个算法帧**（GUI 发了约 18 个），**11 次 `switch_algorithm` 调用** 全部完成。

### 3.3 推论

- LOGTAG 帧的成功率 ≫ 紧跟其后的算法帧成功率
- 单击模式下（每帧间隔数秒），算法帧（被 LOGTAG 抢先）几乎必丢
- 3 连击模式下，第 1 击的算法帧仍然丢（因为 LOGTAG 抢先），第 2/3 击 `algo_changed=False` 不再发 LOGTAG，只发单算法帧，能挤过去

**结论：BLE 链路上，"短时间内的第 2 个 128B 写"会被丢。** 整条链 `GUI → pyserial → BLE bridge (ItsyBitsy nRF52840) → Teensy Serial5` 上至少有一个环节做不了背靠背的 128B 写。可能机制：

- BLE bridge 的发送队列满 → 第 2 帧被丢
- BLE 协议层 MTU 限制（默认 23B）+ 30ms connection interval，128B 需要 ~210ms 才能发完，第 2 个 128B 还没排上队就被覆盖
- Teensy Serial5 RX 缓冲只有 64B（`Controller_RL.cpp:8-12` 注释提到 Serial8 缓冲被扩到 512B，但 Serial5 没扩；不过这条机制只会引起字节级丢失，不会"整帧丢"）

---

## 4. Test 2 — Power OFF/ON 混乱

GUI 端 Power 切换路径打印一切正常：

- Power ON：`maxT_raw=1500 power_on=True`
- Power OFF：`maxT_raw=0 power_on=False`
- `_maxT_before_off` 始终保持 15.00（从未被破坏）

Teensy 端总共收到 **38 个算法帧 + 14 个 LOGTAG**。从 `BLE_RX #22` 起的 maxT 序列：

```
#22..#29: maxT=0  (Power OFF 系列)
#30..#34: maxT=15 (Power ON 系列)
#35..#36: maxT=0
#37..#38: maxT=15
```

GUI 期间有大量 Power ON/OFF 短时间连点（150-200ms 一次）。每个 toggle 是一个单 `_tx_params`（不带 LOGTAG），但当用户连点时也会出现"两个相邻 toggle 都在 200ms 内"的情况——这跟 Test 1 的 LOGTAG+算法 双帧丢失同一机制：**前一个到，后一个被丢**。

**Bug 2 的"混乱"本质 = Bug 1 同一根因**：
- 用户点 Power ON，发 1 帧；
- 短时间内点 Power OFF，发 1 帧——可能丢；
- 用户点 Power ON，又发一帧——可能丢；
- GUI 这边 button checked 状态自洽，Teensy 那边 max_torque_cfg 跟丢失模式相关**有可能不同步**；
- 用户视觉上看到按钮状态和实际力矩对不上，于是疯狂连点，进一步加剧丢帧。

GUI 没有"等 Teensy ACK"机制（Power 没有 ACK 路径，max_torque_cfg 只在 10Hz 上行 `mt100` 字段里反映），所以 GUI 完全无从知道 Power 切换实际有没有生效。

---

## 5. Test 3 — RL 参数下发

GUI 端 9 次 `_tx_params` 全部带 `rpi_data=1, rpi_head=524c0101`（合法 RL 透传 magic）。Teensy 端**全部 9 帧都收到了**（`ble[algo=9 lg=8 B=3382]`），并且每次都通过 `forward_rpi_passthrough_to_pi()` 经 Serial8 转发给 RPi。

但用户报告："树莓派的 console 数据没有，但是我很确定的事没有任何收到信号的提示"。

可能原因（待 RPi 端日志验证）：

1. RPi 端 Serial8 接收侧根本没在监听这种 `0xA5 0x5A | 41 | 0x02 | <40B>` 类型的下行包（RPi 主程序只处理 `0xAA 0x55/56/59` 上行格式，`0xA5 0x5A` 下行格式可能没解析路径）。
2. 或者 RPi 解析路径存在但没打印，所以"console 没反应"。
3. 或者 Serial8 的 RX 在 RPi 那侧根本没启用 read。

**结论**：Bug 3 不是 BLE/Teensy 问题，是 **Serial8 (Teensy → RPi) 下行解析的问题**。需要看 `RPi_Unified/RL_controller_torch.py` 和相关 networks 模块如何（或是否）处理 type=0x02 下行包。Phase 1 的 Teensy 打印对这个 bug 没帮助。

---

## 6. 修复方案（待用户确认后执行）

### 6.1 Bug 1 + Bug 2：BLE 链路抗丢帧

**两层都修，独立有效**：

#### 修复 A（GUI 侧，最简单也最确定）
在 `GUI.py` 的 `self.ser.write()` 上加节流，强制相邻两次写之间至少 50ms。可以在 `_tx_params` 和 `_send_logtag_text` 内通过单一 `_throttled_write()` helper 统一管理。
- 优点：纯 Python、立即生效、不需要重 upload Teensy
- 代价：每次 click 多 50ms 延迟（用户感知不到）

#### 修复 B（Teensy 侧）
扩展 Serial5 RX 缓冲到 512B，照搬 `Controller_RL.cpp:12` 的写法。即使 GUI 没修，Teensy 也能对抗主循环抖动。
- 优点：让 Serial5 像 Serial8 一样能容忍 ~45ms 抖动
- 代价：需要重 upload 一次

**建议：A + B 一起做**。A 解决"GUI 发太快"，B 解决"Teensy 来不及读"。两者都低风险。

#### 修复 C（GUI 侧，可选）
Power 切换加 ACK 检测：上行帧的 `mt100` 字段反映 Teensy 实际 `max_torque_cfg`。GUI Power ON/OFF 后等到上行的 `maxT_rx` 与期望值匹配再清"待确认"状态；不匹配就重发 1-2 次。这能让 Power 行为变得可预测。

### 6.2 Bug 3：RL 参数转发到 RPi

需要更多数据：
1. 在 RPi 上抓 Serial8 RX 字节流（任意 hex dump）
2. 或在 `RL_controller_torch.py` 的 Serial8 read 路径加打印

确认 RPi 那侧根本没在解析 `0xA5 0x5A` 下行包之后，再加 RPi 端解析逻辑 + 应用 RL 参数。

---

## 8. 复测用例 + 成功判定

### 准备
1. **重 upload Teensy**（含 Fix B 的 Serial5 缓冲扩展 + 之前的 Phase 1 打印）
2. **重启 GUI**（含 Fix A 的 throttled BLE 写）
3. 抓 USB 日志（Teensy 一侧）：
   ```bash
   tio -t -l teensy_phase1_post.log /dev/cu.usbmodem<TAB>
   ```
4. GUI 日志会自动落到 `GUI_RL_update/debug_logs/phase1_<时间戳>.log`

### Test A — Apply Algorithm 单击就生效（Bug 1 复测）

**操作（不要 Power ON）：**
1. GUI 启动并连接 Teensy，等 5 秒基线稳定
2. 点 segment Samsung → 点 Apply Algorithm **一次** → 等 1.5 秒
3. 点 segment RL → 点 Apply Algorithm **一次** → 等 1.5 秒
4. 点 segment EG → 点 Apply Algorithm **一次** → 等 1.5 秒
5. 点 segment SOGI → 点 Apply Algorithm **一次** → 等 1.5 秒
6. 点 segment Test → 点 Apply Algorithm **一次** → 等 1.5 秒

**成功指标（必须全部满足）：**
- ✅ 每次单击之后，**~150ms 内**应在 GUI 日志看到 `[uplink ACK] active_algo=<X> matches pending; clearing waiting_ack`
- ✅ Teensy 日志每次都看到对应的 `[ALGO] ENTER switch_algorithm: from_id=A to_id=B` + `[ALGO] EXIT switch_algorithm: active_id=B`
- ✅ Teensy `BLE_RX #N` 算法帧计数随每次 click 严格 +1（5 次 click → 至少 +5）
- ✅ GUI 日志看不到任何 "click had no effect" 或 algo_pending != algo_select 持续超过 1 秒
- ✅ 每次 Apply Algorithm 按钮在 ~150ms 内变回灰色 disabled 状态（不再要求点 3 次）

**失败指标（如出现需汇报）：**
- ❌ 单击之后 button 持续橙色 / 持续 dirty 超过 2 秒
- ❌ Teensy 端 `BLE_RX` 计数没有相应增加
- ❌ Teensy 端 `[ALGO] ENTER` 没有触发（说明帧到了但 algo_select 没变）

### Test B — Power OFF/ON 力矩恢复（Bug 2 复测）

**操作（戴好设备）：**
1. 选 EG，点 Apply Algorithm 一次（用 Test A 验证过的方式）
2. Power ON → 走 5 步 → 应有助力
3. Power OFF（**不要连点**，等至少 2 秒）→ 站着不动 5 秒
4. Power ON → 走 5 步 → 应该恢复助力
5. 重复 step 3-4 共 3 个 OFF/ON 循环

**成功指标：**
- ✅ 每次 Power ON/OFF 单击都触发一次 GUI `[_on_power_toggled ENTER/EXIT]`，且 `_tx_params` 实际写出对应的 `maxT_raw=1500` 或 `maxT_raw=0`
- ✅ Teensy 端 10Hz 状态行 `maxT=` 字段在 ~150ms 内反映期望值（OFF→0，ON→15）
- ✅ Power ON 后，戴上设备走路时 `L_cmd` / `R_cmd` 非零，电机有响应
- ✅ 3 次 OFF/ON 循环全部正常恢复

**失败指标：**
- ❌ Power ON 后 Teensy `maxT=15` 但 `L_cmd=0` / `R_cmd=0` 持续——说明算法有问题（不是 BLE 问题）
- ❌ Power OFF 之后 Teensy `maxT=0` 但电机仍在转——说明输出阶段 bypass 了 clamp
- ❌ GUI/Teensy 状态错位（GUI 显示 ON 但 Teensy 收到的最新 maxT 是 0）

### Test C — RL 参数下发到 RPi（Bug 3 取证）

**操作：**
1. 启动 GUI + RPi（确保 RL_controller_torch.py 在跑）
2. **抓 RPi console 输出到文件**：在跑 `RL_controller_torch.py` 的终端用 `script` 或者重定向：
   ```bash
   # 方法 1: 重定向 stdout/stderr
   python RL_controller_torch.py --nn dnn 2>&1 | tee rpi_phase1.log
   
   # 方法 2: 已经在跑了, 用 script
   script -q rpi_phase1.log
   # 然后 attach 到现有 process / 重新跑
   ```
3. GUI 切到 RL（Apply Algorithm 后等 ACK），确认右上角 RPi 在线
4. 修改 RL Cutoff Hz 从 3.0 → 8.0 → 点 Apply RL Settings → 等 2 秒
5. 修改 RL Cutoff Hz 8.0 → 5.0 → Apply → 等 2 秒
6. 切 RL Auto Delay 一次 → Apply → 等 2 秒
7. 修改 Scale 0.4 → 0.6 → Apply → 等 2 秒

**成功指标（Bug 3 修复 = 看到任何一个）：**
- ✅ RPi 日志看到 `[RPi DBG read_packet] CFG #N len=41 payload_head=524c0101...`
- ✅ RPi 日志看到 `[RPi DBG parse_cfg] ACCEPT scale=...` + `[RPi DBG cfg APPLIED] NN_TYPE=...`

**取证指标（仅诊断, 不一定坏）：**
- 1Hz 心跳 `[RPi DBG hb] in_waiting=N imu=A cfg=B...`：
  - `imu=` 计数应一直在涨（说明 Serial8 RX 链路活着）
  - `cfg=` 计数应该在每次 Apply RL 之后 +1
  - `bad_cksum` / `bad_len` 偶发可接受，连续涨说明链路损坏
  - `parse_reject` 涨且非零说明 cfg 到了 RPi 但被 `parse_runtime_cfg` 拒了（看具体 reject 原因 print）

**失败指标：**
- ❌ `imu=` 一直 0 → RPi 根本没在读 Serial8（serial port 问题）
- ❌ `imu=` 在涨但 `cfg=` 一直 0，且每次 GUI Apply RL 之后 Teensy 那边 `[BLE_RX]` 都 `rpi_data=1`、forward 也调用了 → Serial8 GUI→Teensy→RPi 这一段下行被阻断（Teensy 写过去但 RPi 收不到，可能是 Teensy 的 `forward_rpi_passthrough_to_pi` 没触发，或 Teensy Serial8 TX 配置有误）
- ❌ `cfg=` 涨但 `parse_reject` 同步涨 → 字节内容损坏

---

## 7. 进度日志

| 日期 | 事件 |
|---|---|
| 2026-05-01 早 | 用户上机做完 3 个 test，提供日志 |
| 2026-05-01 | AI 完成日志分析，锁定 Bug 1/2 同根因 = BLE 帧丢失 |
| 2026-05-01 | 用户批准 Fix A + Fix B；AI 完成实现：<br>- `GUI.py`：加 `_throttled_ble_write()` 模块级 helper，节流间隔 50ms，覆盖两处 `ser.write()`<br>- `.ino`：`setup()` 里 `Serial5.addMemoryForRead(BLE_SERIAL_RX_EXTRA, 512)`<br>- 详见 CHANGELOG `[Unreleased]` 段 |
| 2026-05-01 | 等待用户重新 upload Teensy + 重启 GUI 后跑复测（Test 1/2，详见 §8） |
| 2026-05-01 | RPi 端补 Phase 1 打印（Bug 3 取证，无控制逻辑改动）：<br>- `RL_controller_torch.py` 加 `DBG_PHASE1_RPI` 开关 + 一组帧/字节计数器<br>- `read_packet` 在 cfg 帧到达时打印 `[RPi DBG read_packet] CFG #N`，bad_len/bad_cksum 也会打印<br>- `parse_runtime_cfg` accept/reject 都加打印（包括失败原因）<br>- 主循环 cfg 分支加 `[RPi DBG cfg APPLIED]`<br>- 主循环加 1Hz 心跳 `[RPi DBG hb]` 报 `in_waiting / imu帧数 / cfg帧数 / bad_*`<br>- 关于代码层面的 Bug 3 静态分析：RPi cfg 解析路径完整，magic/version/cmd 都对得上，**但整条 cfg 应用过程没有任何 print** —— "用户没看到任何提示"很可能只是这个原因，需要复测验证 |
| 2026-05-01 | Test 复测用例与成功判定写在 §8 |
| 2026-05-01 | **Test A + Test B 上机复测通过** ✅。用户反馈："taska 和 b 解决"。Bug 1 / Bug 2 修复确认有效。 |
| 2026-05-01 | Test C 启动遇到 RPi `UnboundLocalError: cannot access local variable '_DBG_RPI_LAST_HEARTBEAT_TS'` —— 我在 main() 里给模块级变量赋值漏写 `global` 声明。已修：在 `main()` 函数顶部加 `global _DBG_RPI_LAST_HEARTBEAT_TS`。`py_compile` 通过。 |
| 2026-05-01 | **Test C 上机复测通过** ✅。RPi cfg 包路径全程畅通——之前的 "RPi 没反应" 完全是因为 RPi 端 cfg 处理流程没有任何 print 语句。三个 Bug 全部修复完毕。 |
| 2026-05-01 | **Phase 1 全部收尾**：`Docs/CHANGELOG.md` 切到 `[v5.2] - 2026-05-01` 段，`[Unreleased]` 留空给下一轮。Phase 1 工作正式结束。 |
