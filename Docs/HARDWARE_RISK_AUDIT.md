# Hardware Risk Audit — Motor_Servo_Switch_EG_6_IMU.ino

> **审计日期:** 2026-03-09
> **状态:** 待系统级修复，不要单独修某个 bug（代码可能依赖某些 bug 运行）

---

## P0 — Critical（直接硬件风险）

### 1. CAN 双发：每条指令发两次

**位置:** `Sig_Motor_Control.cpp:20-31`

```cpp
void Motor_Control_Tmotor::send_CAN_message()
{
  Can3.write(msgW);          // ← 第1次发送（无条件）
  if (Can3.write(msgW))      // ← 第2次发送（又发了一次）
  { ... }
}
```

**影响:**
- CAN 总线负载翻倍（100Hz × 2电机 × 2次 = 400帧/s）
- 电机驱动器可能对重复帧有不可预测行为
- 增加总线冲突/仲裁失败概率

**⚠ 依赖可能性:** 如果电机驱动器忽略重复帧，当前系统可能"恰好正常"。需先在台架上验证修复后电机响应不变。

---

### 2. BLE 接收 payload 读了两次（数据被覆盖）

**位置:** `.ino:931-938`

```cpp
size_t n = Serial5.readBytes((char*)data_rs232_rx, PAYLOAD_LEN);  // 第1次读 ✅
if (n < PAYLOAD_LEN) {
  Serial.printf("[BLE] Payload incomplete...\n");
  return;
}
Serial5.readBytes((char*)data_rs232_rx, PAYLOAD_LEN);  // 第2次读 ❌ 覆盖！
```

**影响:**
- 第一次读的正确 payload 被第二次读覆盖
- 第二次读到的是下一帧残留或 buffer 里的随机字节
- 可能导致 `max_torque_cfg` 被解析为极大值 → 突然爆扭矩
- 可能导致方向位翻转 → 电机反转

**⚠ 依赖可能性:** 如果 BLE 发送频率远低于接收轮询频率，第二次 readBytes 可能总是超时返回 0 字节（不覆盖），系统"恰好"用的是第一次读的数据。但这只是运气，不可靠。

---

### ~~3. Serial7 冲突 — IMU 和 Serial_Com 抢同一串口~~ ✅ 已修复

> **[2026-03-10 已修复]** Serial_Com 是遗留模块（旧版上位机串口通信），功能已被 BLE(Serial5) + RPi(Serial8) 完全取代。`SerialData2[]` 数据无任何下游使用。已从 .ino 中移除全部引用（`#include`、实例、`INIT()`、`READ2()`），并删除 `Serial_Com.h` 和 `Serial_Com.cpp` 文件。Serial7 现在完全归左大腿 IMU 独占使用。

---

## P1 — High（逻辑错误，影响控制精度或稳定性）

### 4. gait_freq=0 时除零 → NaN/inf 传播

**位置:** `.ino:508`

```cpp
float T_gait_ms1 = 1000.0f / gait_freq;  // gait_freq 初始=0 → inf
```

**影响:**
- `ext_ms_L = ext_phase_frac_L * inf = inf`
- `lrintf(inf)` 行为未定义，可能返回 INT_MAX
- `extN_L/R` 可能越界访问 `flexL_hist[]` 数组 → 内存越界读

**修复方向:** 在 `gait_freq` 使用前加保护：`if (gait_freq < 0.1f) { skip extension logic }`

---

### 5. TMOTOR 反馈符号错误 — M2 用了 M1 的电流符号

**位置:** `.ino:903-904`

```cpp
float iq_meas_signed1 = copysignf(iq_meas_mag1, iq1);  // ✅ M1 用 iq1
float iq_meas_signed2 = copysignf(iq_meas_mag2, iq1);  // ❌ M2 也用 iq1，应该是 iq2
```

**影响:**
- M2 实测扭矩符号可能反了
- GUI 和日志中 M2 torque 反馈方向不对
- 如果下游有基于 M2_torque_meas 的闭环，方向可能错

**⚠ 注意:** 仅在 `MOTOR_BRAND == 1`（TMOTOR）时触发。当前代码 `MOTOR_BRAND = 0`（SIG），此 bug 不会被执行。

---

### 6. ext_i 整数溢出 → 负索引越界

**位置:** `.ino:554`

```cpp
ext_i++;  // int, 永远自增，从不取模
```

**影响:**
- 100Hz 下约 248 天后 int32 溢出变负数
- `ext_i % EXT_BUF_LEN` 对负数取模在 C++ 中返回负数
- `flexL_hist[负索引]` → 内存越界读写

**修复方向:** `ext_i = (ext_i + 1) % EXT_BUF_LEN`

---

## P2 — Medium（性能/鲁棒性）

### 7. reinit 函数阻塞主控制循环（1-2 秒）

**位置:** `.ino:321-335`

```cpp
if (imu_reinit_pending) {
  imu.REZERO_LR(600, 200);   // 内含 delay(600) + 200次采样×5ms = ~1.6s 阻塞
}
if (motor_reinit_pending) {
  reinit_Sig_motor();          // 内含多个 delay()，总计 ~0.5-1s
}
```

**影响:**
- 阻塞期间电机无新 CAN 帧
- 如果电机有 watchdog → 自动停转（安全）
- 如果电机无 watchdog → 保持最后扭矩指令输出（危险）
- 6 路 IMU 的 RX buffer 可能溢出丢数据

---

### 8. BLE readBytes 在控制循环内可阻塞 5ms

**位置:** `.ino:185, 931`

```cpp
Serial5.setTimeout(5);  // setup 中设置
// ...
size_t n = Serial5.readBytes((char*)data_rs232_rx, PAYLOAD_LEN);  // 控制循环内
```

**影响:**
- 10ms 控制周期中，如果 BLE 帧不完整，readBytes 阻塞最多 5ms
- 占掉半个控制周期，导致电机指令延迟
- 偶发抖动

---

### 9. 无硬件看门狗 (Watchdog)

**影响:**
- 如果主循环因 SD 卡写入卡死、Serial 阻塞或任何异常而挂起
- 电机保持最后扭矩指令无限期运行
- 无自动复位机制

---

## P3 — Low（代码质量/长期隐患）

### 10. SD 日志写入在 100Hz 控制循环内

**位置:** `.ino:606-650`

- 每个控制周期约 20 次 `logger.print()` 调用
- SD 卡写入延迟不确定（偶发几十 ms）
- 可能导致控制循环偶尔超时抖动

---


---

## 已修复

### [2026-03-09] L/R 扭矩数据链路全链反转 + TMOTOR copysignf bug

**问题：** Teensy BLE 发送中 L/R torque 与实际电机对应关系反了，GUI 画图时再次交叉使用 buffer "巧合修正"了图表，但数值标签始终是反的。

**根因：** `sig_m1` = 右电机、`sig_m2` = 左电机，但 `Transmit_ble_Data()` 中把 `sig_m1.torque` 命名为 `L_tau`，`M1_torque_command` 命名为 `L_cmd`。

**完整追踪表（修复前）：**

| 数据 | Teensy 内部 | BLE 发送名 | GUI 变量 | 图表 | 数值标签 |
|---|---|---|---|---|---|
| 左角度 | imu.LTx | L_ang | L_angle | Left Leg 图 ✅ | Lθ ✅ |
| 右角度 | imu.RTx | R_ang | R_angle | Right Leg 图 ✅ | Rθ ✅ |
| 右电机实测 | sig_m1.torque | **L**_tau ❌ | L_tau | Right Leg 图 ✅(双反) | **Lτ** ❌ |
| 左电机实测 | sig_m2.torque | **R**_tau ❌ | R_tau | Left Leg 图 ✅(双反) | **Rτ** ❌ |
| 右电机指令 | M1_cmd | **L**_cmd ❌ | L_tau_d | Right Leg 图 ✅(双反) | — |
| 左电机指令 | M2_cmd | **R**_cmd ❌ | R_tau_d(取反) | Left Leg 图 ⚠ | — |

**修复内容：**

1. `.ino Transmit_ble_Data()`：L_tau=sig_m2(左), R_tau=sig_m1(右), L_cmd=M2, R_cmd=M1
2. `.ino receive_motor_feedback()`：`copysignf(iq_meas_mag2, iq2)` 修正 M2 符号
3. `GUI.py` 绘图：torque/cmd buffer 与 angle buffer 统一用同侧数据，不再交叉
4. `GUI.py`：移除 `R_tau_d` 的无理由取反 (`-R_tau_d` → `R_tau_d`)

---

## 修复原则

> **⚠ 警告：本代码可能依赖某些 bug 运行。**
>
> 例如 CAN 双发可能在当前系统中被电机驱动器静默忽略，
> BLE 双读可能因为时序巧合总是读到空数据，
> Serial7 冲突可能因为 READ2 的 header 不匹配而"无害"。
>
> **建议修复策略：**
> 1. 先搭建台架测试环境（电机不带负载）
> 2. 逐个修复，每修一个跑完整测试
> 3. 对比修复前后的 SD 日志：角度、扭矩、步频是否一致
> 4. 特别关注 CAN 双发修复后电机响应是否变化
> 5. 系统级修复时统一处理，而非单独 patch
