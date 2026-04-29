# EG Debug Notes (Legacy A/B)

## Scope

This note records the EG timing-chain comparison and the reproducible A/B switches added in v4.x.

Target files:
- `All_in_one_hip_controller_RL_update/Controller_EG.cpp`
- `All_in_one_hip_controller_RL_update/BleProtocol.h`
- `GUI_RL_update/GUI.py`

---

## 1) Gait Frequency & Period

- Frequency estimator is hysteretic same-direction crossing (`<= -2°` arm, `>= +2°` trigger), i.e. one full-cycle interval per trigger.
- It is **not** half-cycle counting.

Current implementation:
- `estimateFreqFromRLTx(...)` in `All_in_one_hip_controller_RL_update.ino`

Period conversion:
- `gait_period_ms = 1000 / gait_freq`

Logging/status:
- Teensy SD CSV now logs both `gait_freq_Hz` and `gait_period_ms`.
- GUI CSV now logs both `gait_freq_Hz` and `gait_period_ms`.

---

## 2) Why old EG and new EG could differ

Before this patch, the main differences vs old archive EG were:

1. Delay scaling semantics:
- New chain: full-range `Assist_delay_dynamic = round(idx * 0.7 / gait_freq)` (low freq can enlarge delay).
- Old chain: only shrink when `gait_freq > 0.7`, plus a minimum of 5 samples.

2. Gate input:
- New chain used `x_pred`.
- Old archive behavior gated with `x_prev` path in practice.

3. LPF location:
- New chain used unified Teensy pre-motor LPF (for native algos), EG internal LPF removed.
- Old chain had EG internal LPF (`0.85/0.15`).

---

## 3) Legacy A/B Switch (one-click reproduction)

Protocol field:
- Downlink `payload[33] = eg_legacy_flags`:
  - `bit0`: legacy delay scaling (`f > 0.7` only shrink + min 5 samples)
  - `bit1`: gate input uses `x_prev`
  - `bit2`: enable EG internal LPF (`0.85/0.15`)

GUI control:
- EG panel checkbox: `Legacy EG Path`
- When checked, GUI sends `eg_legacy_flags = 0x07` (enable all three).

Firmware behavior:
- If `bit2=1`, EG internal LPF is enabled and Teensy unified pre-motor LPF is bypassed for EG to avoid double filtering.

---

## 4) A/B Validation Checklist

1. Baseline (current path):
- `Legacy EG Path = OFF`
- Expect: current EG chain (`x_pred`, current delay scaling, unified pre-motor LPF).

2. Legacy reproduction:
- `Legacy EG Path = ON`
- Expect:
  - old delay scaling rule
  - gate uses `x_prev`
  - EG internal LPF active
  - unified pre-motor LPF bypassed for EG

3. Compare logs:
- Teensy SD CSV: `walking_log_*.csv`
- GUI CSV: `GUI_RL_update/data/*.csv`
- Check `gait_freq_Hz`, `gait_period_ms`, `L/R_command_actuator`, `L/R_pwr_W`, and `+Ratio/+P/-P` overlay behavior.

