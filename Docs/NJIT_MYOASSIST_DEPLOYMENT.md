# NJIT MyoAssist Integration Notes (RPi RL Path)

## 1) Source inspection summary (`Algorithm Reference/NJIT Reference/HipExoCode_Apr20.py`)

### 1.1 Control modes in the reference script

`run_mode()` supports 7 control modes:

- `0`: `NN_mode`
- `1`: `Spring_Mode`
- `2`: `Impedance_Mode`
- `3`: `Spline_Mode`
- `4`: `Biological_Mode`
- `5`: `DOFC_Mode`
- `6`: `Zero_Mode`

### 1.2 NN model options in `NN_mode`

Reference `NN_NAMES` contains 10 model names:

- `walk_scaled_vel`
- `walk_july1`
- `walk_july11`
- `walk_100hz_15msdelay`
- `squat_nov18`
- `s2s`
- `testing_model2`
- `myoassist_model`
- `uchida_bio4`
- `no_assistance`

`MODEL_FILE_MAPPING` points to multiple `.pt` checkpoints plus MyoAssist ZIP (`model_1966080.zip` / `model_2293760.zip` candidates) and Uchida `.pt`.

---

## 2) What is migrated now

### 2.1 In-scope (migrated)

Two NJIT MyoAssist checkpoints are integrated into `RPi_Unified` and can run via `--nn`:

- `--nn myoassist_1966080`
- `--nn myoassist_2293760`

They use the same runtime chain as existing RL algorithms:

`IMU input -> MyoAssist inference -> unified torque filter -> runtime delay -> runtime scale -> send AA59`

### 2.2 Out-of-scope for this change (listed only)

Not migrated in this patch:

- `walk_scaled_vel`
- `walk_july1`
- `walk_july11`
- `walk_100hz_15msdelay`
- `squat_nov18`
- `s2s`
- `testing_model2`
- `uchida_bio4`
- classical modes (`Spring/Impedance/Spline/Biological/DOFC/Zero`)

---

## 3) Input/output mapping and compatibility

## 3.1 Required input/output in this project

- Input: hip IMU angle/velocity from current RL stream (`imu_LTx`, `imu_RTx`, `imu_Lvel`, `imu_Rvel`)
- Output: left/right torque command (`tau_L`, `tau_R`)

## 3.2 NJIT MyoAssist inference logic kept identical

The migrated controller keeps the NJIT MyoAssist observation and mapping exactly:

- `current_input = [right_angle, left_angle, right_vel, left_vel]` (radians)
- observation dimension `18`:
  - input history: `2 x 4`
  - current input: `1 x 4`
  - output history: `3 x 2`
- symmetric actor swap indices:
  - `[1,0,3,2,5,4,7,6,9,8,11,10,13,12,15,14,17,16]`
- torque mapping:
  - `tau_R = output[0] * max_torque`
  - `tau_L = output[1] * max_torque`

## 3.3 Difference from project runtime (intentional)

- NJIT MyoAssist branch itself has no internal torque low-pass filter.
- In this project, we keep that behavior and apply the existing unified pre-send torque filter in `RL_controller_torch.py` main loop (same as other RL algorithms).

---

## 4) Deployment implementation

### 4.1 Added files / changes

- `RPi_Unified/networks/myoassist.py`
  - `ExoActorMyoAssist` + `MyoAssistController`
- `RPi_Unified/RL_controller_torch.py`
  - argparse `--nn` adds `myoassist_1966080` / `myoassist_2293760`
  - `load_network()` adds MyoAssist branches
  - `NN_TYPE_CODE` adds:
    - `5 -> myoassist_1966080`
    - `6 -> myoassist_2293760`
  - default runtime delay:
    - both MyoAssist types use `0 ms`
- `RPi_Unified/run.sh`
  - menu/direct-run adds both MyoAssist types
- `GUI_RL_update/GUI.py`
  - new remote start buttons:
    - `Start Myo-1966`
    - `Start Myo-2293`
  - RL panel mapping adds `nn_type=5/6`
  - MyoAssist staged defaults:
    - `scale=1.00`, `delay=0 ms`

### 4.2 Model lookup order

For each MyoAssist type, runtime looks for ZIP in this order:

1. `RPi_Unified/models/njit/<model>.zip`
2. `Algorithm Reference/NJIT Reference/<model>.zip`
3. `RPi_Unified/<model>.zip`
4. `RPi_Unified/models/<model>.zip`

Recommended deployment location on Pi:

- `RPi_Unified/models/njit/model_1966080.zip`
- `RPi_Unified/models/njit/model_2293760.zip`

---

## 5) Consistency evaluation (acceptance)

Goal: verify migrated logic matches NJIT reference logic on the same input sequence.

Dataset and window:

- CSV: `Algorithm Reference/NJIT Reference/PI5_lstm_pd-20260424-221451.csv`
- Time window: `50 s ~ 150 s`
- Inputs used only:
  - `imu_LTx`, `imu_RTx`, `imu_Lvel`, `imu_Rvel`

Tool:

- `tools/myoassist_consistency_eval.py`

Command:

```bash
python tools/myoassist_consistency_eval.py \
  --csv "Algorithm Reference/NJIT Reference/PI5_lstm_pd-20260424-221451.csv" \
  --t0 50 --t1 150 --max-torque 15.0 --out-dir Docs
```

Generated per-sample comparison files:

- `Docs/myoassist_consistency_model_1966080_50_150s.csv`
- `Docs/myoassist_consistency_model_2293760_50_150s.csv`

### 5.1 Results

For both checkpoints (`1966080` and `2293760`), migrated outputs are numerically identical to NJIT reference logic on the tested window:

- Left leg:
  - `corr = 1.000000`
  - `RMSE = 0.000000 Nm`
  - `best lag = 0.0 ms`
  - `amp_ratio = 1.000000`
- Right leg:
  - `corr = 1.000000`
  - `RMSE = 0.000000 Nm`
  - `best lag = 0.0 ms`
  - `amp_ratio = 1.000000`

Conclusion: time and amplitude are fully一致 on this acceptance dataset/window.

---

## 6) Runtime usage

On Pi:

```bash
source ~/venvs/pytorch-env/bin/activate
cd RPi_Unified
python RL_controller_torch.py --nn myoassist_1966080
# or
python RL_controller_torch.py --nn myoassist_2293760
```

From GUI (RL panel):

- `Start Myo-1966`
- `Start Myo-2293`

