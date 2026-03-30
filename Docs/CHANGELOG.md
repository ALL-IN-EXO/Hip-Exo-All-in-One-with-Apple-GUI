# Changelog

All notable changes to this project will be documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

（开发中的变更暂时写在这里）

---

## [v3.0] - 2026-03-20

### Added
- **Multi-Algorithm Architecture**: Runtime-switchable control algorithms via GUI without recompilation
  - `Controller` abstract base class (`Controller.h`) with `compute()`, `parse_params()`, `reset()` interface
  - `Controller_EG`: Energy Gate algorithm (~12 parameters)
  - `Controller_Samsung`: Samsung algorithm (kappa, delay)
  - `Controller_RL`: RL neural network — Serial8 passthrough to RPi
  - `Controller_Test`: Fixed/sine torque test mode
- **MotorDriver abstraction layer** (`MotorDriver.h`): SIG (ODrive) / T-Motor (VESC) runtime switch via GUI
- **RPi_Unified**: Unified RL controller, single entry point for all 4 network types
  - `dnn`: DNN feedforward (18→128→64→2)
  - `lstm`: LSTMNetwork (4→256→2)
  - `lstm_leg_dcp`: Per-leg decoupled LSTM (2→256→1 × 2)
  - `lstm_pd`: LSTM-PD (2→256→1 × 2, position error control)
- **GUI v4.0**: RL panel with RPi online/offline status, dynamic parameter panel per algorithm, algo select + confirm mode, dual direction control (motor vs plot), Dark/Light theme

### Changed
- BLE frame expanded to **128 bytes** (up from previous size), with RPi passthrough zone (40B each direction)
- Serial8 protocol: IMU frame (TYPE=0x01, 32B) + GUI passthrough frame (TYPE=0x02, 41B)
- Project restructured into `All_in_one_hip_controller_RL_update/`, `RPi_Unified/`, `GUI_RL_update/`

---

## [v2.x] - Archive

Refer to `Archive_Old_Reference/` and `Deployment/` for prior versions:
- `All_in_one_hip_controller/`: Single-algorithm Teensy firmware (no runtime switching)
- `GUI/`: GUI without RL panel
- `Deployment/RL_controller_torch.py`: Single LSTM-PD deployment script (verified working)
