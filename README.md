# Hip Exo All-in-One with Apple GUI

A multi-component control system for the hip exoskeleton, consisting of Teensy 4.1 firmware, a Raspberry Pi RL controller, and a macOS GUI. Supports runtime-switchable control algorithms (Energy Gate, Samsung, RL neural networks, Test mode) via BLE.

**Current Version: v3.0** (2026-03-20) | [Full Changelog](Docs/CHANGELOG.md)

## System Overview

```
macOS GUI (GUI_RL_update/)
    │  BLE 128-byte frames (via ItsyBitsy nRF52840)
    ▼
Teensy 4.1 (All_in_one_hip_controller_RL_update/)
    │  Serial8 115200 baud
    ▼
Raspberry Pi 5 (RPi_Unified/)  ──  RL neural network inference
```

## Project Structure

```
Hip-Exo-All-in-One-with-Apple-GUI/
├── README.md                          # This file
├── CLAUDE.md                          # AI assistant guide (auto-loaded by Claude Code)
├── .gitignore
│
├── All_in_one_hip_controller_RL_update/   # Teensy 4.1 firmware (Arduino)
│   ├── All_in_one_hip_controller_RL_update.ino  # Main entry point
│   ├── Controller.h / Controller_EG / _Samsung / _RL / _Test
│   ├── MotorDriver.h                  # SIG / T-Motor runtime switch
│   ├── BleProtocol.h                  # 128-byte BLE frame packing/parsing
│   ├── IMU_Adapter.h / .cpp           # 6-IMU unified interface
│   ├── Motor_Control_Sig / _Tmotor    # CAN motor drivers
│   └── sdlogger.h / .cpp              # SD card logging
│
├── RPi_Unified/                       # Raspberry Pi 5 RL controller (Python)
│   ├── RL_controller_torch.py         # Single entry point (--nn to select network)
│   ├── filter_library.py              # IIR / EMA / Kalman filters
│   ├── networks/                      # Network class definitions (DNN, LSTM, LegDcp, PD)
│   ├── models/                        # Trained model weights (.pt)
│   │   ├── dnn/                       # DNN weights
│   │   └── lstm/end2end/              # LSTM weights (by motion type)
│   └── output/                        # Runtime CSV logs (gitignored)
│
├── GUI_RL_update/                     # macOS GUI (Python / PyQt5)
│   └── GUI.py                         # Single entry point
│
├── Docs/                              # All documentation
│   ├── CHANGELOG.md                   # Version history
│   ├── CONTRIBUTING.md                # Collaboration guide (branching, PR, code style)
│   ├── SYSTEM_ARCHITECTURE.md        # Full system architecture reference
│   ├── HARDWARE_RISK_AUDIT.md        # Hardware safety audit
│   └── REPO_STANDARD.md              # Repo creation standard (cross-project)
│
├── scripts/                           # Git workflow shell scripts
│   ├── new_feature.sh                 # Pull main → create branch → commit → push
│   ├── push_current.sh                # Commit current branch → push
│   ├── cleanup_branch.sh             # After PR merge: switch to main, delete branch
│   └── status.sh                      # Show full git status at a glance
│
├── tools/                             # RPi sync tools (code deploy + data pull)
│   ├── deploy_code.sh                 # Push RPi_Unified/ to Raspberry Pi
│   ├── pull_data_watch.sh             # Pull output/ from Raspberry Pi (watch mode)
│   └── rpi_sync.py                    # Underlying sync script
│
└── data_pi/                           # Data pulled from RPi (gitignored)
```

## Quick Start

### GUI (macOS)

```bash
pip install pyqt5 pyserial matplotlib numpy
cd GUI_RL_update
python GUI.py
```

### Teensy Firmware

Open `All_in_one_hip_controller_RL_update/All_in_one_hip_controller_RL_update.ino` in Arduino IDE and upload to Teensy 4.1.

### Raspberry Pi RL Controller

```bash
source ~/venvs/pytorch-env/bin/activate
cd RPi_Unified
python RL_controller_torch.py --nn dnn         # DNN (default)
python RL_controller_torch.py --nn lstm        # LSTM
python RL_controller_torch.py --nn lstm_leg_dcp
python RL_controller_torch.py --nn lstm_pd
```

### Deploy Code to RPi / Pull Data

```bash
./tools/deploy_code.sh           # Push RPi_Unified/ to Pi
./tools/pull_data_watch.sh       # Continuously pull output/ from Pi (every 2s)
./tools/pull_data_watch.sh 0     # Pull once
```

## Documentation

| Document | Description |
|----------|-------------|
| [CHANGELOG](Docs/CHANGELOG.md) | Version history and release notes |
| [CONTRIBUTING](Docs/CONTRIBUTING.md) | How to collaborate: branching, PRs, code style |
| [SYSTEM_ARCHITECTURE](Docs/SYSTEM_ARCHITECTURE.md) | Full hardware + software architecture, protocols |
| [HARDWARE_RISK_AUDIT](Docs/HARDWARE_RISK_AUDIT.md) | Hardware safety audit |
| [REPO_STANDARD](Docs/REPO_STANDARD.md) | Cross-project repo creation standard |
