# Hip Exo All-in-One with Apple GUI

A multi-component control system for the hip exoskeleton, consisting of Teensy 4.1 firmware, a Raspberry Pi RL controller, and a macOS GUI. Supports runtime-switchable control algorithms (Energy Gate, Samsung, RL neural networks, Test mode) via BLE.

**Current Version: v3.3** (2026-04-15) | [Full Changelog](Docs/CHANGELOG.md)

**What's new in v3.3:** CSV replay controls were added to the GUI (`Load/Pause/Speed/<<5s/>>5s/Stop`) with column mapping support; RL auto-delay now supports `Grid` and `BO` runtime method switching via GUI passthrough; GUI plotting performance was improved for long-running sessions (decoupled ingest/render, replay time-budget processing, overlay/style update throttling).

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
├── scripts/                           # Git workflow + PyInstaller build scripts
│   ├── new_feature.sh                 # Pull main → create branch → commit → push
│   ├── push_current.sh                # Commit current branch → push
│   ├── cleanup_branch.sh             # After PR merge: switch to main, delete branch
│   ├── status.sh                      # Show full git status at a glance
│   ├── build_win.ps1                  # Package GUI.py into distributable .exe (Windows)
│   └── build_mac.sh                   # Package GUI.py into distributable .app (macOS)
│
├── tools/                             # RPi sync tools (code deploy + data pull)
│   ├── deploy_code.sh                 # Push RPi_Unified/ to Raspberry Pi
│   ├── pull_data_watch.sh             # Pull output/ from Raspberry Pi (watch mode)
│   ├── rpi_sync.py                    # Underlying sync script
│   ├── rpi_profiles.conf.example      # Pi connection profile template (copy → rpi_profiles.conf)
│   └── rpi_profiles.conf              # Your personal Pi profile (gitignored, not committed)
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

**First-time setup** — create your personal Pi connection profile (gitignored):

```bash
cp tools/rpi_profiles.conf.example tools/rpi_profiles.conf
# Edit rpi_profiles.conf: set [active] profile = <your-name>, fill in host/user/remote_dir/password
```

```bash
./tools/deploy_code.sh              # Push RPi_Unified/ to Pi (uses active profile)
./tools/pull_data_watch.sh          # Continuously pull output/ from Pi (every 2s)
./tools/pull_data_watch.sh 0        # Pull once

# Override profile at runtime:
python tools/rpi_sync.py --profile aboutberlin --direction push
```

### Packaging the GUI (distributable .exe / .app)

Package `GUI_RL_update/GUI.py` into a standalone executable so non-developers can run the GUI without installing Python.

**Windows:**

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\build_win.ps1
# Output: GUI_RL_update/release/windows/<timestamp_gitsha>/HipExoRLGUI_win_*.zip
```

**macOS:**

```bash
chmod +x ./scripts/build_mac.sh
./scripts/build_mac.sh
# Output: GUI_RL_update/release/macos/<timestamp_gitsha>/HipExoRLGUI_mac_*.zip
```

Distribute the `.zip` file. Common options (`-FullBuild` / `--full`, `-SkipDepInstall` / `--skip-deps`), full parameter list, troubleshooting, and first-launch notes for recipients (SmartScreen / Gatekeeper quarantine) are documented in [scripts/README.md](scripts/README.md).

### Auto Delay (Samsung / EG / RL)

v3.1 adds on-device delay optimization that maximizes positive power ratio in real time:

- **RL mode** (RPi): 1Hz side-loop scans ±100 ms in 10 ms steps; per-leg L/R independent; power metrics (`ratio` / `+P/s` / `-P/s`) always computed even when Auto Delay is OFF
- **Samsung / EG mode** (Teensy-native, no RPi needed): same algorithm via `AutoDelayOptimizer` C++ class; results reported in the same 40B BLE status slot
- Toggle via the **Auto Delay** checkbox in the GUI Samsung / EG / RL panels

## Documentation

| Document | Description |
|----------|-------------|
| [CHANGELOG](Docs/CHANGELOG.md) | Version history and release notes |
| [CONTRIBUTING](Docs/CONTRIBUTING.md) | How to collaborate: branching, PRs, code style |
| [SYSTEM_ARCHITECTURE](Docs/SYSTEM_ARCHITECTURE.md) | Full hardware + software architecture, protocols |
| [HARDWARE_RISK_AUDIT](Docs/HARDWARE_RISK_AUDIT.md) | Hardware safety audit |
| [REPO_STANDARD](Docs/REPO_STANDARD.md) | Cross-project repo creation standard |
