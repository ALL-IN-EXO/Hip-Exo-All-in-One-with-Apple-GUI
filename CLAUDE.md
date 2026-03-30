# CLAUDE.md - AI Assistant Guide

This file is automatically loaded by Claude Code at the start of every conversation.
It tells the AI where to find things and how to work in this project.

## Project Overview

Hip Exo All-in-One with Apple GUI: a multi-component control system for the hip exoskeleton.
Three active subsystems: Teensy 4.1 firmware, Raspberry Pi 5 RL controller, macOS PyQt5 GUI.
Communication: BLE 128-byte frames (GUI ↔ Teensy) + Serial8 115200 baud (Teensy ↔ RPi).

## Project Structure

```
All_in_one_hip_controller_RL_update/   # Teensy firmware (Arduino .ino + .h/.cpp)
RPi_Unified/                           # RPi RL controller (Python, entry: RL_controller_torch.py)
  ├── networks/                        # Network class definitions
  ├── models/                          # Trained .pt weights
  └── output/                          # Runtime CSV logs (gitignored)
GUI_RL_update/GUI.py                   # macOS GUI entry point
Docs/                                  # All documentation (see table below)
scripts/                               # Git workflow shell scripts
tools/                                 # RPi sync scripts (deploy + data pull, NOT git)
data_pi/                               # Local data pulled from RPi (gitignored)
```

## Key Documentation — Read These When Relevant

| When you need to... | Read this |
|---------------------|-----------|
| Understand hardware, serial protocols, BLE frame format | `Docs/SYSTEM_ARCHITECTURE.md` |
| Check hardware safety constraints | `Docs/HARDWARE_RISK_AUDIT.md` |
| Know Git workflow, branching, PR process | `Docs/CONTRIBUTING.md` |
| Check version history and what changed | `Docs/CHANGELOG.md` |

## Development Rules

1. **Never push directly to `main`** — always use a feature branch + PR
2. **One branch per task** — name it `feature/xxx`, `fix/xxx`, `docs/xxx`, or `refactor/xxx`
3. **After making code changes**, update `Docs/CHANGELOG.md` under `[Unreleased]`
4. **Teensy, RPi, and GUI are separate subsystems** — changes in one do not require changes in the others unless the protocol changes (see `Docs/SYSTEM_ARCHITECTURE.md` §6–10)
5. **BLE protocol changes** require coordinated updates to both `BleProtocol.h` (Teensy) and `GUI.py` (GUI) — document in SYSTEM_ARCHITECTURE.md
6. **Serial8 protocol changes** require coordinated updates to `Controller_RL.cpp` (Teensy) and `RL_controller_torch.py` (RPi)
7. **New network types** go in `RPi_Unified/networks/` as a separate file, then register in `RL_controller_torch.py`
8. **Never hardcode absolute paths** — `tools/` scripts use `${BASH_SOURCE[0]}` relative paths; keep this pattern

## Common Commands

```bash
# Run GUI
cd GUI_RL_update && python GUI.py

# Run RPi controller (on Pi)
source ~/venvs/pytorch-env/bin/activate
cd RPi_Unified && python RL_controller_torch.py --nn dnn

# Deploy RPi code from Mac
./tools/deploy_code.sh

# Pull data from RPi
./tools/pull_data_watch.sh        # continuous (every 2s)
./tools/pull_data_watch.sh 0      # once
```

## Code Conventions

- **Python**: 3.8+, snake_case functions, CamelCase classes; PyQt5 for GUI, PyTorch for networks
- **Arduino/C++**: follow existing style in `.ino` / `.h` / `.cpp` files
- **New controller algorithms**: extend `Controller` base class in `Controller.h`
- **New network types**: extend `Network` base class in `RPi_Unified/networks/base_network.py`
- **Output files**: always write to `RPi_Unified/output/` on Pi; `data_pi/` on Mac (both gitignored)
