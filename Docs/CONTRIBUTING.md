# Contributing Guide

## Git Workflow

### Branch Strategy

- **`main`**: Stable, working version. Never push directly.
- **`feature/<name>`**: New features (e.g., `feature/add-emg-support`)
- **`fix/<name>`**: Bug fixes (e.g., `fix/ble-reconnect-crash`)
- **`docs/<name>`**: Documentation updates
- **`refactor/<name>`**: Code refactoring

### Workflow

1. Pull latest `main`
2. Create a feature/fix branch
3. Make changes, test locally
4. Push branch and open a Pull Request on GitHub
5. At least one reviewer approves before merging
6. Squash merge into `main`

### Commit Messages

Use clear, descriptive messages:

```
add: runtime algorithm switching via GUI
fix: BLE reconnect crash after motor reinit
update: expand RPi passthrough zone to 40 bytes
docs: add Serial8 protocol spec to SYSTEM_ARCHITECTURE
refactor: consolidate motor CAN init into MotorDriver
```

type 取值: `add` / `fix` / `update` / `refactor` / `docs` / `remove`

---

## Code Organization

```
All_in_one_hip_controller_RL_update/   # Teensy firmware
  ├── *.ino                            # Main loop
  ├── Controller.h                     # Algorithm abstract base class
  ├── Controller_EG / _Samsung / _RL / _Test   # Algorithm implementations
  ├── MotorDriver.h                    # Motor brand abstraction
  ├── BleProtocol.h                    # BLE 128-byte frame format
  ├── IMU_Adapter.h / .cpp             # 6-IMU interface
  └── sdlogger / Motor_Control_*       # Logging + motor drivers

RPi_Unified/                           # RPi RL controller
  ├── RL_controller_torch.py           # Single entry point
  ├── filter_library.py                # Shared filters
  ├── networks/                        # One file per network class
  └── models/                          # Trained .pt weights

GUI_RL_update/GUI.py                   # macOS GUI (single file)
```

### Adding a New Control Algorithm (Teensy)

1. Create `Controller_YourAlgo.h` and `.cpp` in `All_in_one_hip_controller_RL_update/`
2. Extend `Controller` base class and implement `compute()`, `parse_params()`, `reset()`, `name()`, `id()`
3. Add static instance and register in main `.ino` (search for `active_ctrl`)
4. Allocate parameter bytes in `BleProtocol.h` downlink params area [5..57]
5. Add GUI panel in `GUI_RL_update/GUI.py` (search for `QStackedWidget`)
6. Document in `Docs/SYSTEM_ARCHITECTURE.md`

### Adding a New Network Type (RPi)

1. Create `RPi_Unified/networks/your_network.py` extending `Network` base class
2. Import and register in `RL_controller_torch.py` (add to `--nn` choices and dispatch)
3. Train and place model weights in `RPi_Unified/models/your_network/`
4. Add `NN_TYPE_CODE` constant and update RPi status uplink if needed
5. Update GUI RL panel if network has new tunable parameters
6. Document in `Docs/SYSTEM_ARCHITECTURE.md` §10

### Protocol Changes (BLE or Serial8)

Protocol changes affect multiple subsystems simultaneously. Required steps:

- **BLE frame change**: update `BleProtocol.h` (Teensy) + `GUI.py` (GUI) + `Docs/SYSTEM_ARCHITECTURE.md` §6
- **Serial8 frame change**: update `Controller_RL.cpp` (Teensy) + `RL_controller_torch.py` (RPi) + `Docs/SYSTEM_ARCHITECTURE.md` §10.6

---

## Testing

Before submitting a PR, verify:

1. **GUI**: App launches, connects to Teensy, all algorithm panels render correctly
2. **Teensy**: Firmware compiles cleanly; all 4 algorithms switch without crash; motor brand switch works
3. **RPi**: `python RL_controller_torch.py --nn dnn` runs without error; check CSV output in `output/`
4. **Protocol**: If BLE or Serial8 changed, test full roundtrip: GUI → Teensy → RPi → Teensy → GUI
5. **Safety**: Confirm angle limit (80°) safety cutoff still active; never disable without hardware present

---

## Updating Documentation

When you add or change functionality:

1. Update `Docs/CHANGELOG.md` under `[Unreleased]`
2. If the BLE or Serial8 protocol changed, update `Docs/SYSTEM_ARCHITECTURE.md`
3. If you added a new algorithm or network, add to SYSTEM_ARCHITECTURE.md §4–10
4. Keep `README.md` Quick Start in sync if commands change
