# Module 2: IMU Pairing and Validation

This is the first hands-on hardware module for new members.

## Submodule Map

| Path | Purpose | Status | Audience |
|---|---|---|---|
| `pair_tr100_with_teensy/` | Pair TR100 link module with Teensy UART | Active | Undergrad |
| `imu_stream_validation/` | Verify dual-IMU stream and 128-byte bridge payload path | Active | Undergrad/Intern |
| `archive_sdcard_legacy/` | Old SD-card logging prototype | Legacy | Owner |

## Code Review Snapshot

| File | Function | Notes |
|---|---|---|
| `pair_tr100_with_teensy/U7_pair/U7_pair.ino` | TR100 pairing on `Serial6` | Sends fixed 30-byte AT commands |
| `pair_tr100_with_teensy/U8_pair/U8_pair.ino` | TR100 pairing on `Serial7` | Same logic as U7, different UART |
| `imu_stream_validation/im948_main/im948_main.ino` | Left/right IMU parse + Serial5 telemetry | Uses IM948 command library + fixed bridge frame |

## Minimal Bring-Up Sequence

1. Read `pair_tr100_with_teensy/README.md`.
2. Flash either `U7_pair.ino` or `U8_pair.ino` based on wiring.
3. Confirm pairing responses in Serial Monitor.
4. Read `imu_stream_validation/README.md` and flash `im948_main/im948_main.ino`.
5. Confirm left/right angle and velocity values update stably.

## Success Criteria

- Pairing responds reliably after power cycle.
- IMU stream is continuous without persistent garbage frames.
- Left/right channel mapping is documented in lab notes.
