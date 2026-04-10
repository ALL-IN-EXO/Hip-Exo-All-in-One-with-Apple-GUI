# Submodule 2.2: IMU Stream Validation

This submodule validates left/right IMU streaming and serial telemetry formatting.

## Entry Path

- Firmware folder: `im948_main/`
- Main sketch: `im948_main/im948_main.ino`

## What Is Validated

- IM948 command bring-up sequence (`Cmd_03`, `Cmd_12`, `Cmd_19` variants)
- Left/right IMU packet parsing
- Telemetry relay on Teensy `Serial5` for bridge testing

## Prerequisite

Complete TR100 pairing in submodule `2.1` first.
