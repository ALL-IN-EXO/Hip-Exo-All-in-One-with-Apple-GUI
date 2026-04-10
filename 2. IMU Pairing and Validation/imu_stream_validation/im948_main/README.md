# IM948 Main Validation Sketch

Files in this folder:

- `im948_main.ino`: main loop, left/right parsing, relay payload transmission
- `im948_CMD.h`: IM948 command definitions and serial mapping macros
- `im948_CMD.ino`: IM948 packet parser and command implementation

## Current Serial Mapping

- Left IMU: `Serial7`
- Right IMU: `Serial6`
- Telemetry relay: `Serial5` (to BLE bridge path)

## Practical Validation Steps

1. Flash `im948_main.ino`.
2. Confirm startup command sequence runs without repeated errors.
3. Verify `AngleXLeft/AngleXRight` and `VelXLeft/VelXRight` update continuously.
4. Confirm relay bytes are present on `Serial5` when GUI/bridge path is connected.

## Notes

- This is a validation sketch, not the full integrated exoskeleton runtime firmware.
- Keep this path simple for onboarding; move experimental variants into archive folders.
