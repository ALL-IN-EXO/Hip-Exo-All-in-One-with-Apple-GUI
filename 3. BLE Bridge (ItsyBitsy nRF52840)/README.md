# Module 3: BLE Bridge (ItsyBitsy nRF52840)

This module provides the BLE UART bridge between host serial and Teensy serial transport.

## Submodule Map

| Path | Purpose | Status | Audience |
|---|---|---|---|
| `active_central_128/` | Host-side central firmware (USB serial <-> BLE) | Active | Undergrad/Intern |
| `active_prph_128/` | Device-side peripheral firmware (BLE <-> UART) | Active | Undergrad/Intern |
| `legacy_central_30/` | Historical 30-byte central path | Legacy | Owner |
| `legacy_prph_30/` | Historical 30-byte peripheral path | Legacy | Owner |
| `archive_legacy_paths/` | Duplicate historical path variants | Legacy | Owner |

## Active Link Contract

- Framing header: `A5 5A 80`
- Frame length: `128` bytes
- Central and peripheral must use matching target name/device name

## First Bring-Up Order

1. Read `active_prph_128/README.md` and flash peripheral firmware.
2. Read `active_central_128/README.md` and flash central firmware.
3. Set matching BLE names.
4. Verify frame relay with known 128-byte test payload.
