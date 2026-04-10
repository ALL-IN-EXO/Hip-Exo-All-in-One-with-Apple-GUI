# Active Peripheral 128

- Firmware: `High_speed_ble_prph_128data.ino`
- Role: Device bridge endpoint (BLE to Teensy UART)

## Critical Configuration

In the sketch, update:

- `Bluefruit.setName("...")` to match the central target expectation

## Data Path

- Input: BLE UART packets from central board
- Output: `Serial1` UART bytes to Teensy-side serial link
- Expected frame: header `A5 5A 80`, length `128`

## Validation Checklist

1. Confirm advertising starts and central can discover by name.
2. Confirm connection and notify enabled.
3. Confirm `Serial1` relay sends received frame bytes to Teensy path.
