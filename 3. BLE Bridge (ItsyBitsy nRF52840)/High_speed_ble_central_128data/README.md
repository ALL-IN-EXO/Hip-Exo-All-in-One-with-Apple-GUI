# Active Central 128

- Firmware: `High_speed_ble_central_128data.ino`
- Role: Host bridge endpoint (USB serial on central board)

## Critical Configuration

In the sketch, update:

- `kTargetName` to the peripheral device name you want to connect

## Data Path

- Input: USB serial bytes from host PC
- Output: BLE UART packets to peripheral board
- Expected frame: header `A5 5A 80`, length `128`

## Validation Checklist

1. Open Serial Monitor and confirm device scan output.
2. Confirm target name match and connection event.
3. Send a valid 128-byte frame from host and confirm relay statistics update.
