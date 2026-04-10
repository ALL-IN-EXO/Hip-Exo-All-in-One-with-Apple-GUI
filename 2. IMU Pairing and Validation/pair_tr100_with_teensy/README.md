# Submodule 2.1: TR100 Pairing With Teensy

This folder contains the minimal TR100 pairing sketches.

## Which Sketch to Use

- Use `U7_pair/U7_pair.ino` when TR100 is wired to Teensy `Serial6`.
- Use `U8_pair/U8_pair.ino` when TR100 is wired to Teensy `Serial7`.

## What the Sketch Does

1. Opens USB serial for monitor output.
2. Opens TR100 UART at `460800`.
3. Sends fixed-length AT pairing command.
4. Optionally sets BLE name to `TR100`.
5. Prints module responses to USB serial.

## Safety / Usage Note

These sketches are for pairing and quick connectivity checks only. They are not runtime control firmware.
