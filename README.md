# BLE Controller to GPIO (ESP32-S3-Zero)

This project uses an ESP32-S3-Zero as a BLE gamepad bridge. It connects to a compatible BLE controller and maps controller button presses to GPIO outputs that behave like physical button presses on a target device (for example, Nintendo DS Lite test pads).

## Device Needed

- ESP32-S3-Zero board
- BLE-compatible controller (tested: Xbox Wireless Controller model 1914)
- Target device with active-low button pads (example: Nintendo DS Lite)
- Hook-up wire and soldering tools
- USB-C cable for power/programming

## What The Code Does

- Pairs with a BLE game controller using Bluepad32.
- Reads gamepad buttons, D-pad, and misc buttons (Start/Select/System).
- Drives mapped GPIO pins as open-drain active-low outputs:
  - Pressed: pin sinks to GND
  - Released: pin floats (high-impedance)
- Includes turbo mode:
  - Hold Start + Select for 3 seconds to toggle turbo on/off.
  - Turbo auto-fires A/B/X/Y/L1/R1 while those buttons are held.
- Uses onboard RGB LED status:
  - Red: no controller connected
  - Solid green: connected
  - Blinking green: turbo enabled

## Important Compatibility Notes

- ESP32-S3 supports BLE only (no Bluetooth Classic).
- Controllers that require Bluetooth Classic will not connect on this board.
- A shared ground between ESP32 and target device is mandatory.

## Pin Wiring (Attach Each GPIO To These Button Pads)

Connect ESP32 GND to target device GND first, then wire each GPIO to the corresponding button pad:

| Controller Input | ESP32 GPIO | Attach To Target Pad |
|---|---:|---|
| A | 1 | A button pad |
| B | 2 | B button pad |
| X | 3 | X button pad |
| Y | 4 | Y button pad |
| L1 | 5 | L shoulder pad |
| R1 | 6 | R shoulder pad |
| D-pad Up | 7 | D-pad Up pad |
| D-pad Down | 8 | D-pad Down pad |
| D-pad Left | 9 | D-pad Left pad |
| D-pad Right | 10 | D-pad Right pad |
| Select | 11 | Select pad |
| Start | 12 | Start pad |


## Electrical Notes

- Outputs are open-drain/active-low by design (safe for switch-pad style inputs).
- Optional 100-330 ohm series resistor per line is recommended as wiring protection.
- Verify target pad idle voltage is <= 3.3V before soldering.

## Arduino IDE Setup (Quick)

1. Add board manager URLs:
   - `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - `https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json`
2. Install `esp32_bluepad32 by Ricardo Quesada` from Boards Manager.
3. Select an ESP32S3 Dev Module board from the `(esp32_bluepad32)` group.
4. Set USB CDC On Boot to Enabled.
5. Upload sketch.

## Sketch File

- `PadBridge-S3.ino`
