# Hardware notes

This document describes the intended hardware for the current AgriLens firmware. Confirm every connection against your own board before powering it on.

## Main parts

| Part | Purpose |
|---|---|
| ESP32-S3 camera board | Runs the firmware and captures crop images |
| ST7789 TFT, 240 × 320 | Displays the interface and analysis results |
| User button | Navigates or confirms actions in the interface |
| Flash LED | Helps illuminate the subject when needed |
| BMP280 (optional) | Environmental sensor support is present in the firmware |

## Pin mapping in the firmware

### ST7789 display

| Signal | ESP32-S3 GPIO |
|---|---:|
| CS | 21 |
| DC | 47 |
| RESET | 48 |
| MOSI | 1 |
| SCLK | 2 |
| Backlight | 3 |

### Controls and I²C

| Signal | ESP32-S3 GPIO |
|---|---:|
| User button | 19 |
| Flash LED | 14 |
| I²C SDA | 41 |
| I²C SCL | 42 |
| BMP280 address | `0x76` |

### Camera

The camera pin map is defined near the top of `AgriLens.ino`. Use that section as the source of truth for this firmware version.

## Important

ESP32-S3 camera boards vary significantly. Do not assume these pins match a different board just because it also uses an ESP32-S3. Check the board's schematic and confirm the display, camera, button, flash, and sensor wiring before use.

The BMP280 is optional in the project description. Do not rely on environmental readings unless the sensor is connected and the firmware has been tested with it.
