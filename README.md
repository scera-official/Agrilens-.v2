# AgriLens 🌱

**Helping farmers make more informed decisions, one crop image at a time.**

AgriLens is a student-built agricultural assistance prototype that combines a camera, a small on-device display, and AI-based image analysis. The goal is simple: make it easier to get an initial idea of what might be affecting a crop, without needing to understand complicated technology.

A farmer or demonstrator can capture an image of a plant and ask the AI service to look for visible signs of disease, pest damage, nutrient-related symptoms, or other crop concerns. The device presents the response in a compact, farmer-friendly interface.

AgriLens is a decision-support prototype, not a replacement for an agricultural officer or a laboratory diagnosis. AI can be wrong, and important crop decisions should be checked with a trusted local agricultural expert.

## What it aims to do

- Capture crop images with an ESP32-S3 camera board.
- Show the device interface and results on an ST7789 TFT display.
- Request an AI-assisted assessment of visible crop symptoms.
- Offer clear, practical guidance in a format designed for field use.
- Support English and Tamil interface options.
- Keep the project open for learning, testing, and future improvements.

Features depend on the firmware version, connected hardware, network access, and a valid AI service configuration.

## Hardware in this build

- ESP32-S3 camera development board
- ST7789 TFT display (240 × 320)
- User button and camera flash LED
- Optional BMP280 environmental sensor support in the firmware

Please check the pin definitions in `AgriLens.ino` against your exact board revision before connecting hardware. ESP32-S3 camera boards do not all share the same pin layout.

## Repository layout

- `AgriLens.ino` — main Arduino firmware
- `docs/PROJECT_OVERVIEW.md` — project purpose and current scope
- `docs/HARDWARE.md` — hardware notes and pin mapping
- `docs/PRIVACY_AND_SECURITY.md` — credentials and responsible use
- `.gitignore` — files that should stay out of version control

## Libraries and services

The firmware uses the Arduino-ESP32 environment and libraries including ArduinoJson, Adafruit GFX, Adafruit ST7789, and JPEGDEC for the live camera preview. Crop-image analysis is sent to a configured Google Gemini API endpoint, so network access and a valid API key are required for that feature.

Library APIs and AI model availability can change over time. Treat this repository as a project in development and verify compatibility with your installed library versions.

## A note about accuracy

AgriLens can provide a useful first look, but a photo may not show the whole problem. Lighting, camera quality, crop variety, growth stage, and local conditions can affect the result. Do not apply pesticides, fertilisers, or other treatments based only on an AI response.

## Why we are building it

Farmers put time, skill, and care into every crop. AgriLens is our attempt to make useful technology feel less distant and more practical—especially for people who need clear information rather than complicated screens.

**AgriLens — See the problem. Understand the crop. Protect the harvest.**

## Project team

Developed by the AgriLens student team, Team Chinmaya.

Names and acknowledgements can be updated to reflect the final team and contributions.
