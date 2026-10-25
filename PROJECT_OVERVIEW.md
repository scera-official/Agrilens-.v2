# Project overview

## The problem we care about

Crop problems are not always easy to recognise early. A farmer may notice a change in a leaf or fruit but still need help understanding what it could mean and what to check next. Access to timely advice can also vary by location and language.

## The AgriLens approach

AgriLens combines a camera-based embedded device with AI-assisted image analysis. The device captures a crop image, sends it to the configured AI service when a network connection is available, and displays the returned assessment in a compact interface.

The intention is to make an initial assessment easier to access—not to claim that a photograph alone can confirm every crop disease.

## Design priorities

- **Practicality:** a compact prototype that can be demonstrated outside a conventional computer setup.
- **Clarity:** results should be readable and understandable.
- **Accessibility:** Tamil and English interface support can help make the experience more approachable.
- **Responsible use:** results should explain possibilities, not present uncertain AI output as a guaranteed diagnosis.
- **Room to grow:** the project can be extended with field testing, verified agricultural data, and additional sensors.

## Current scope

The current firmware includes camera/display interaction, network configuration, image submission to a Gemini endpoint, and result/history screens. The exact behaviour depends on the firmware and hardware revision being used.

Environmental sensor support is defined in the code, but the sensor should only be described as active in a particular build if it is physically connected and its readings have been verified.

## What comes next

Useful next steps for the project include testing with real crop images from different conditions, recording where the AI is uncertain, checking recommendations with agricultural experts, and documenting which features have been tested on the actual device.
