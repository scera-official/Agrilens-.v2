# Privacy and security

AgriLens is intended to be shared as a learning and demonstration project. Please keep personal information and access credentials out of public commits.

## Credentials

The public firmware copy uses blank default values for the Wi-Fi network, Wi-Fi password, and Gemini API key. Configure your own values on your own device rather than publishing them in the repository.

The firmware also stores settings using the ESP32 Preferences feature. If you previously configured credentials on a device, those saved settings may remain on that device even after you upload a cleaned source file.

## If a credential was exposed

If a real API key or password was committed or shared publicly, treat it as exposed. Revoke or rotate it with the relevant service, and review the repository history as well as the current files. Removing a secret from the latest version does not erase it from earlier Git history.

## Crop images and AI responses

Crop images may be sent to the configured external AI service for analysis. Avoid photographing people, private property, or identifying information unless you have permission. Review the provider's current privacy and data-handling terms before using the project with sensitive images.

## Accuracy and safety

AI-generated crop assessments can be incomplete or incorrect. Use them as an initial reference only. Confirm important treatment decisions with a qualified local agricultural professional, particularly before applying pesticides or other chemicals.
