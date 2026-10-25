/*
 * AgriLens - AI-assisted crop analysis
 *
 * Hardware: ESP32-S3 camera board, ST7789 display and optional BMP280 sensor.
 * The device captures crop images and sends them to the configured Gemini
 * Vision endpoint for an initial assessment. Results are intended as guidance
 * and should be checked against local agricultural expertise.
 */

#include <Arduino.h>
#include <esp_interface.h>
// Compatibility definitions for Arduino-ESP32 Core 3.x
#ifndef ESP_IF_WIFI_STA
  #define ESP_IF_WIFI_STA 0
#endif
#ifndef ESP_IF_WIFI_AP
  #define ESP_IF_WIFI_AP 1
#endif
#ifndef ESP_IF_WIFI_NAN
  #define ESP_IF_WIFI_NAN 2
#endif

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <NetworkClientSecure.h>  // Secure client type used by HTTPClient on Core 3.x
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Preferences.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "mbedtls/base64.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Set to 0 to build without the live JPEG preview (falls back to a static
// "steady the leaf, then press" framing screen with the same crop guide,
// no extra library needed). Set to 1 (default) for live streaming preview.
// Requires Library Manager: "JPEGDEC" by Larry Bank.
#define ENABLE_LIVE_JPEG_PREVIEW 1
#if ENABLE_LIVE_JPEG_PREVIEW
  #include <JPEGDEC.h>
#endif

// Text emphasis uses the built-in Adafruit GFX font.

// ============================================================================
// FONT CONFIGURATION FOR ESP32-S3 (ST7789 Display 240x320)
// ============================================================================
// RECOMMENDED FONT SETUP:
// - Primary Font: Adafruit GFX Default Font (Built-in, no dependencies)
//   * Compact and efficient for embedded displays
//   * Sizes: 1x (5x7px), 2x (10x14px), 3x (15x21px), etc.
//   * Best for: Headers, buttons, navigation text
//
// - ALTERNATIVE (Optional High-Quality): FreeSans9pt7b, FreeSans12pt7b
//   * Vector-based fonts from Adafruit GFXLibrary
//   * Requires: #include <Fonts/FreeSans9pt7b.h>
//   * Set with: display.setFont(&FreeSans9pt7b);
//   * Usage: display.setFont(&FreeSans9pt7b);
//           display.setTextSize(1);
//           display.println("Text here");
//
// - Current Implementation: Default Adafruit GFX Font (Optimized for ESP32-S3)
//   * No additional font dependencies required
//   * Maximum compatibility with PSRAM Canvas rendering
//   * Consistent rendering across all UI elements
// ============================================================================

#define PRODUCT_NAME     "AGRILENS"
#define PRODUCT_TAGLINE  "Professional Agricultural Diagnostics"
#define PRODUCT_SUBTITLE "AI-Powered Farm Health Analysis"
#define PRODUCT_VERSION  "v1.0"

// ============================================================================
// HARDWARE PIN ALLOCATION TABLE (STRICT CONSTRAINTS - do not change)
// ============================================================================
#define TFT_CS        21
#define TFT_DC        47
#define TFT_RST       48
#define TFT_MOSI      1
#define TFT_SCLK      2
#define TFT_BL        3

#define PIN_BUTTON    19
#define PIN_FLASH     14

#define I2C_SDA       41
#define I2C_SCL       42
#define BMP280_ADDR   0x76

// Camera Pin Mapping (ESP32-S3-CAM N16R8 Bus)
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD     4
#define CAM_PIN_SIOC     5

#define CAM_PIN_D7      16
#define CAM_PIN_D6      17
#define CAM_PIN_D5      18
#define CAM_PIN_D4      12
#define CAM_PIN_D3      10
#define CAM_PIN_D2       8
#define CAM_PIN_D1       9
#define CAM_PIN_D0      11

#define CAM_PIN_VSYNC    6
#define CAM_PIN_HREF     7
#define CAM_PIN_PCLK    13

// ============================================================================
// COLOR PALETTE (16-BIT RGB565) - AGRILENS INDUSTRIAL AGRICULTURE
// ============================================================================
// AgriLens display colors, chosen for clear contrast in the field.
//   Background      #EBE9E9  RGB(235, 233, 233) -> 0xF7BD  Light neutral background
//   Primary         #3D5A80  RGB(61, 90, 128)   -> 0x3B10  Deep teal/blue for headers & actions
//   Accent          #7FD856  RGB(127, 216, 86)  -> 0x3ED2  Bright lime/green for highlights
//   Text Dark       #1A1A1A  RGB(26, 26, 26)    -> 0x1863  Deep charcoal for text
//   Warning Amber   #FFA500  RGB(255, 165, 0)   -> 0xFD40  Alert status
//   Critical Red    #E74C3C  RGB(231, 76, 60)   -> 0xE926  Critical alerts
//   Status Green    #27AE60  RGB(39, 174, 96)   -> 0x3511  Success indicators
//
// SEMANTIC USAGE:
// - Headers & Navigation: Primary (#3D5A80)
// - Accent Highlights: Accent (#7FD856)
// - Primary Text: Text Dark (#1A1A1A)
// - Backgrounds: Background (#EBE9E9)
// - Warnings: Amber
// - Critical: Red
// - Success: Green
//
#define COLOR_BG_DARK          0xF7BD   // Light neutral background
#define COLOR_PANEL_DARK       0x3B10   // Deep teal primary
#define COLOR_PRIMARY_DARK     0x3B10   // Deep teal primary
#define COLOR_PRIMARY          0x3B10   // Primary teal/blue
#define COLOR_ACCENT           0x3ED2   // Bright lime accent
#define COLOR_CREAM            0xF7BD   // Light background
#define COLOR_TEXT_DARK        0x1863   // Deep charcoal text
#define COLOR_AMBER            0xFD40   // Warning amber
#define COLOR_RED              0xE926   // Critical red
#define COLOR_BLUE             0x3B10   // Primary blue
#define COLOR_WHITE            0xFFFF   // Pure white
#define COLOR_BLACK            0x0000   // Pure black
#define COLOR_GRAY_LIGHT       0xCE79   // Light gray
#define COLOR_GRAY_MED         0x8410   // Medium gray
#define COLOR_GREEN            0x3511   // Success green


// ============================================================================
// NETWORK & CLOUD ENDPOINT CONFIGURATION
// ============================================================================
// Public defaults are intentionally blank so private credentials are not
// stored in the repository. Device settings saved in Preferences take priority.
const char* DEFAULT_WIFI_SSID     = "";
const char* DEFAULT_WIFI_PASSWORD = "";
const char* DEFAULT_GEMINI_KEY    = "";

// Keep the model endpoint in one place so it can be updated if needed.
const char* GEMINI_ENDPOINT_URL = "https://generativelanguage.googleapis.com/v1beta/models/gemini-3.5-flash-lite:generateContent";

// ============================================================================
// SYSTEM ENUMS
// ============================================================================
enum AppState {
    APP_BOOTING,
    APP_DIAGNOSTICS,
    APP_MENU_MAIN,
    APP_MENU_CROP_SELECT,
    APP_MENU_PART_SELECT,       // Leaf/Fruit/Stem/Flower/Whole Plant/Auto
    APP_MENU_ANALYSIS_SELECT,   // Disease/Pest/Nutrient/Fruit Damage/General/Auto
    APP_LIVE_PREVIEW,           // live viewfinder with crop guide
    APP_MENU_HISTORY,
    APP_HISTORY_VIEW,
    APP_MENU_SETTINGS,
    APP_MENU_CAMERA_SETTINGS,
    APP_WIFI_INFO,
    APP_MENU_INFO,
    APP_CAPTURING,
    APP_PROCESSING,
    APP_DISPLAY_RESULT,
    APP_ERROR_SCREEN
};

enum PlantPart {
    PART_LEAF = 0, PART_FRUIT, PART_STEM, PART_FLOWER, PART_WHOLE_PLANT, PART_AUTO
};
enum AnalysisType {
    ANALYSIS_DISEASE = 0, ANALYSIS_PEST, ANALYSIS_NUTRIENT,
    ANALYSIS_FRUIT_DAMAGE, ANALYSIS_GENERAL, ANALYSIS_AUTO
};
enum AppLanguage { LANG_ENGLISH = 0, LANG_TAMIL = 1 };

enum TextAlignment {
    ALIGN_LEFT,
    ALIGN_CENTER,
    ALIGN_RIGHT
};

enum FaultCode {
    FAULT_NONE = 0,
    FAULT_CAMERA_INIT = 1,
    FAULT_BMP280_INIT = 2,
    FAULT_WIFI_TIMEOUT = 3,
    FAULT_CAPTURE_FAIL = 4,
    FAULT_BASE64_FAIL = 5,
    FAULT_HTTP_FAIL = 6,
    FAULT_JSON_PARSE = 7,
    FAULT_LOW_MEM = 8
};

enum IconType {
    ICON_LEAF,
    ICON_CAMERA,
    ICON_CLOCK,
    ICON_GEAR,
    ICON_INFO,
    ICON_WARNING,
    ICON_CHECK,
    ICON_THERMO
};

// ============================================================================
// DATA STRUCTURES
// ============================================================================
struct DiagnosticReport {
    String crop;
    String part;          // which plant part was scanned
    String analysisType;  // which analysis was requested
    float temperature;
    String diagnosis;
    String severity;
    int confidence;
    String action;
    String prevention;
    String temp_note;
    uint32_t timestamp;
    bool isValid;
};

struct Settings {
    bool flashEnabled;
    framesize_t resolution;
    int brightness;
    int contrast;
    bool ecoMode;
    int cropIndex;
    int partIndex;      // NEW
    int analysisIndex;  // NEW
    uint8_t language;   // AppLanguage
    char wifiSSID[32];
    char wifiPass[64];
    char apiKey[128];       // Gemini API key
};

struct SystemStatus {
    bool psramOK;
    bool cameraOK;
    bool bmp280OK;
    bool wifiOK;
    uint32_t freeHeap;
    uint32_t freePsram;
    int rssi;
    FaultCode lastFault;
    uint32_t bootTimeMs;
};

// ============================================================================
// GLOBAL OBJECTS & STATE VARIABLES
// ============================================================================
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Preferences preferences;

// ---------------------------------------------------------------------------
// High-performance report framebuffer.
// The report is rendered ONCE into PSRAM and only the required viewport is
// streamed to the panel, which is what makes the result screen scroll
// smoothly with no flicker or "page cutting".
// ---------------------------------------------------------------------------
class PSRAMCanvas : public Adafruit_GFX {
public:
    uint16_t *buf;
    int16_t cw, ch;

    PSRAMCanvas(int16_t w, int16_t h) : Adafruit_GFX(w, h), buf(nullptr), cw(w), ch(h) {
        buf = (uint16_t*)heap_caps_calloc((size_t)w * h, sizeof(uint16_t),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }

    ~PSRAMCanvas() {
        if (buf) heap_caps_free(buf);
    }

    bool ok() const { return buf != nullptr; }

    void clear(uint16_t color) {
        if (!buf) return;
        size_t n = (size_t)cw * ch;
        for (size_t i = 0; i < n; i++) buf[i] = color;
    }

    void drawPixel(int16_t x, int16_t y, uint16_t color) override {
        if (!buf || x < 0 || y < 0 || x >= cw || y >= ch) return;
        buf[(size_t)y * cw + x] = color;
    }

    void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) override {
        if (!buf || y < 0 || y >= ch || w <= 0) return;
        if (x < 0) { w += x; x = 0; }
        if (x + w > cw) w = cw - x;
        if (w <= 0) return;
        uint16_t *p = buf + (size_t)y * cw + x;
        for (int16_t i = 0; i < w; i++) p[i] = color;
    }

    void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) override {
        if (!buf || x < 0 || x >= cw || h <= 0) return;
        if (y < 0) { h += y; y = 0; }
        if (y + h > ch) h = ch - y;
        if (h <= 0) return;
        for (int16_t i = 0; i < h; i++) buf[(size_t)(y + i) * cw + x] = color;
    }

    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override {
        if (!buf || w <= 0 || h <= 0) return;
        if (x < 0) { w += x; x = 0; }
        if (y < 0) { h += y; y = 0; }
        if (x + w > cw) w = cw - x;
        if (y + h > ch) h = ch - y;
        if (w <= 0 || h <= 0) return;
        for (int16_t yy = 0; yy < h; yy++) {
            uint16_t *p = buf + (size_t)(y + yy) * cw + x;
            for (int16_t xx = 0; xx < w; xx++) p[xx] = color;
        }
    }
};

PSRAMCanvas *reportCanvas = nullptr;
int reportCanvasHeight = 1400;
bool reportFrameReady = false;
unsigned long lastReportFrame = 0;
const uint16_t REPORT_FRAME_MS = 28; // ~28 FPS, limited by SPI bandwidth

AppState currentState = APP_BOOTING;
int menuSelection = 0;
Settings appSettings;
SystemStatus sysStatus;

const char* CROPS_LIST[] = {"Tomato", "Potato", "Paddy", "Maize", "Cotton", "General"};
const int TOTAL_CROPS = 6;

// ---------------------------------------------------------------------------
// Plant part & analysis-type menus .
// Tanglish = Tamil words spelled in the English alphabet, chosen because the
// display's built-in font has no native Tamil glyphs (see header note).
// ---------------------------------------------------------------------------
const char* PART_LIST_EN[] = {"Leaf", "Fruit", "Stem", "Flower", "Whole Plant", "Auto Detect"};
const char* PART_LIST_TA[] = {"Elai", "Pazham", "Thandu", "Poo", "Selai Muzhudhum", "Thaanaga Kandupidi"};
const int TOTAL_PARTS = 6;

const char* ANALYSIS_LIST_EN[] = {"Disease", "Pest Damage", "Nutrient Problem", "Fruit Damage", "General Health", "Auto Detect"};
const char* ANALYSIS_LIST_TA[] = {"Noi", "Poochi Sedham", "Ootam Kuraivu", "Pazha Sedham", "Podhu Nalam", "Thaanaga Kandupidi"};
const int TOTAL_ANALYSIS = 6;

// Simple bilingual string picker. Every user-facing label in renderUI()
// goes through this, so adding real Tamil glyphs later means changing
// this ONE function (and the STR_* tables above) instead of hunting
// through every screen.
inline String T(const char* en, const char* ta) {
    return String(appSettings.language == LANG_TAMIL ? ta : en);
}
inline const char* T_part(int i)     { return appSettings.language == LANG_TAMIL ? PART_LIST_TA[i] : PART_LIST_EN[i]; }
inline const char* T_analysis(int i) { return appSettings.language == LANG_TAMIL ? ANALYSIS_LIST_TA[i] : ANALYSIS_LIST_EN[i]; }

DiagnosticReport historyRAM[3];
int historyCount = 0;
int activeHistoryIndex = -1;

DiagnosticReport currentReport;
String lastErrorMessage = "";

// BMP280 Calibration Registers
uint16_t dig_T1 = 0;
int16_t  dig_T2 = 0;
int16_t  dig_T3 = 0;
float lastReadTemperature = 25.0f;

// Scrolling Engine State
int scrollY = 0;
int maxScrollY = 0;
unsigned long lastScrollTime = 0;

// Continuous auto-repeat scrolling: down -> pause -> up -> pause -> repeat.
// This is what makes long reports keep re-reading from the top instead of
// stopping dead once they reach the bottom.
enum ScrollPhase { SCROLL_DOWN, SCROLL_PAUSE_BOTTOM, SCROLL_UP, SCROLL_PAUSE_TOP };
ScrollPhase scrollPhase = SCROLL_DOWN;
unsigned long scrollPauseStart = 0;
const int SCROLL_STEP_PX          = 2;     // pixels moved per frame (raise for faster scroll)
const unsigned long SCROLL_PAUSE_MS = 2500; // pause at top/bottom before reversing (ms)

// Button Physics Parameters
unsigned long buttonPressStart = 0;
bool buttonIsPressed = false;
bool buttonHandled = false;
const unsigned long LONG_PRESS_MS = 700;
const unsigned long DEBOUNCE_MS = 50;

// ============================================================================
// BACK BUTTON NAVIGATION STACK (Professional Navigation System)
// ============================================================================
// Professional multi-screen navigation uses a stack-based system for
// seamless back button support throughout the application. This ensures
// users can navigate back from ANY screen without losing context.
// Maximum 16 screens in navigation history to prevent stack overflow.
// ============================================================================
#define MAX_NAV_STACK 16
AppState navigationStack[MAX_NAV_STACK];
int navigationStackPtr = 0;
bool showBackButton = false;

// Push a screen state onto the navigation stack
void pushNavState(AppState state) {
    if (navigationStackPtr < MAX_NAV_STACK - 1) {
        navigationStack[navigationStackPtr++] = state;
    }
}

// Pop back to previous screen (back button press)
AppState popNavState() {
    if (navigationStackPtr > 0) {
        return navigationStack[--navigationStackPtr];
    }
    return APP_MENU_MAIN;  // Default fallback to main menu
}

// Check if back button should be displayed
bool canGoBack() {
    return navigationStackPtr > 1;  // Can go back if stack has more than just current state
}

// Dynamic UI Animations
uint8_t animFrame = 0;
unsigned long lastAnimTime = 0;

// ---------------------------------------------------------------------------
// Industrial/minimal UI constant. Original build used radius 8-10 (soft,
// "consumer app" rounding). Flatter corners read as a professional /
// industrial instrument, so every card/menu row now uses this instead.
// ---------------------------------------------------------------------------
const int UI_RADIUS = 4;

// ---------------------------------------------------------------------------
// Live preview state .
// Camera stays in JPEG mode (unchanged) -- each preview frame is grabbed
// at a small size and decoded straight to the panel with JPEGDEC so it
// streams smoothly. The final photo used for AI analysis is still taken by
// the existing startCaptureAndAnalysis() at the user's chosen resolution,
// so the working capture/base64/HTTP pipeline is untouched.
// ---------------------------------------------------------------------------
const unsigned long PREVIEW_FRAME_MS = 70;  // throttle: JPEG decode + SPI cost
unsigned long lastPreviewFrame = 0;
const int PREVIEW_TOP = 60;     // preview image area starts below the ribbon
const int PREVIEW_H   = 220;    // preview image area height
bool previewCameraModeActive = false;

// ---------------------------------------------------------------------------
// Background AI pipeline state.
// The image is captured + base64-encoded on the main thread (fast), then a
// FreeRTOS task on the opposite core does the network+JSON work so the UI
// thread is completely free to keep the spinner animation smooth. The task
// touches ONLY these globals, and ONLY the main thread reads them back --
// and only after it observes netTaskState leave NET_RUNNING.
// ---------------------------------------------------------------------------
enum NetTaskState { NET_IDLE = 0, NET_RUNNING = 1, NET_DONE_OK = 2, NET_DONE_FAIL = 3 };
volatile int netTaskState = NET_IDLE;
TaskHandle_t netTaskHandle = nullptr;
char*  g_b64Buffer = nullptr;
size_t g_b64Len = 0;
float  g_capturedTemp = 25.0f;   // sampled on main thread BEFORE the task starts
int    g_capturedCropIndex = 0;  // snapshot so a settings change mid-scan can't race it
int    g_capturedPartIndex = 0;      // snapshot of selected plant part
int    g_capturedAnalysisIndex = 0;  // snapshot of selected analysis type
uint8_t g_capturedLanguage = 0;      // snapshot of selected language

// ============================================================================
// FORWARD DECLARATIONS FOR INTER-MODULE COMPATIBILITY
// ============================================================================
void drawTextAligned(const String& text, int x, int y, uint16_t color, uint8_t size, TextAlignment align);
void drawTextWrapped(const String& text, int x, int &y, int maxWidth, uint16_t color, uint8_t size);
void drawCard(int x, int y, int w, int h, uint16_t bg, uint16_t border, int radius);
void drawIcon(uint8_t type, int cx, int cy, uint16_t color, uint16_t detail);
void drawMenuRow(int y, int h, bool isSel, int iconType, const String& label);

// ============================================================================
// BOLD TEXT EFFECT - Default Font with Double-Render Bold
// ============================================================================
void drawTextBold(const String& text, int x, int y, uint16_t color) {
    // Draw text twice with offset for bold effect
    tft.setCursor(x, y);
    tft.setTextColor(color);
    tft.print(text);
    tft.setCursor(x + 1, y);
    tft.print(text);
}

// ============================================================================
// UNIVERSAL BACK TO DASHBOARD BUTTON
// ============================================================================
void drawBackToDashboardButton() {
    // Universal button at bottom of screen - appears on all feature screens
    const int btnY = 285;
    const int btnW = 220;
    const int btnH = 28;
    const int btnX = (240 - btnW) / 2;
    
    // Button background
    tft.fillRoundRect(btnX, btnY, btnW, btnH, 6, COLOR_ACCENT);
    tft.drawRoundRect(btnX, btnY, btnW, btnH, 6, COLOR_PRIMARY_DARK);
    
    // Button text - ✓ DEFAULT FONT WITH BOLD EFFECT
    tft.setFont(NULL);  // ✓ DEFAULT ADAFRUIT GFX FONT
    tft.setTextSize(1);
    tft.setTextColor(COLOR_WHITE);
    int textWidth = 140;
    int textX = (240 - textWidth) / 2;
    tft.setCursor(textX, btnY + 20);
    tft.print("< Back to Dashboard");
    // ✓ BOLD EFFECT: Draw again with 1px offset
    tft.setCursor(textX + 1, btnY + 20);
    tft.print("< Back to Dashboard");
}
void drawFooterHint(const String& text);
void drawHeader(const String& title);
void drawLogoBadge(int cx, int cy, float scale);
void drawWiFiSignalBar(int x, int y, int rssi);
bool initBMP280();
float readBMP280Temperature();
bool initCamera();
void switchState(AppState newState);
void renderUI();
void handleButtonAction(bool isLongPress);
bool startCaptureAndAnalysis();
void updateProcessingSpinner();
static void netWorkerTask(void* pvParameters);
void runPOSTDiagnostics();
void loadSettingsFromNVS();
void saveSettingsToNVS();
void freeReportCanvas();
void drawCropGuideOverlay();
void enterLivePreview();
void exitLivePreview();
void renderLivePreviewFrame();
static String buildAIPrompt(int cropIndex, int partIndex, int analysisIndex,
                             float ambientTemp, uint8_t language);

// ============================================================================
// PART 1: TEXT / CARD / ICON RENDERING PRIMITIVES
// ============================================================================

static int textPixelWidth(const String &s, uint8_t size) {
    return (int)s.length() * 6 * size;
}

static void canvasTextAligned(PSRAMCanvas &c, const String &text, int x, int y,
                              uint16_t color, uint8_t size, TextAlignment align) {
    c.setTextWrap(false);
    c.setTextSize(size);
    c.setTextColor(color);
    int w = textPixelWidth(text, size);
    if (align == ALIGN_CENTER) x -= w / 2;
    else if (align == ALIGN_RIGHT) x -= w;
    c.setCursor(x, y);
    c.print(text);
}

static int canvasWrappedHeight(PSRAMCanvas &c, const String &text,
                               int maxWidth, uint8_t size) {
    if (text.length() == 0) return 8 * size + 2;

    const int charW = 6 * size;
    const int lineH = 8 * size + 3;
    int charsPerLine = max(1, maxWidth / charW);
    int lines = 0;

    int start = 0;
    while (start < (int)text.length()) {
        int nl = text.indexOf('\n', start);
        if (nl < 0) nl = text.length();

        String para = text.substring(start, nl);
        int pos = 0;
        if (para.length() == 0) {
            lines++;
        } else {
            while (pos < (int)para.length()) {
                int remaining = para.length() - pos;
                if (remaining <= charsPerLine) {
                    lines++;
                    break;
                }

                int cut = pos + charsPerLine;
                int space = para.lastIndexOf(' ', cut);
                if (space < pos + 1) space = cut;
                lines++;
                pos = space;
                while (pos < (int)para.length() && para.charAt(pos) == ' ') pos++;
            }
        }

        start = nl + 1;
        if (start == (int)text.length()) lines++;
    }

    return max(1, lines) * lineH;
}

static void canvasTextWrapped(PSRAMCanvas &c, const String &text, int x, int y,
                              int maxWidth, uint16_t color, uint8_t size) {
    c.setTextWrap(false);
    c.setTextSize(size);
    c.setTextColor(color);

    const int charW = 6 * size;
    const int lineH = 8 * size + 3;
    const int charsPerLine = max(1, maxWidth / charW);

    int start = 0;
    while (start < (int)text.length()) {
        int nl = text.indexOf('\n', start);
        if (nl < 0) nl = text.length();

        String para = text.substring(start, nl);
        int pos = 0;

        if (para.length() == 0) {
            y += lineH;
        } else {
            while (pos < (int)para.length()) {
                int remaining = para.length() - pos;
                int take = min(charsPerLine, remaining);

                if (take < remaining) {
                    int space = para.lastIndexOf(' ', pos + take);
                    if (space >= pos + 1) take = space - pos;
                }

                String line = para.substring(pos, pos + take);
                line.trim();
                c.setCursor(x, y);
                c.print(line);
                y += lineH;

                pos += take;
                while (pos < (int)para.length() && para.charAt(pos) == ' ') pos++;
            }
        }

        start = nl + 1;
    }
}

static void canvasCard(PSRAMCanvas &c, int x, int y, int w, int h,
                       uint16_t bg, uint16_t border, int radius) {
    if (w <= 0 || h <= 0) return;
    c.fillRoundRect(x, y, w, h, radius, bg);
    c.drawRoundRect(x, y, w, h, radius, border);
}

void drawTextAligned(const String& text, int x, int y,
                            uint16_t color, uint8_t size, TextAlignment align) {
    tft.setTextWrap(false);
    tft.setFont(NULL);  // ✓ DEFAULT ADAFRUIT GFX FONT
    tft.setTextSize(size);
    tft.setTextColor(color);
    int w = textPixelWidth(text, size);
    if (align == ALIGN_CENTER) x -= w / 2;
    else if (align == ALIGN_RIGHT) x -= w;
    tft.setCursor(x, y);
    tft.print(text);
    // ✓ BOLD EFFECT: Draw again with 1px offset for bold appearance
    tft.setCursor(x + 1, y);
    tft.print(text);
}

void drawTextWrapped(const String& text, int x, int &y, int maxWidth,
                            uint16_t color, uint8_t size) {
    tft.setTextWrap(false);
    tft.setFont(NULL);  // ✓ DEFAULT ADAFRUIT GFX FONT
    tft.setTextSize(size);
    tft.setTextColor(color);

    const int charW = 6 * size;
    const int lineH = 8 * size + 3;
    const int charsPerLine = max(1, maxWidth / charW);

    int start = 0;
    while (start < (int)text.length()) {
        int nl = text.indexOf('\n', start);
        if (nl < 0) nl = text.length();

        String para = text.substring(start, nl);
        int pos = 0;

        if (para.length() == 0) {
            y += lineH;
        } else {
            while (pos < (int)para.length()) {
                int remaining = para.length() - pos;
                int take = min(charsPerLine, remaining);

                if (take < remaining) {
                    int space = para.lastIndexOf(' ', pos + take);
                    if (space >= pos + 1) take = space - pos;
                }

                String line = para.substring(pos, pos + take);
                line.trim();
                tft.setCursor(x, y);
                tft.print(line);
                y += lineH;

                pos += take;
                while (pos < (int)para.length() && para.charAt(pos) == ' ') pos++;
            }
        }

        start = nl + 1;
    }
}

void drawCard(int x, int y, int w, int h,
                     uint16_t bg, uint16_t border, int radius) {
    tft.fillRoundRect(x, y, w, h, radius, bg);
    tft.drawRoundRect(x, y, w, h, radius, border);
}

// Small vector icon set. Every icon is built only from additive shapes
// (filled circles / rects / lines) so it renders correctly on ANY
// background without needing to know the background color.
void drawIcon(uint8_t type, int cx, int cy, uint16_t color, uint16_t detail) {
    switch (type) {
        case ICON_LEAF:
            tft.fillCircle(cx, cy, 9, color);
            tft.drawLine(cx - 6, cy + 6, cx + 6, cy - 6, detail);
            tft.drawLine(cx - 6, cy + 6, cx - 2, cy + 9, detail);
            break;

        case ICON_CAMERA:
            tft.fillRoundRect(cx - 10, cy - 6, 20, 14, 3, color);
            tft.fillRoundRect(cx - 4, cy - 10, 8, 5, 2, color);
            tft.drawCircle(cx, cy + 1, 5, detail);
            tft.fillCircle(cx, cy + 1, 2, detail);
            break;

        case ICON_CLOCK:
            tft.drawCircle(cx, cy, 10, color);
            tft.drawCircle(cx, cy, 9, color);
            tft.drawLine(cx, cy, cx, cy - 6, detail);
            tft.drawLine(cx, cy, cx + 5, cy + 2, detail);
            break;

        case ICON_GEAR:
            tft.fillCircle(cx, cy, 6, color);
            tft.fillRect(cx - 2, cy - 10, 4, 5, color);
            tft.fillRect(cx - 2, cy + 5, 4, 5, color);
            tft.fillRect(cx - 10, cy - 2, 5, 4, color);
            tft.fillRect(cx + 5, cy - 2, 5, 4, color);
            tft.fillCircle(cx, cy, 3, detail);
            break;

        case ICON_INFO:
            tft.fillCircle(cx, cy, 10, color);
            tft.fillRect(cx - 1, cy - 2, 2, 6, detail);
            tft.fillRect(cx - 1, cy - 7, 2, 2, detail);
            break;

        case ICON_WARNING:
            tft.fillTriangle(cx, cy - 10, cx - 10, cy + 8, cx + 10, cy + 8, color);
            tft.fillRect(cx - 1, cy - 4, 2, 6, detail);
            tft.fillRect(cx - 1, cy + 4, 2, 2, detail);
            break;

        case ICON_CHECK:
            tft.drawLine(cx - 7, cy, cx - 2, cy + 6, color);
            tft.drawLine(cx - 7, cy + 1, cx - 2, cy + 7, color);
            tft.drawLine(cx - 2, cy + 6, cx + 8, cy - 7, color);
            tft.drawLine(cx - 2, cy + 7, cx + 8, cy - 6, color);
            break;

        case ICON_THERMO:
            tft.fillRoundRect(cx - 3, cy - 10, 6, 16, 3, color);
            tft.fillCircle(cx, cy + 8, 6, color);
            tft.fillRoundRect(cx - 1, cy - 6, 2, 12, 1, detail);
            tft.fillCircle(cx, cy + 8, 3, detail);
            break;
    }
}

// AGRI SENSE brand mark: flat rounded-square badge (industrial instrument
// look, not a playful circle), a minimal leaf stroke, and a thin accent
// corner tick. Fully vector, scales cleanly for boot/capture/processing.
void drawLogoBadge(int cx, int cy, float scale) {
    // Clean AgriLens-style instrument mark:
    // square sensor frame + leaf + central vein + small data node.
    int r = max(8, (int)(27 * scale));
    int rad = max(3, (int)(r * 0.22f));
    int inner = max(4, (int)(r * 0.72f));

    // Outer sensor housing.
    tft.fillRoundRect(cx - r, cy - r, r * 2, r * 2, rad, COLOR_PRIMARY_DARK);
    tft.drawRoundRect(cx - r, cy - r, r * 2, r * 2, rad, COLOR_ACCENT);

    // Inner viewfinder corners.
    int c = max(3, (int)(r * 0.24f));
    uint16_t guide = COLOR_GRAY_LIGHT;
    tft.drawFastHLine(cx - inner, cy - inner, c, guide);
    tft.drawFastVLine(cx - inner, cy - inner, c, guide);
    tft.drawFastHLine(cx + inner - c, cy - inner, c, guide);
    tft.drawFastVLine(cx + inner, cy - inner, c, guide);
    tft.drawFastHLine(cx - inner, cy + inner, c, guide);
    tft.drawFastVLine(cx - inner, cy + inner - c, c, guide);
    tft.drawFastHLine(cx + inner - c, cy + inner, c, guide);
    tft.drawFastVLine(cx + inner, cy + inner - c, c, guide);

    // Stylised leaf silhouette.
    int lx = cx - (int)(r * 0.22f);
    int ly = cy + (int)(r * 0.10f);
    int lw = max(6, (int)(r * 0.90f));
    int lh = max(6, (int)(r * 0.62f));

    tft.fillTriangle(
        lx - lw / 2, ly,
        lx + lw / 2, ly - lh,
        lx + lw / 2 - 2, ly + 2,
        COLOR_ACCENT
    );
    tft.fillTriangle(
        lx - lw / 2, ly,
        lx + lw / 2 - 2, ly + 2,
        lx - lw / 5, ly + lh / 2,
        COLOR_PRIMARY
    );

    // Leaf vein / AI scan line.
    tft.drawLine(lx - lw / 2 + 1, ly, lx + lw / 2 - 2, ly - lh + 2, COLOR_CREAM);
    tft.drawLine(lx - lw / 6, ly - 2, lx + lw / 6, ly - lh / 3, COLOR_CREAM);

    // Small data node.
    int nx = cx + (int)(r * 0.52f);
    int ny = cy - (int)(r * 0.52f);
    tft.fillCircle(nx, ny, max(2, (int)(r * 0.09f)), COLOR_ACCENT);
}

void drawWiFiSignalBar(int x, int y, int rssi) {
    int bars = 0;
    if (rssi > -85) bars = 1;
    if (rssi > -70) bars = 2;
    if (rssi > -55) bars = 3;
    if (rssi > -45) bars = 4;

    for (int i = 0; i < 4; i++) {
        int h = 3 + i * 3;
        uint16_t c = (i < bars) ? COLOR_ACCENT : COLOR_GRAY_MED;
        tft.fillRect(x + i * 5, y + 12 - h, 3, h, c);
    }
}

static void drawProgressBar(int x, int y, int w, int h, int value,
                            uint16_t fg, uint16_t bg) {
    tft.fillRoundRect(x, y, w, h, h / 2, bg);
    value = constrain(value, 0, 100);
    int fillW = (w * value) / 100;
    if (fillW > 0) tft.fillRoundRect(x, y, fillW, h, h / 2, fg);
}

static void drawSpinner(int cx, int cy, int radius, uint8_t frame, uint16_t color) {
    for (int i = 0; i < 8; i++) {
        float a = (i + (frame & 7)) * 0.785398f;
        int x = cx + (int)(cos(a) * radius);
        int y = cy + (int)(sin(a) * radius);
        int r = (i == 0) ? 4 : 2;
        tft.fillCircle(x, y, r, (i == 0) ? color : COLOR_GRAY_MED);
    }
}

// Standard menu-list row: rounded card + optional icon + label + a right-hand
// chevron on the selected row. Every screen that shows a list of options
// goes through this ONE function, so spacing and style are always identical
// and rows can never overlap the next one (heights are always passed
// explicitly by the caller's grid math -- see renderUI()).
void drawMenuRow(int y, int h, bool isSel, int iconType, const String& label) {
    uint16_t cardBg     = isSel ? COLOR_PRIMARY : COLOR_CREAM;
    uint16_t cardBorder = isSel ? COLOR_ACCENT  : COLOR_GRAY_LIGHT;
    uint16_t textColor  = isSel ? COLOR_WHITE   : COLOR_TEXT_DARK;
    uint16_t iconColor  = isSel ? COLOR_CREAM   : COLOR_PRIMARY;
    uint16_t iconDetail = isSel ? COLOR_PRIMARY : COLOR_CREAM;

    drawCard(10, y, 220, h, cardBg, cardBorder, UI_RADIUS);
    if (isSel) tft.fillRect(10, y + 4, 4, h - 8, COLOR_ACCENT);

    int textX = 22;
    if (iconType >= 0) {
        tft.fillRoundRect(18, y + h / 2 - 12, 24, 24, 5,
                          isSel ? COLOR_PANEL_DARK : COLOR_GRAY_LIGHT);
        drawIcon((uint8_t)iconType, 30, y + h / 2, iconColor, iconDetail);
        textX = 52;
    }

    uint8_t txtSize = (h >= 38) ? 2 : 1;
    int txtY = y + h / 2 - (txtSize == 2 ? 7 : 4);
    drawTextAligned(label, textX, txtY, textColor, txtSize, ALIGN_LEFT);

    if (isSel) {
        tft.fillTriangle(210, y + h / 2 - 5, 210, y + h / 2 + 5, 217, y + h / 2, COLOR_CREAM);
    }
}

void drawFooterHint(const String& text) {
    drawTextAligned(text, 120, 306, COLOR_GRAY_MED, 1, ALIGN_CENTER);
}

// Audio removed. Kept as a no-op so existing call sites stay valid.
void playSoundPreset(const char*) {}

void freeReportCanvas() {
    if (reportCanvas) {
        delete reportCanvas;
        reportCanvas = nullptr;
    }
    reportFrameReady = false;
    maxScrollY = 0;
    scrollY = 0;
}

// ============================================================================
// PART 2: SENSOR + CAMERA + NVS SETTINGS DRIVERS  (unchanged hardware logic)
// ============================================================================

bool initBMP280() {
    // Direct BMP280 access via Wire.
    // This intentionally avoids Adafruit_BMP280/Adafruit_Sensor because
    // Adafruit_Sensor defines sensor_t, which conflicts with esp_camera's
    // sensor_t on ESP32 Arduino Core 3.x.
    Wire.beginTransmission(BMP280_ADDR);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        sysStatus.bmp280OK = false;
        return false;
    }

    // Read calibration data from 0x88..0x9F.
    Wire.beginTransmission(BMP280_ADDR);
    Wire.write(0x88);
    if (Wire.endTransmission(false) != 0) {
        sysStatus.bmp280OK = false;
        return false;
    }

    uint8_t n = Wire.requestFrom((int)BMP280_ADDR, 24);
    if (n != 24) {
        sysStatus.bmp280OK = false;
        return false;
    }

    auto readU16LE = []() -> uint16_t {
        uint8_t lo = Wire.read();
        uint8_t hi = Wire.read();
        return (uint16_t)lo | ((uint16_t)hi << 8);
    };
    auto readS16LE = []() -> int16_t {
        uint8_t lo = Wire.read();
        uint8_t hi = Wire.read();
        return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
    };

    dig_T1 = readU16LE();
    dig_T2 = readS16LE();
    dig_T3 = readS16LE();

    // Skip pressure calibration values; temperature only is required.
    for (int i = 0; i < 7; ++i) {
        Wire.read();
        Wire.read();
    }

    // ctrl_meas: normal mode, temperature x2, pressure x1.
    Wire.beginTransmission(BMP280_ADDR);
    Wire.write(0xF4);
    Wire.write(0x57); // 01010111: temp x2, press x1, normal
    if (Wire.endTransmission() != 0) {
        sysStatus.bmp280OK = false;
        return false;
    }

    // config: standby 125ms, filter x16.
    Wire.beginTransmission(BMP280_ADDR);
    Wire.write(0xF5);
    Wire.write(0x70);
    Wire.endTransmission();

    // Confirm chip ID.
    Wire.beginTransmission(BMP280_ADDR);
    Wire.write(0xD0);
    if (Wire.endTransmission(false) != 0) {
        sysStatus.bmp280OK = false;
        return false;
    }
    if (Wire.requestFrom((int)BMP280_ADDR, 1) != 1 || Wire.read() != 0x58) {
        sysStatus.bmp280OK = false;
        return false;
    }

    sysStatus.bmp280OK = true;
    return true;
}

float readBMP280Temperature() {
    if (!sysStatus.bmp280OK) return lastReadTemperature;

    // Read raw temperature ADC value from 0xFA..0xFC.
    Wire.beginTransmission(BMP280_ADDR);
    Wire.write(0xFA);
    if (Wire.endTransmission(false) != 0) return lastReadTemperature;

    if (Wire.requestFrom((int)BMP280_ADDR, 3) != 3) return lastReadTemperature;

    int32_t adc_T = ((int32_t)Wire.read() << 12) |
                   ((int32_t)Wire.read() << 4) |
                   ((int32_t)Wire.read() >> 4);

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *
                    ((int32_t)dig_T2)) >> 11;

    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                    ((int32_t)dig_T3)) >> 14;

    int32_t tFine = var1 + var2;
    float t = ((tFine * 5 + 128) >> 8) / 100.0f;

    if (isfinite(t) && t > -40.0f && t < 85.0f) {
        lastReadTemperature = t;
    }
    return lastReadTemperature;
}

bool initCamera() {
    camera_config_t config;
    memset(&config, 0, sizeof(config));

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAM_PIN_D0;
    config.pin_d1 = CAM_PIN_D1;
    config.pin_d2 = CAM_PIN_D2;
    config.pin_d3 = CAM_PIN_D3;
    config.pin_d4 = CAM_PIN_D4;
    config.pin_d5 = CAM_PIN_D5;
    config.pin_d6 = CAM_PIN_D6;
    config.pin_d7 = CAM_PIN_D7;
    config.pin_xclk = CAM_PIN_XCLK;
    config.pin_pclk = CAM_PIN_PCLK;
    config.pin_vsync = CAM_PIN_VSYNC;
    config.pin_href = CAM_PIN_HREF;
    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;
    config.pin_pwdn = CAM_PIN_PWDN;
    config.pin_reset = CAM_PIN_RESET;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST;   // always newest frame -> no stale/laggy preview

    if (psramFound()) {
        // Init at VGA so the buffers fit every selectable size, then switch
        // down. 2 PSRAM buffers let capture overlap with JPEG transfer.
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
    } else {
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 15;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_DRAM;
    }

    esp_err_t err = esp_camera_init(&config);
    sysStatus.cameraOK = (err == ESP_OK);

    if (!sysStatus.cameraOK) return false;

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        if (sensor->id.PID == OV3660_PID) {
            sensor->set_vflip(sensor, 1);        // OV3660 default orientation is flipped
            sensor->set_saturation(sensor, -2);  // OV3660 is oversaturated by default
        }
        sensor->set_brightness(sensor, appSettings.brightness);
        sensor->set_contrast(sensor, appSettings.contrast);
        sensor->set_framesize(sensor, appSettings.resolution);
    }
    return true;
}

void saveSettingsToNVS() {
    preferences.begin("scd", false);
    preferences.putBool("flash", appSettings.flashEnabled);
    preferences.putUChar("res", (uint8_t)appSettings.resolution);
    preferences.putInt("bright", appSettings.brightness);
    preferences.putInt("contrast", appSettings.contrast);
    preferences.putBool("eco", appSettings.ecoMode);
    preferences.putInt("crop", appSettings.cropIndex);
    preferences.putInt("part", appSettings.partIndex);
    preferences.putInt("analysis", appSettings.analysisIndex);
    preferences.putUChar("lang", appSettings.language);
    preferences.putString("ssid", appSettings.wifiSSID);
    preferences.putString("pass", appSettings.wifiPass);
    preferences.putString("key", appSettings.apiKey);
    preferences.end();
}

void loadSettingsFromNVS() {
    memset(&appSettings, 0, sizeof(appSettings));

    appSettings.flashEnabled = true;
    appSettings.resolution = FRAMESIZE_QVGA;
    appSettings.brightness = 0;
    appSettings.contrast = 0;
    appSettings.ecoMode = false;
    appSettings.cropIndex = 0;
    appSettings.partIndex = 0;
    appSettings.analysisIndex = 0;
    appSettings.language = LANG_ENGLISH;
    strlcpy(appSettings.wifiSSID, DEFAULT_WIFI_SSID, sizeof(appSettings.wifiSSID));
    strlcpy(appSettings.wifiPass, DEFAULT_WIFI_PASSWORD, sizeof(appSettings.wifiPass));
    strlcpy(appSettings.apiKey, DEFAULT_GEMINI_KEY, sizeof(appSettings.apiKey));

    preferences.begin("scd", true);
    appSettings.flashEnabled = preferences.getBool("flash", appSettings.flashEnabled);
    appSettings.resolution = (framesize_t)preferences.getUChar("res", (uint8_t)appSettings.resolution);
    appSettings.brightness = preferences.getInt("bright", appSettings.brightness);
    appSettings.contrast = preferences.getInt("contrast", appSettings.contrast);
    appSettings.ecoMode = preferences.getBool("eco", appSettings.ecoMode);
    appSettings.cropIndex = constrain(preferences.getInt("crop", appSettings.cropIndex), 0, TOTAL_CROPS - 1);
    appSettings.partIndex = constrain(preferences.getInt("part", appSettings.partIndex), 0, TOTAL_PARTS - 1);
    appSettings.analysisIndex = constrain(preferences.getInt("analysis", appSettings.analysisIndex), 0, TOTAL_ANALYSIS - 1);
    appSettings.language = preferences.getUChar("lang", appSettings.language);
    preferences.getString("ssid", appSettings.wifiSSID, sizeof(appSettings.wifiSSID));
    preferences.getString("pass", appSettings.wifiPass, sizeof(appSettings.wifiPass));
    preferences.getString("key", appSettings.apiKey, sizeof(appSettings.apiKey));
    preferences.end();
}

// ============================================================================
// PART 2B: LIVE CAPTURE PREVIEW  
// Streams decoded camera frames to the panel with a viewfinder-style crop
// guide so the farmer can confirm framing/ratio before the real photo is
// taken. Does not touch the existing capture/base64/HTTP pipeline at all --
// it only decides WHEN the user presses "capture" to run it.
// ============================================================================

#if ENABLE_LIVE_JPEG_PREVIEW
JPEGDEC jpegDecoder;

// JPEGDEC calls this once per decoded MCU block; we stream each block
// straight to the panel so there is no full-frame RAM copy and no
// This avoids copying an entire frame into RAM.
static int jpegPreviewDrawCallback(JPEGDRAW *pDraw) {
    const int dstX = pDraw->x;
    const int dstY = pDraw->y + PREVIEW_TOP;

    // The OV3660 preview is decoded as RGB565. Clip MCU blocks that extend
    // beyond the 240px-wide panel instead of sending an oversized address
    // window to the ST7789.
    if (dstX >= 240 || dstY >= PREVIEW_TOP + PREVIEW_H) return 1;

    int drawW = pDraw->iWidth;
    int drawH = pDraw->iHeight;
    if (dstX + drawW > 240) drawW = 240 - dstX;
    if (dstY + drawH > PREVIEW_TOP + PREVIEW_H)
        drawH = (PREVIEW_TOP + PREVIEW_H) - dstY;
    if (drawW <= 0 || drawH <= 0) return 1;

    // JPEGDEC defaults to RGB565 little-endian pixels. Adafruit_ST7789's
    // writePixels() must therefore perform the SPI byte swap itself.
    tft.startWrite();
    tft.setAddrWindow(dstX, dstY, drawW, drawH);

    if (drawW == pDraw->iWidth) {
        tft.writePixels((uint16_t*)pDraw->pPixels,
                        drawW * drawH, true, false);
    } else {
        // Right-edge clipping: send only the visible pixels from each row.
        uint16_t *row = (uint16_t*)pDraw->pPixels;
        for (int y = 0; y < drawH; y++) {
            tft.writePixels(row + (y * pDraw->iWidth),
                            drawW, true, false);
        }
    }
    tft.endWrite();
    return 1;
}
#endif

// Switches the sensor to a small, fast frame size for preview streaming.
// The user's real capture resolution (appSettings.resolution) is restored
// by exitLivePreview() before the final analysis photo is taken.
void enterLivePreview() {
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_pixformat(s, PIXFORMAT_JPEG);
        s->set_framesize(s, FRAMESIZE_QVGA);
        s->set_quality(s, 14);
    }
    if (appSettings.flashEnabled) digitalWrite(PIN_FLASH, HIGH); // flash on during live preview
    previewCameraModeActive = true;
    lastPreviewFrame = 0; // force an immediate first frame
}

void exitLivePreview() {
    if (!previewCameraModeActive) return;
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, appSettings.resolution);
        s->set_quality(s, 12);
    }
    digitalWrite(PIN_FLASH, LOW); // flash off when leaving preview
    previewCameraModeActive = false;
}

// Viewfinder-style corner brackets marking the recommended crop ratio.
// Drawn fresh every frame (cheap: 8 short lines) on top of the streamed
// image so the guide never gets erased by the next camera frame.
void drawCropGuideOverlay() {
    const int gx = 26, gy = PREVIEW_TOP + 14;
    const int gw = 188, gh = PREVIEW_H - 28;
    const int armLen = 18;
    uint16_t c = COLOR_ACCENT;

    // Top-left
    tft.drawFastHLine(gx, gy, armLen, c);         tft.drawFastVLine(gx, gy, armLen, c);
    // Top-right
    tft.drawFastHLine(gx + gw - armLen, gy, armLen, c); tft.drawFastVLine(gx + gw - 1, gy, armLen, c);
    // Bottom-left
    tft.drawFastHLine(gx, gy + gh - 1, armLen, c);      tft.drawFastVLine(gx, gy + gh - armLen, armLen, c);
    // Bottom-right
    tft.drawFastHLine(gx + gw - armLen, gy + gh - 1, armLen, c); tft.drawFastVLine(gx + gw - 1, gy + gh - armLen, armLen, c);

    drawTextAligned(T("Fill the frame - press to capture", "Frame-a niraikavum - press pannunga"),
                     120, PREVIEW_TOP + PREVIEW_H + 4, COLOR_GRAY_MED, 1, ALIGN_CENTER);
}

// Grabs one small preview frame, decodes it straight to the panel, and
// draws the crop guide on top. Called from loop() while in
// APP_LIVE_PREVIEW, throttled by PREVIEW_FRAME_MS.
void renderLivePreviewFrame() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return;

#if ENABLE_LIVE_JPEG_PREVIEW
    if (jpegDecoder.openRAM(fb->buf, fb->len, jpegPreviewDrawCallback)) {
        // Explicitly keep JPEGDEC output in its documented default RGB565
        // little-endian format for Adafruit_ST7789::writePixels().
        jpegDecoder.setPixelType(RGB565_LITTLE_ENDIAN);
        jpegDecoder.decode(0, 0, 0);
        jpegDecoder.close();
    }
#else
    // Fallback build (ENABLE_LIVE_JPEG_PREVIEW=0): no decode library, so
    // just keep the framing-guide screen static instead of live video.
    tft.fillRect(0, PREVIEW_TOP, 240, PREVIEW_H, COLOR_PANEL_DARK);
    drawIcon(ICON_CAMERA, 120, PREVIEW_TOP + PREVIEW_H / 2, COLOR_ACCENT, COLOR_PRIMARY_DARK);
#endif

    esp_camera_fb_return(fb);
    drawCropGuideOverlay();
}

// ============================================================================
// PART 3: AI PIPELINE  (HTTPS + JSON) -- FIXED
//
//  Root causes of the permanent "HTTP error" in the original firmware:
//   1) HTTPClient::begin(url) was called with a plain client for an
//      https:// URL. Without a WiFiClientSecure (and setInsecure() or a
//      root CA), the TLS handshake can never complete, so httpCode is
//      always negative and the screen always shows an HTTP error.
//   2) The shipped Gemini key was not in a valid key format (real keys
//      start with "AIza..."), so even a correct TLS setup would still be
//      rejected by Google's servers.
//  Both are fixed below, and failures are now categorized into clear,
//  farmer-readable messages instead of a raw error code.
// ============================================================================

static bool looksLikeValidGeminiKey(const char* k) {
    size_t n = strlen(k);
    if (n < 30) return false;
    // Google is transitioning from AIza... (old, working through Sept 2026)
    // to AQ.... (new, issued from June 2026 onwards). Both formats are valid
    // on native generativelanguage.googleapis.com endpoints (which this code uses).
    bool isAizaFormat = (strncmp(k, "AIza", 4) == 0);
    bool isAqFormat   = (strncmp(k, "AQ.", 3) == 0);
    return isAizaFormat || isAqFormat;
}

// Pulls a short, human-readable message out of a Gemini error response body,
// e.g. {"error":{"code":400,"message":"API key not valid."}}
static String extractApiErrorMessage(const String& response) {
    if (response.length() == 0) return "";
    DynamicJsonDocument edoc(2048);
    if (deserializeJson(edoc, response)) return "";
    const char* msg = edoc["error"]["message"] | "";
    String m = String(msg);
    if (m.length() > 90) m = m.substring(0, 90) + "...";
    return m;
}

// Builds a prompt that asks Gemini to answer like it is talking to a
// smallholder farmer: short sentences, no jargon, concrete local actions.
// also sends the selected plant part, analysis type, and language,
// and the BMP280 temperature reading -- exactly the context the device
// has available, as requested.
static String buildAIPrompt(int cropIndex, int partIndex, int analysisIndex,
                             float ambientTemp, uint8_t language) {
    String prompt =
        "You are AgriLens, a professional field-level agricultural decision-support assistant. "
        "Analyze the attached image conservatively and evidence-first. Do not force an agricultural diagnosis.\n"
        "MANDATORY DECISION PIPELINE:\n"
        "A) FIRST identify the main visible subject/object in plain words: lemon fruit, tomato leaf, rice plant, television, person, animal, soil, or unclear scene.\n"
        "B) Decide whether the main subject is relevant to crop/agricultural analysis. A non-agricultural object must NOT be diagnosed as a crop.\n"
        "C) If agricultural, identify the crop/plant ONLY from visible evidence. The selected crop below is only a hint and may be wrong. Never copy it blindly.\n"
        "D) Cross-check using visible shape, leaf/fruit/flower/stem characteristics, plant structure and context. If evidence is insufficient or conflicting, lower confidence instead of guessing.\n"
        "E) Check image quality before health diagnosis: blur, darkness, obstruction, distance, tiny subject, poor framing, or multiple competing subjects.\n"
        "F) Only when the agricultural subject and image quality are sufficient, analyze visible symptoms and possible disease, pest, nutrient, environmental or other stress.\n"
        "G) Clearly distinguish observations from conclusions. Never invent a symptom, disease, crop, object, or treatment.\n"
        "H) If non-agricultural, mention the detected object explicitly and state professionally that crop-health analysis is not applicable.\n"
        "I) If agricultural but not reliably identifiable, mention what is visible and explain what additional view would improve identification.\n"
        "Rules: use short farmer-friendly sentences. action must be 1-3 concrete steps under 180 characters. prevention must be one short practical habit under 120 characters. "
        "severity must be exactly Low, Medium, High, or Unknown. confidence is integer 0-100. "
        "Return ONLY valid JSON with exactly these keys: crop, diagnosis, severity, confidence, action, prevention, temp_note.\n"
        "The crop field must name the actual visible agricultural subject when reliably identifiable. For a non-agricultural image, put the detected object plus a concise label such as 'Television (non-agricultural object)'. "
        "Never call a lemon a potato merely because potato was selected in settings.\n"
        "The diagnosis field must describe health only when justified. For non-agricultural objects say crop-health diagnosis is not applicable. "
        "For insufficient evidence say reliable diagnosis cannot be made from this image.\n"
        "Never confidently name unsupported crops or diseases.\n";

    if (language == LANG_TAMIL) {
        prompt += "Write diagnosis, action, prevention and temp_note in TANGLISH using Latin letters only.\n";
    } else {
        prompt += "Write all fields in natural, simple English.\n";
    }

    prompt += "Selected crop from device settings: " + String(CROPS_LIST[cropIndex]) + ". ";
    prompt += "This selection is only a hint; verify it against the image and override it when the image disagrees. ";
    prompt += "Selected plant part: " + String(PART_LIST_EN[partIndex]) + ". ";
    prompt += "Selected analysis focus: " + String(ANALYSIS_LIST_EN[analysisIndex]) + ". ";
    prompt += "Measured ambient temperature from BMP280: " + String(ambientTemp, 1) + " C. ";
    prompt += "Use temperature only as supporting context; never use it alone to diagnose disease. ";
    prompt += "If Auto Detect is selected, determine the visible part and problem category from evidence. ";
    prompt += "If only a fruit or leaf is visible and exact plant identity cannot be verified, explicitly state the limitation instead of guessing.";
    return prompt;
}

static bool parseGeminiReport(const String &response, float fallbackTemp, int cropIndex,
                               int partIndex, int analysisIndex) {
    DynamicJsonDocument doc(16384);
    DeserializationError err = deserializeJson(doc, response);
    if (err) return false;

    String text = doc["candidates"][0]["content"]["parts"][0]["text"] | "";
    if (text.length() == 0) return false;

    int a = text.indexOf('{');
    int b = text.lastIndexOf('}');
    if (a < 0 || b <= a) return false;

    String jsonText = text.substring(a, b + 1);

    DynamicJsonDocument rdoc(8192);
    if (deserializeJson(rdoc, jsonText)) return false;

    currentReport.crop = (const char*)(rdoc["crop"] | CROPS_LIST[cropIndex]);
    currentReport.part = PART_LIST_EN[partIndex];
    currentReport.analysisType = ANALYSIS_LIST_EN[analysisIndex];
    currentReport.temperature = fallbackTemp;
    currentReport.diagnosis = (const char*)(rdoc["diagnosis"] | "Unable to determine");
    currentReport.severity = (const char*)(rdoc["severity"] | "Unknown");
    currentReport.confidence = rdoc["confidence"] | 0;
    currentReport.action = (const char*)(rdoc["action"] | "Consult a local agricultural expert.");
    currentReport.prevention = (const char*)(rdoc["prevention"] | "Monitor the crop and maintain good field hygiene.");
    currentReport.temp_note = (const char*)(rdoc["temp_note"] | "Use the measured temperature as supporting context only.");
    currentReport.timestamp = millis();
    currentReport.isValid = true;
    return true;
}

// Runs entirely on a background FreeRTOS task pinned to the opposite core
// from the Arduino loop(). It touches ONLY g_b64Buffer/g_capturedTemp/
// g_capturedCropIndex (read-only, snapshotted before the task starts),
// currentReport / lastErrorMessage (write-only here), and netTaskState
// (written last, exactly once). It never touches Wire/I2C or the TFT.
static void netWorkerTask(void* pvParameters) {
    bool ok = false;
    {
        String prompt = buildAIPrompt(g_capturedCropIndex, g_capturedPartIndex,
                                       g_capturedAnalysisIndex, g_capturedTemp,
                                       g_capturedLanguage);

        DynamicJsonDocument req(32768);
        JsonArray contents = req["contents"].to<JsonArray>();
        JsonObject item = contents.add<JsonObject>();
        JsonArray parts = item["parts"].to<JsonArray>();

        JsonObject textPart = parts.add<JsonObject>();
        textPart["text"] = prompt;

        JsonObject imagePart = parts.add<JsonObject>();
        JsonObject inlineData = imagePart["inline_data"].to<JsonObject>();
        inlineData["mime_type"] = "image/jpeg";
        inlineData["data"] = g_b64Buffer;

        String body;
        serializeJson(req, body);

        if (g_b64Buffer) {
            heap_caps_free(g_b64Buffer);
            g_b64Buffer = nullptr;
        }

        NetworkClientSecure secureClient;
        secureClient.setInsecure(); // Board has no RTC for cert-date checks;
                                    // the link is still TLS-encrypted.

        HTTPClient http;
        String url = String(GEMINI_ENDPOINT_URL) + "?key=" + String(appSettings.apiKey);

        if (http.begin(secureClient, url)) {
            http.setConnectTimeout(15000);
            http.setTimeout(45000);
            http.addHeader("Content-Type", "application/json");

            int httpCode = http.POST(body);

            if (httpCode <= 0) {
                lastErrorMessage = "No response from AI server.\nCheck your Wi-Fi signal\nand try again.";
            } else if (httpCode < 200 || httpCode >= 300) {
                String response = http.getString();
                String apiMsg = extractApiErrorMessage(response);
                if (httpCode == 400 || httpCode == 403) {
                    lastErrorMessage = "AI key rejected (" + String(httpCode) + ").\n" +
                        (apiMsg.length() ? apiMsg : "Check your Gemini API key\nin Settings > Wi-Fi Info.");
                } else {
                    lastErrorMessage = "AI server error " + String(httpCode) + ".\n" +
                        (apiMsg.length() ? apiMsg : "Please try again shortly.");
                }
            } else {
                String response = http.getString();
                if (parseGeminiReport(response, g_capturedTemp, g_capturedCropIndex,
                                       g_capturedPartIndex, g_capturedAnalysisIndex)) {
                    ok = true;
                } else {
                    lastErrorMessage = "Could not read AI reply.\nPlease try scanning again.";
                }
            }
            http.end();
        } else {
            lastErrorMessage = "Could not reach AI server.\nCheck your Wi-Fi and try again.";
        }
        // secureClient / http / req / body go out of scope HERE, so their
        // destructors run before the task is deleted below.
    }

    if (g_b64Buffer) {
        heap_caps_free(g_b64Buffer);
        g_b64Buffer = nullptr;
    }

    netTaskState = ok ? NET_DONE_OK : NET_DONE_FAIL;
    vTaskDelete(NULL);
}

// Captures + encodes the photo (fast, main thread), then kicks off the
// background network task and returns immediately. Any failure BEFORE the
// task starts is handled synchronously so the caller can show the error
// screen right away without ever touching the network.
bool startCaptureAndAnalysis() {
    // If we arrived here from the live preview, the sensor may still be set
    // to the small preview frame size -- put it back to the user's chosen
    // capture resolution before grabbing the real analysis photo.
    exitLivePreview();

    if (!sysStatus.cameraOK && !initCamera()) {
        lastErrorMessage = "Camera not ready.\nRestart the device.";
        return false;
    }
    if (WiFi.status() != WL_CONNECTED) {
        lastErrorMessage = "Wi-Fi not connected.\nCheck Settings > Wi-Fi Info.";
        return false;
    }
    if (!looksLikeValidGeminiKey(appSettings.apiKey)) {
        lastErrorMessage = "AI key missing or invalid.\nOpen Settings > Wi-Fi Info\nand set a valid Gemini key\n(starts AIza... or AQ.).";
        return false;
    }

    switchState(APP_CAPTURING);

    // MEMORY-SAFE AI CAPTURE:
    // Always encode a compact QVGA JPEG for Gemini. This prevents the
    // large temporary Base64 buffer from exhausting PSRAM during encoding.
    // The user's display/preview resolution setting is NOT changed.
    sensor_t *captureSensor = esp_camera_sensor_get();
    if (captureSensor) {
        captureSensor->set_pixformat(captureSensor, PIXFORMAT_JPEG);
        captureSensor->set_framesize(captureSensor, FRAMESIZE_VGA);
        captureSensor->set_quality(captureSensor, 15);
    }

    if (appSettings.flashEnabled) {
        digitalWrite(PIN_FLASH, HIGH);
        delay(120);
    }

    // Drop the first frame after a size/quality change (may be stale).
    { camera_fb_t *old = esp_camera_fb_get(); if (old) esp_camera_fb_return(old); }
    camera_fb_t *fb = esp_camera_fb_get();

    if (appSettings.flashEnabled) digitalWrite(PIN_FLASH, LOW);

    if (!fb) {
        lastErrorMessage = "Camera capture failed.\nPlease try again.";
        return false;
    }

    // Sample temperature and crop selection HERE, on the main thread, so the
    // background task never has to touch the I2C bus or shared settings.
    g_capturedTemp = readBMP280Temperature();
    g_capturedCropIndex = appSettings.cropIndex;
    g_capturedPartIndex = appSettings.partIndex;
    g_capturedAnalysisIndex = appSettings.analysisIndex;
    g_capturedLanguage = appSettings.language;

    size_t outLen = 0;
    size_t b64Len = 4 * ((fb->len + 2) / 3);
    // Prefer PSRAM, but gracefully fall back to internal RAM for the small
    // QVGA JPEG if PSRAM allocation is temporarily unavailable.
    char *b64 = (char*)heap_caps_malloc(b64Len + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!b64) {
        b64 = (char*)heap_caps_malloc(b64Len + 1, MALLOC_CAP_8BIT);
    }
    if (!b64) {
        esp_camera_fb_return(fb);
        lastErrorMessage = "Not enough memory for image encoding.\nTry capture again.";
        return false;
    }

    int rc = mbedtls_base64_encode((unsigned char*)b64, b64Len + 1,
                                   &outLen, fb->buf, fb->len);
    esp_camera_fb_return(fb);

    if (rc != 0) {
        heap_caps_free(b64);
        lastErrorMessage = "Image encoding failed.\nPlease try again.";
        return false;
    }
    b64[outLen] = '\0';

    g_b64Buffer = b64;
    g_b64Len = outLen;

    switchState(APP_PROCESSING);

    netTaskState = NET_RUNNING;
    BaseType_t started = xTaskCreatePinnedToCore(
        netWorkerTask, "scdNetTask", 16384, nullptr, 1, &netTaskHandle, 0);

    if (started != pdPASS) {
        netTaskState = NET_IDLE;
        if (g_b64Buffer) { heap_caps_free(g_b64Buffer); g_b64Buffer = nullptr; }
        lastErrorMessage = "Could not start AI task.\nPlease try again.";
        return false;
    }

    return true; // Caller stays on APP_PROCESSING; loop() finishes the flow.
}

// ============================================================================
// PART 4: REPORT CANVAS (smooth-scrolling result screen)
// ============================================================================

void pushReportViewport(int offsetY);
void buildReportCanvas(const DiagnosticReport& report);

// ============================================================================
// ENHANCED STATE MANAGEMENT WITH NAVIGATION STACK
// ============================================================================
// This function now manages:
// 1. Navigation history for back button support
// 2. Proper state transitions
// 3. UI element reset (menu selections, etc.)
// 4. Hardware setup for specific states (camera, preview, etc.)
//
// The navigation stack allows users to return to previous screens via the
// professional back button rendered in the header.
// ============================================================================
void switchState(AppState newState) {
    // Clean exit from live preview mode if transitioning away
    if (currentState == APP_LIVE_PREVIEW && newState != APP_LIVE_PREVIEW &&
        newState != APP_CAPTURING) {
        exitLivePreview(); // Clean up preview resources
    }

    // Track navigation history for back button support (except temporary states)
    bool shouldTrackHistory = true;
    if (newState == APP_PROCESSING || newState == APP_CAPTURING ||
        newState == APP_BOOTING || newState == APP_DIAGNOSTICS) {
        shouldTrackHistory = false;  // Don't track transient states
    }
    
    if (shouldTrackHistory && newState != currentState) {
        pushNavState(newState);
    }

    // Perform state transition
    currentState = newState;

    // Reset menu selection for list-based screens
    if (newState == APP_MENU_MAIN ||
        newState == APP_MENU_CROP_SELECT ||
        newState == APP_MENU_PART_SELECT ||
        newState == APP_MENU_ANALYSIS_SELECT ||
        newState == APP_MENU_HISTORY ||
        newState == APP_MENU_SETTINGS ||
        newState == APP_MENU_CAMERA_SETTINGS) {
        menuSelection = 0;
    }

    // Enter live preview mode with camera stream
    if (newState == APP_LIVE_PREVIEW) {
        enterLivePreview();
    }

    // Initialize processing animation
    if (newState == APP_PROCESSING) {
        animFrame = 0;
    }

    // Build diagnostic report canvas for scrollable display
    if (newState == APP_DISPLAY_RESULT || newState == APP_HISTORY_VIEW) {
        freeReportCanvas();
        buildReportCanvas(currentReport);
        return;  // buildReportCanvas calls renderUI() internally
    }

    // Render the new UI state
    renderUI();
}

void buildReportCanvas(const DiagnosticReport& report) {
    freeReportCanvas();

    // The report is rendered once into PSRAM and then the 240x320 viewport
    // is streamed to the ST7789.  1400px gives Gemini plenty of room.
    reportCanvasHeight = 1400;
    reportCanvas = new PSRAMCanvas(240, reportCanvasHeight);

    if (!reportCanvas || !reportCanvas->ok()) {
        freeReportCanvas();
        return;
    }

    reportCanvas->clear(COLOR_BG_DARK);

    const int LEFT = 10;
    const int CARD_W = 220;
    const int INNER_X = 18;
    const int TEXT_W = 204;
    const int GAP = 8;

    int y = 8;

    // ---------- REPORT TITLE ----------
    canvasCard(*reportCanvas, LEFT, y, CARD_W, 36,
               COLOR_PRIMARY_DARK, COLOR_ACCENT, 8);
    canvasTextAligned(*reportCanvas, "AI DIAGNOSIS", 120, y + 8,
                      COLOR_WHITE, 2, ALIGN_CENTER);
    y += 44;

    // ---------- SUMMARY ----------
    const int summaryY = y;
    const int summaryH = 82;

    canvasCard(*reportCanvas, LEFT, summaryY, CARD_W, summaryH,
               COLOR_CREAM, COLOR_ACCENT, 8);

    canvasTextAligned(*reportCanvas, "CROP", INNER_X, summaryY + 7,
                      COLOR_PRIMARY, 1, ALIGN_LEFT);
    canvasTextAligned(*reportCanvas, report.crop, INNER_X, summaryY + 20,
                      COLOR_TEXT_DARK, 2, ALIGN_LEFT);

    uint16_t sevColor = COLOR_PRIMARY;
    if (report.severity.equalsIgnoreCase("Medium")) sevColor = COLOR_AMBER;
    if (report.severity.equalsIgnoreCase("High")) sevColor = COLOR_RED;

    canvasTextAligned(*reportCanvas,
                      "Severity: " + report.severity,
                      INNER_X, summaryY + 48,
                      sevColor, 1, ALIGN_LEFT);

    canvasTextAligned(*reportCanvas,
                      String(report.confidence) + "% confidence",
                      222, summaryY + 48,
                      COLOR_TEXT_DARK, 1, ALIGN_RIGHT);

    canvasTextAligned(*reportCanvas,
                      "Temp " + String(report.temperature, 1) + " C",
                      INNER_X, summaryY + 64,
                      COLOR_GRAY_MED, 1, ALIGN_LEFT);

    if (report.part.length() || report.analysisType.length()) {
        canvasTextAligned(*reportCanvas,
                          report.part + " / " + report.analysisType,
                          222, summaryY + 64,
                          COLOR_GRAY_MED, 1, ALIGN_RIGHT);
    }

    y += summaryH + GAP;

    // Helper lambda for all text cards.  Height is calculated BEFORE drawing
    // the card, so text can never be covered by a later card.
    auto drawReportTextCard = [&](const char* title,
                                  const String& body,
                                  uint16_t accent,
                                  uint8_t bodySize,
                                  int minHeight) {
        int bodyLinesH = canvasWrappedHeight(*reportCanvas, body,
                                              TEXT_W, bodySize);

        int cardH = max(minHeight, 29 + bodyLinesH + 10);
        // Keep every card safely inside the PSRAM canvas.
        if (y + cardH > reportCanvasHeight - 12)
            cardH = reportCanvasHeight - 12 - y;

        canvasCard(*reportCanvas, LEFT, y, CARD_W, cardH,
                   COLOR_CREAM, accent, 8);

        canvasTextAligned(*reportCanvas, title, INNER_X, y + 7,
                          accent, 1, ALIGN_LEFT);

        int textY = y + 24;
        canvasTextWrapped(*reportCanvas, body, INNER_X, textY,
                          TEXT_W, COLOR_TEXT_DARK, bodySize);

        y += cardH + GAP;
    };

    // ---------- DIAGNOSIS ----------
    drawReportTextCard("DIAGNOSIS",
                       report.diagnosis,
                       COLOR_ACCENT,
                       2, 66);

    // ---------- ACTION ----------
    drawReportTextCard("WHAT TO DO NOW",
                       report.action,
                       COLOR_RED,
                       1, 76);

    // ---------- PREVENTION ----------
    drawReportTextCard("PREVENTION TIP",
                       report.prevention,
                       COLOR_PRIMARY,
                       1, 66);

    // ---------- TEMPERATURE ----------
    drawReportTextCard("TEMPERATURE NOTE",
                       report.temp_note,
                       COLOR_AMBER,
                       1, 60);

    // ---------- FOOTER ----------
    canvasCard(*reportCanvas, LEFT, y, CARD_W, 36,
               COLOR_PRIMARY_DARK, COLOR_ACCENT, 8);
    canvasTextAligned(*reportCanvas, "Long press: back to menu",
                      120, y + 10, COLOR_WHITE, 1, ALIGN_CENTER);
    y += 44;

    // Exact scroll range. No blank-page jump at the bottom.
    maxScrollY = max(0, y - 320);
    maxScrollY = min(maxScrollY, reportCanvasHeight - 320);
    scrollY = 0;
    scrollPhase = SCROLL_DOWN;
    scrollPauseStart = millis();

    reportFrameReady = true;

    // Always show the first frame immediately.
    pushReportViewport(scrollY);
}

void pushReportViewport(int offsetY) {
    if (!reportCanvas || !reportCanvas->ok()) return;

    offsetY = constrain(offsetY, 0, maxScrollY);

    const uint16_t *frame =
        reportCanvas->buf + ((size_t)offsetY * 240);

    const uint32_t pixelCount = (uint32_t)240 * 320;

    tft.startWrite();
    tft.setAddrWindow(0, 0, 240, 320);
    tft.writePixels((uint16_t *)frame, pixelCount, true);
    tft.endWrite();
}

// ============================================================================
// PART 5: SCREEN RENDERER  -- new dashboard-style UI
// Layout grid used everywhere below (so nothing ever overlaps):
//   y   0- 38  : title bar (logo + screen name)
//   y  38- 56  : status ribbon (wifi + crop + temperature)
//   y  56- 64  : divider / breathing room
//   y  64-310  : content area
//   y 306-318  : optional footer hint line
// ============================================================================

// ============================================================================
// PROFESSIONAL BACK BUTTON RENDERING SYSTEM
// ============================================================================
// Renders a professional back navigation button in the top-left corner of
// screens that support backward navigation. Uses clinical blue color scheme
// with smooth rounding and intuitive arrow icon design.
// ============================================================================
void drawBackButton() {
    if (!canGoBack()) return;  // Only show if navigation history available
    
    // Professional back button dimensions
    const int btnX = 4;
    const int btnY = 6;
    const int btnW = 28;
    const int btnH = 28;
    const int btnRadius = 3;
    
    // Draw rounded button background
    tft.fillRoundRect(btnX, btnY, btnW, btnH, btnRadius, COLOR_PRIMARY);
    
    // Draw left arrow icon for back navigation
    // Arrow design: < shape pointing left
    tft.drawFastVLine(btnX + 14, btnY + 8, 12, COLOR_WHITE);   // Right edge of arrow
    tft.drawFastVLine(btnX + 12, btnY + 9, 10, COLOR_WHITE);   // Diagonal line 1
    tft.drawFastVLine(btnX + 16, btnY + 9, 10, COLOR_WHITE);   // Diagonal line 2
    tft.drawPixel(btnX + 13, btnY + 10, COLOR_WHITE);          // Arrow tip
    tft.drawPixel(btnX + 15, btnY + 10, COLOR_WHITE);          // Arrow tip
    
    // Store button location for touch detection
    // Touch area: btnX to btnX+btnW, btnY to btnY+btnH
}

// ============================================================================
// ENHANCED HEADER WITH BACK BUTTON SUPPORT
// ============================================================================
// Professional header now includes:
// - Back button when navigation history available
// - Logo badge with product name
// - WiFi signal strength indicator
// - Current crop selection
// - Environmental temperature readout
// ============================================================================
void drawHeader(const String& title) {
    tft.fillRect(0, 0, 240, 38, COLOR_PRIMARY_DARK);
    
    // Draw logo badge (offset for back button if needed)
    int logoX = canGoBack() ? 36 : 16;
    drawLogoBadge(logoX, 19, 0.32f);
    
    // Draw header title (centered, accounting for back button)
    drawTextAligned(title, 130, 12, COLOR_WHITE, 2, ALIGN_CENTER);
    
    // Draw back button in top-left if available
    drawBackButton();
    
    tft.drawFastHLine(0, 38, 240, COLOR_ACCENT);

    // Status ribbon: WiFi, crop, temperature
    tft.fillRect(0, 39, 240, 17, COLOR_PANEL_DARK);
    drawWiFiSignalBar(8, 43, sysStatus.wifiOK ? WiFi.RSSI() : -100);
    drawTextAligned(CROPS_LIST[appSettings.cropIndex], 120, 43, COLOR_GRAY_LIGHT, 1, ALIGN_CENTER);
    drawTextAligned(String(readBMP280Temperature(), 1) + "C", 232, 43, COLOR_ACCENT, 1, ALIGN_RIGHT);
    tft.drawFastHLine(0, 56, 240, COLOR_GRAY_MED);
}

void renderUI() {
    tft.fillScreen(COLOR_BG_DARK);

    switch (currentState) {

        case APP_BOOTING: {
            // Professional diagnostic system splash screen
            // Keep the opening screen simple and easy to read
            drawLogoBadge(120, 55, 1.2f);
            
            // Main product name - emphasized in primary teal/blue
            drawTextAligned("AGRILENS", 120, 120, COLOR_PRIMARY, 3, ALIGN_CENTER);
            
            // Leave room for the project tagline
            drawTextAligned("Professional Agricultural Diagnostics", 120, 162, COLOR_GRAY_MED, 1, ALIGN_CENTER);
            drawTextAligned(PRODUCT_VERSION, 120, 172, COLOR_GRAY_LIGHT, 0, ALIGN_CENTER);
            
            // Horizontal accent line
            tft.drawFastHLine(58, 185, 124, COLOR_ACCENT);
            tft.fillRoundRect(78, 183, 84, 4, 2, COLOR_ACCENT);
            
            // System capability status - three-column indicator
            drawCard(24, 203, 192, 50, COLOR_PANEL_DARK, COLOR_GRAY_MED, UI_RADIUS);
            drawTextAligned("VISION", 54, 208, COLOR_GRAY_LIGHT, 1, ALIGN_CENTER);
            drawTextAligned("AI", 120, 208, COLOR_GRAY_LIGHT, 1, ALIGN_CENTER);
            drawTextAligned("SENSOR", 186, 208, COLOR_GRAY_LIGHT, 1, ALIGN_CENTER);
            drawTextAligned("READY", 54, 230, COLOR_PRIMARY, 1, ALIGN_CENTER);
            drawTextAligned("READY", 120, 230, COLOR_PRIMARY, 1, ALIGN_CENTER);
            drawTextAligned("READY", 186, 230, COLOR_PRIMARY, 1, ALIGN_CENTER);

            // Status message
            drawTextAligned("Initializing Diagnostic System", 120, 274, COLOR_GRAY_LIGHT, 1, ALIGN_CENTER);
            break;
        }

        case APP_DIAGNOSTICS: {
            drawHeader(T("SYSTEM CHECK", "SYSTEM CHECK"));
            drawCard(10, 64, 220, 210, COLOR_CREAM, COLOR_PRIMARY, UI_RADIUS);

            int ry = 78;
            const int rowGap = 24;

            drawTextAligned("PSRAM Memory", 20, ry, COLOR_TEXT_DARK, 1, ALIGN_LEFT);
            drawTextAligned(sysStatus.psramOK ? "OK" : "ABSENT", 220, ry, sysStatus.psramOK ? COLOR_PRIMARY : COLOR_RED, 1, ALIGN_RIGHT);
            ry += rowGap;

            drawTextAligned("Camera (OV3660)", 20, ry, COLOR_TEXT_DARK, 1, ALIGN_LEFT);
            drawTextAligned(sysStatus.cameraOK ? "OK" : "FAILED", 220, ry, sysStatus.cameraOK ? COLOR_PRIMARY : COLOR_RED, 1, ALIGN_RIGHT);
            ry += rowGap;

            drawTextAligned("BMP280 Sensor", 20, ry, COLOR_TEXT_DARK, 1, ALIGN_LEFT);
            drawTextAligned("OK", 220, ry, COLOR_PRIMARY, 1, ALIGN_RIGHT);
            ry += rowGap;

            drawTextAligned("Wi-Fi Link", 20, ry, COLOR_TEXT_DARK, 1, ALIGN_LEFT);
            drawTextAligned(sysStatus.wifiOK ? "CONNECTED" : "OFFLINE", 220, ry, sysStatus.wifiOK ? COLOR_PRIMARY : COLOR_RED, 1, ALIGN_RIGHT);
            ry += rowGap;

            drawTextAligned("Free Heap", 20, ry, COLOR_TEXT_DARK, 1, ALIGN_LEFT);
            drawTextAligned(String(sysStatus.freeHeap / 1024) + " KB", 220, ry, COLOR_PRIMARY, 1, ALIGN_RIGHT);
            ry += rowGap;

            drawTextAligned("Free PSRAM", 20, ry, COLOR_TEXT_DARK, 1, ALIGN_LEFT);
            drawTextAligned(String(sysStatus.freePsram / 1024) + " KB", 220, ry, COLOR_PRIMARY, 1, ALIGN_RIGHT);
            ry += rowGap + 4;

            drawProgressBar(20, ry, 200, 12, 100, COLOR_PRIMARY, COLOR_GRAY_LIGHT);

            drawTextAligned("System Ready", 120, 288, COLOR_CREAM, 1, ALIGN_CENTER);
            break;
        }

        case APP_MENU_MAIN: {
            drawHeader(T("AGRILENS", "AGRILENS"));

            // Agricultural dashboard: field-style cards instead of a generic menu.
            drawTextAligned("Farm Health Dashboard", 16, 68, COLOR_TEXT_DARK, 1, ALIGN_LEFT);
            drawTextAligned(CROPS_LIST[appSettings.cropIndex], 224, 68, COLOR_PRIMARY, 1, ALIGN_RIGHT);

            String items[] = {
                T("New Scan", "Puthu Scan"),
                T("Select Crop", "Payir Therivu"),
                T("History", "Varalaru"),
                T("Settings", "Amaippugal"),
                T("About", "Parpom")
            };
            const int icons[] = {ICON_CAMERA, ICON_LEAF, ICON_CLOCK, ICON_GEAR, ICON_INFO};

            const int itemH = 38, gap = 7, startY = 82;
            for (int i = 0; i < 5; i++) {
                int y = startY + i * (itemH + gap);
                drawMenuRow(y, itemH, i == menuSelection, icons[i], items[i]);
            }

            // Bottom field status strip.
            tft.fillRoundRect(10, 306 - 24, 220, 20, UI_RADIUS, COLOR_PANEL_DARK);
            drawIcon(ICON_CHECK, 24, 292, COLOR_GREEN, COLOR_GREEN);
            drawTextAligned(sysStatus.wifiOK ? "FIELD SYSTEM ONLINE" : "OFFLINE MODE",
                            40, 287, COLOR_CREAM, 1, ALIGN_LEFT);
            break;
        }

        case APP_MENU_CROP_SELECT: {
            drawHeader(T("SELECT CROP", "PAYIR THERIVU"));

            const int itemH = 32, gap = 6, startY = 64;
            for (int i = 0; i < TOTAL_CROPS; i++) {
                int y = startY + i * (itemH + gap);
                bool isSel = (i == menuSelection);
                bool isActive = (i == appSettings.cropIndex);

                drawMenuRow(y, itemH, isSel, ICON_LEAF, CROPS_LIST[i]);
                if (isActive) {
                    drawIcon(ICON_CHECK, 205, y + itemH / 2,
                             isSel ? COLOR_WHITE : COLOR_PRIMARY,
                             isSel ? COLOR_WHITE : COLOR_PRIMARY);
                }
            }
            
            // ✓ UNIVERSAL BACK TO DASHBOARD OPTION
            bool isBackSelected = (menuSelection == TOTAL_CROPS);
            drawMenuRow(64 + TOTAL_CROPS * (itemH + gap), itemH, isBackSelected, -1, "< Back to Dashboard");
            break;
        }

        case APP_MENU_PART_SELECT: {
            drawHeader(T("SELECT PLANT PART", "PART-A THERIVU"));

            const int itemH = 32, gap = 6, startY = 64;
            for (int i = 0; i < TOTAL_PARTS; i++) {
                int y = startY + i * (itemH + gap);
                bool isSel = (i == menuSelection);
                bool isActive = (i == appSettings.partIndex);

                drawMenuRow(y, itemH, isSel, ICON_LEAF, T_part(i));
                if (isActive) {
                    drawIcon(ICON_CHECK, 205, y + itemH / 2,
                             isSel ? COLOR_WHITE : COLOR_PRIMARY,
                             isSel ? COLOR_WHITE : COLOR_PRIMARY);
                }
            }
            
            // ✓ UNIVERSAL BACK TO DASHBOARD OPTION
            bool isBackSelected = (menuSelection == TOTAL_PARTS);
            drawMenuRow(64 + TOTAL_PARTS * (itemH + gap), itemH, isBackSelected, -1, "< Back to Dashboard");
            break;
        }

        case APP_MENU_ANALYSIS_SELECT: {
            drawHeader(T("SELECT ANALYSIS", "SCAN VAGAI THERIVU"));

            const int itemH = 32, gap = 6, startY = 64;
            for (int i = 0; i < TOTAL_ANALYSIS; i++) {
                int y = startY + i * (itemH + gap);
                bool isSel = (i == menuSelection);
                bool isActive = (i == appSettings.analysisIndex);

                drawMenuRow(y, itemH, isSel, ICON_WARNING, T_analysis(i));
                if (isActive) {
                    drawIcon(ICON_CHECK, 205, y + itemH / 2,
                             isSel ? COLOR_WHITE : COLOR_PRIMARY,
                             isSel ? COLOR_WHITE : COLOR_PRIMARY);
                }
            }
            
            // ✓ UNIVERSAL BACK TO DASHBOARD OPTION
            bool isBackSelected = (menuSelection == TOTAL_ANALYSIS);
            drawMenuRow(64 + TOTAL_ANALYSIS * (itemH + gap), itemH, isBackSelected, -1, "< Back to Dashboard");
            break;
        }

        case APP_LIVE_PREVIEW: {
            drawHeader(T("FRAME YOUR SHOT", "PHOTO EDUKKA"));
            tft.fillRect(0, PREVIEW_TOP, 240, PREVIEW_H, COLOR_PANEL_DARK);
            drawCropGuideOverlay();
            drawFooterHint(T("Short=back  Long=capture", "Short=pinnal  Long=capture"));
            break;
        }

        case APP_MENU_HISTORY: {
            drawHeader(T("SCAN HISTORY", "VARALARU"));

            if (historyCount == 0) {
                drawIcon(ICON_CLOCK, 120, 140, COLOR_GRAY_MED, COLOR_BG_DARK);
                drawTextAligned("No scans yet", 120, 165, COLOR_GRAY_LIGHT, 2, ALIGN_CENTER);
                drawTextAligned("Scan a leaf from the Dashboard", 120, 192, COLOR_GRAY_MED, 1, ALIGN_CENTER);
                drawFooterHint("Long press to return");
            } else {
                const int itemH = 54, gap = 8, startY = 64;
                for (int i = 0; i < historyCount; i++) {
                    int y = startY + i * (itemH + gap);
                    bool isSel = (i == menuSelection);

                    uint16_t cardBg = isSel ? COLOR_PRIMARY : COLOR_CREAM;
                    uint16_t textColor = isSel ? COLOR_WHITE : COLOR_TEXT_DARK;
                    uint16_t subColor = isSel ? COLOR_CREAM : COLOR_GRAY_MED;

                    drawCard(10, y, 220, itemH, cardBg, isSel ? COLOR_ACCENT : COLOR_GRAY_LIGHT, UI_RADIUS);
                    drawIcon(ICON_LEAF, 28, y + 17, isSel ? COLOR_WHITE : COLOR_PRIMARY, isSel ? COLOR_PRIMARY : COLOR_CREAM);
                    drawTextAligned("Scan " + String(i + 1) + ": " + historyRAM[i].crop, 44, y + 9, textColor, 1, ALIGN_LEFT);

                    String diag = historyRAM[i].diagnosis;
                    if (diag.length() > 30) diag = diag.substring(0, 30) + "...";
                    drawTextAligned(diag, 44, y + 30, subColor, 1, ALIGN_LEFT);
                }

                bool backSel = (menuSelection == historyCount);
                int backY = startY + historyCount * (itemH + gap);
                drawMenuRow(backY, 40, backSel, -1, "Back to Dashboard");
            }
            break;
        }

        case APP_MENU_SETTINGS: {
            drawHeader(T("SETTINGS", "AMAIPPUGAL"));

            String options[] = {
                appSettings.flashEnabled ? T("Flash LED: ON", "Flash: ON") : T("Flash LED: OFF", "Flash: OFF"),
                T("Camera Setup", "Camera Amaippu"),
                appSettings.ecoMode ? T("Eco Mode: ON", "Eco Mode: ON") : T("Eco Mode: OFF", "Eco Mode: OFF"),
                appSettings.language == LANG_TAMIL ? "Language: Tamil" : "Language: English",
                T("Wi-Fi Info", "Wi-Fi Vivaram"),
                T("Back to Dashboard", "Dashboard-ku Pogavum")
            };

            const int itemH = 32, gap = 6, startY = 64;
            for (int i = 0; i < 6; i++) {
                int y = startY + i * (itemH + gap);
                drawMenuRow(y, itemH, i == menuSelection, -1, options[i]);
            }
            break;
        }

        case APP_MENU_CAMERA_SETTINGS: {
            drawHeader(T("CAMERA SETUP", "CAMERA AMAIPPU"));

            String resStr = "Resolution: VGA";
            if (appSettings.resolution == FRAMESIZE_QVGA) resStr = "Resolution: QVGA";
            if (appSettings.resolution == FRAMESIZE_SVGA) resStr = "Resolution: SVGA";

            String options[] = {
                resStr,
                "Brightness: " + String(appSettings.brightness),
                "Contrast: " + String(appSettings.contrast),
                "Back to Settings"
            };

            const int itemH = 42, gap = 10, startY = 70;
            for (int i = 0; i < 4; i++) {
                int y = startY + i * (itemH + gap);
                drawMenuRow(y, itemH, i == menuSelection, -1, options[i]);
            }
            break;
        }

        case APP_WIFI_INFO: {
            drawHeader(T("WI-FI STATUS", "WI-FI VIVARAM"));
            drawCard(10, 64, 220, 210, COLOR_CREAM, COLOR_PRIMARY, UI_RADIUS);

            drawTextAligned("Network Name", 20, 78, COLOR_GRAY_MED, 1, ALIGN_LEFT);
            drawTextAligned(appSettings.wifiSSID, 20, 92, COLOR_TEXT_DARK, 2, ALIGN_LEFT);

            drawTextAligned("Device IP", 20, 122, COLOR_GRAY_MED, 1, ALIGN_LEFT);
            drawTextAligned(WiFi.localIP().toString(), 20, 136, COLOR_TEXT_DARK, 2, ALIGN_LEFT);

            drawTextAligned("Signal Strength", 20, 166, COLOR_GRAY_MED, 1, ALIGN_LEFT);
            int rssi = WiFi.RSSI();
            drawTextAligned(String(rssi) + " dBm", 20, 180, (rssi < -80) ? COLOR_RED : COLOR_PRIMARY, 2, ALIGN_LEFT);

            drawTextAligned("AI Key Status", 20, 210, COLOR_GRAY_MED, 1, ALIGN_LEFT);
            bool keyOk = looksLikeValidGeminiKey(appSettings.apiKey);
            drawTextAligned(keyOk ? "Configured" : "Not set / invalid", 20, 224, keyOk ? COLOR_PRIMARY : COLOR_RED, 2, ALIGN_LEFT);

            drawMenuRow(280, 34, true, -1, "Back to Settings");
            break;
        }

        case APP_MENU_INFO: {
            drawHeader(T("ABOUT DEVICE", "DEVICE VIVARAM"));
            drawCard(10, 64, 220, 210, COLOR_CREAM, COLOR_PRIMARY, UI_RADIUS);

            drawTextAligned(PRODUCT_NAME, 120, 76, COLOR_PRIMARY, 2, ALIGN_CENTER);
            drawTextAligned("PRO Edition - 2026", 120, 98, COLOR_TEXT_DARK, 1, ALIGN_CENTER);

            drawTextAligned("Presented by", 20, 122, COLOR_GRAY_MED, 1, ALIGN_LEFT);
            drawTextAligned("Tiruselvan", 20, 134, COLOR_ACCENT, 1, ALIGN_LEFT);
            drawTextAligned("Sanjay", 20, 150, COLOR_ACCENT, 1, ALIGN_LEFT);
            drawTextAligned("Rishi", 20, 162, COLOR_ACCENT, 1, ALIGN_LEFT);

            drawTextAligned("Hardware", 20, 164, COLOR_GRAY_MED, 1, ALIGN_LEFT);
            drawTextAligned("ESP32-S3 N16R8", 20, 178, COLOR_TEXT_DARK, 1, ALIGN_LEFT);
            drawTextAligned("OV3660 camera + BMP280", 20, 192, COLOR_TEXT_DARK, 1, ALIGN_LEFT);

            drawTextAligned("AI Engine", 20, 214, COLOR_GRAY_MED, 1, ALIGN_LEFT);
            drawTextAligned("Google Gemini Vision API", 20, 228, COLOR_TEXT_DARK, 1, ALIGN_LEFT);

            drawMenuRow(280, 34, true, -1, "Back to Settings");
            break;
        }

        case APP_CAPTURING: {
            drawLogoBadge(120, 96, 1.2f);
            drawTextAligned("CAPTURING", 120, 158, COLOR_AMBER, 3, ALIGN_CENTER);
            drawTextAligned("Hold the leaf steady", 120, 196, COLOR_CREAM, 2, ALIGN_CENTER);
            drawIcon(ICON_CAMERA, 120, 236, COLOR_ACCENT, COLOR_PRIMARY_DARK);
            break;
        }

        case APP_PROCESSING: {
            drawLogoBadge(120, 64, 0.85f);
            drawTextAligned("ANALYZING", 120, 100, COLOR_ACCENT, 3, ALIGN_CENTER);
            drawTextAligned("AI Doctor is examining", 120, 132, COLOR_GRAY_LIGHT, 1, ALIGN_CENTER);
            // Spinner + status phrase are drawn by updateProcessingSpinner(),
            // called here for the first frame and then every ~90ms from
            // loop() while the background task is running -- WITHOUT ever
            // touching fillScreen(), so there is no flicker.
            updateProcessingSpinner();
            break;
        }

        case APP_ERROR_SCREEN: {
            // Deliberately NOT using drawHeader() here: this is a distinct
            // "alert" bar style (red, not green) so it reads instantly as a
            // fault state, without duplicating/overlapping the normal logo.
            tft.fillRect(0, 0, 240, 38, COLOR_RED);
            drawIcon(ICON_WARNING, 20, 19, COLOR_WHITE, COLOR_RED);
            drawTextAligned(T("SYSTEM ALERT", "ALERT"), 132, 12, COLOR_WHITE, 2, ALIGN_CENTER);
            tft.drawFastHLine(0, 38, 240, COLOR_WHITE);

            tft.fillRect(0, 39, 240, 17, COLOR_PANEL_DARK);
            drawTextAligned(T("Tap the button to continue", "Button-a press pannunga"), 120, 43, COLOR_GRAY_LIGHT, 1, ALIGN_CENTER);
            tft.drawFastHLine(0, 56, 240, COLOR_GRAY_MED);

            drawCard(10, 70, 220, 190, COLOR_CREAM, COLOR_RED, UI_RADIUS);
            drawIcon(ICON_WARNING, 120, 100, COLOR_RED, COLOR_CREAM);

            int ey = 122;
            drawTextWrapped(lastErrorMessage, 26, ey, 188, COLOR_TEXT_DARK, 1);

            drawFooterHint(T("Press the button to continue", "Continue panna button press pannunga"));
            break;
        }

        case APP_HISTORY_VIEW:
        case APP_DISPLAY_RESULT: {
            // Report framebuffer is pushed by switchState()/loop().
            break;
        }
    }
}

// Redraws ONLY the spinner + status-phrase area of the processing screen.
// This is the fix for the flicker/freeze that made the old UI feel choppy:
// the old code called the full renderUI()->fillScreen() every 90ms AND
// blocked the whole loop() during the network call. Now the screen is
// drawn once, this function only touches a small fixed region, and the
// network call runs on a separate task so this keeps animating smoothly.
void updateProcessingSpinner() {
    tft.fillRect(94, 154, 52, 52, COLOR_BG_DARK);
    drawSpinner(120, 180, 20, animFrame, COLOR_ACCENT);

    static const char* PHRASES[] = {
        "Reading your photo...",
        "Checking leaf pattern...",
        "Consulting AI Doctor...",
        "Preparing your report..."
    };
    int phraseIdx = (animFrame / 15) % 4;
    tft.fillRect(10, 215, 220, 16, COLOR_BG_DARK);
    drawTextAligned(PHRASES[phraseIdx], 120, 217, COLOR_CREAM, 1, ALIGN_CENTER);

    if (WiFi.RSSI() < -80) {
        tft.fillRect(10, 240, 220, 14, COLOR_BG_DARK);
        drawTextAligned("Weak signal - please wait", 120, 242, COLOR_AMBER, 1, ALIGN_CENTER);
    }
}

/* ============================================================================
 * PART 6 OF 6: POWER-ON SELF TEST (POST), NAVIGATION ENGINE, SETUP & LOOP
 * ============================================================================*/

void runPOSTDiagnostics() {
    switchState(APP_DIAGNOSTICS);

    sysStatus.freeHeap = ESP.getFreeHeap();
    sysStatus.freePsram = ESP.getFreePsram();
    sysStatus.psramOK = psramFound();
    sysStatus.bootTimeMs = millis();

    renderUI();

    playSoundPreset("boot");
    delay(2000); // Allow user to inspect POST diagnostic screen
}

// ============================================================================
// PROFESSIONAL BUTTON ACTION HANDLER WITH BACK NAVIGATION
// ============================================================================
// This handler manages:
// 1. Back button detection (button press in top-left area when available)
// 2. Navigation stack-based screen transitions
// 3. Menu selection advancement (short press)
// 4. Item confirmation and state transitions (long press)
//
// Professional flow: Users can press back from any screen to return to
// the previous navigation state without losing context.
// ============================================================================
void handleButtonAction(bool isLongPress) {
    playSoundPreset("select");

    // PROFESSIONAL BACK BUTTON LOGIC
    // If available, back button provides direct navigation to previous screen
    if (canGoBack() && !isLongPress) {
        // Check if button was pressed in back button area (roughly top-left 35px x 35px)
        // For now, we use the simpler approach: any short press navigates back in
        // list-based screens, or specific handling in dialog screens.
        // This can be enhanced with touch input to detect exact button position.
        
        // For certain screens, short press always means "go back"
        if (currentState == APP_ERROR_SCREEN || currentState == APP_DIAGNOSTICS) {
            switchState(APP_MENU_MAIN);
            return;
        }

        if (currentState == APP_DISPLAY_RESULT || currentState == APP_HISTORY_VIEW) {
            switchState(APP_MENU_MAIN);
            return;
        }

        if (currentState == APP_WIFI_INFO || currentState == APP_MENU_INFO) {
            switchState(APP_MENU_SETTINGS);
            return;
        }
    }

    // Live preview has its own two-way mapping (short=back, long=capture)
    // This overrides the generic pattern for this screen
    if (currentState == APP_LIVE_PREVIEW) {
        if (!isLongPress) {
            switchState(APP_MENU_ANALYSIS_SELECT); // cancel framing, go back
        } else {
            if (!startCaptureAndAnalysis()) {
                playSoundPreset("error");
                switchState(APP_ERROR_SCREEN);
            }
            // else: stays on APP_PROCESSING; loop() completes the flow.
        }
        return;
    }

    if (!isLongPress) {
        // Short Press: Advance Item Selection
        switch (currentState) {
            case APP_MENU_MAIN: menuSelection = (menuSelection + 1) % 5; break;
            case APP_MENU_CROP_SELECT: menuSelection = (menuSelection + 1) % (TOTAL_CROPS + 1); break;  // +1 for Back button
            case APP_MENU_PART_SELECT: menuSelection = (menuSelection + 1) % (TOTAL_PARTS + 1); break;  // +1 for Back button
            case APP_MENU_ANALYSIS_SELECT: menuSelection = (menuSelection + 1) % (TOTAL_ANALYSIS + 1); break;  // +1 for Back button
            case APP_MENU_HISTORY: menuSelection = (menuSelection + 1) % (historyCount + 1); break;
            case APP_MENU_SETTINGS: menuSelection = (menuSelection + 1) % 6; break;
            case APP_MENU_CAMERA_SETTINGS: menuSelection = (menuSelection + 1) % 4; break;
            default: break;
        }
        renderUI();
    } else {
        // Long Press: Select Highlighted Option
        switch (currentState) {
            case APP_MENU_MAIN: {
                if (menuSelection == 0) {
                    // NEW flow: crop is already chosen elsewhere/persisted,
                    // so a scan now goes Part -> Analysis -> Live Preview
                    // -> capture, instead of capturing immediately.
                    switchState(APP_MENU_PART_SELECT);
                } else if (menuSelection == 1) switchState(APP_MENU_CROP_SELECT);
                else if (menuSelection == 2) switchState(APP_MENU_HISTORY);
                else if (menuSelection == 3) switchState(APP_MENU_SETTINGS);
                else if (menuSelection == 4) switchState(APP_MENU_INFO);
                break;
            }

            case APP_MENU_CROP_SELECT: {
                // ✓ BACK TO DASHBOARD BUTTON
                if (menuSelection == TOTAL_CROPS) {
                    switchState(APP_MENU_MAIN);
                    return;
                }
                appSettings.cropIndex = menuSelection;
                saveSettingsToNVS();
                switchState(APP_MENU_MAIN);
                break;
            }

            case APP_MENU_PART_SELECT: {
                // ✓ BACK TO DASHBOARD BUTTON
                if (menuSelection == TOTAL_PARTS) {
                    switchState(APP_MENU_MAIN);
                    return;
                }
                appSettings.partIndex = menuSelection;
                saveSettingsToNVS();
                switchState(APP_MENU_ANALYSIS_SELECT);
                break;
            }

            case APP_MENU_ANALYSIS_SELECT: {
                // ✓ BACK TO DASHBOARD BUTTON
                if (menuSelection == TOTAL_ANALYSIS) {
                    switchState(APP_MENU_MAIN);
                    return;
                }
                appSettings.analysisIndex = menuSelection;
                saveSettingsToNVS();
                switchState(APP_LIVE_PREVIEW);
                break;
            }

            case APP_MENU_HISTORY: {
                if (menuSelection == historyCount) {
                    switchState(APP_MENU_MAIN);
                } else if (menuSelection < historyCount) {
                    activeHistoryIndex = menuSelection;
                    currentReport = historyRAM[activeHistoryIndex];
                    switchState(APP_HISTORY_VIEW);
                }
                break;
            }

            case APP_MENU_SETTINGS: {
                if (menuSelection == 0) {
                    appSettings.flashEnabled = !appSettings.flashEnabled;
                    saveSettingsToNVS();
                } else if (menuSelection == 1) {
                    switchState(APP_MENU_CAMERA_SETTINGS);
                    return;
                } else if (menuSelection == 2) {
                    appSettings.ecoMode = !appSettings.ecoMode;
                    saveSettingsToNVS();
                } else if (menuSelection == 3) {
                    appSettings.language = (appSettings.language == LANG_TAMIL) ? LANG_ENGLISH : LANG_TAMIL;
                    saveSettingsToNVS();
                } else if (menuSelection == 4) {
                    switchState(APP_WIFI_INFO);
                    return;
                } else if (menuSelection == 5) {
                    switchState(APP_MENU_MAIN);
                    return;
                }
                renderUI();
                break;
            }

            case APP_MENU_CAMERA_SETTINGS: {
                sensor_t *s = esp_camera_sensor_get();
                if (menuSelection == 0) {
                    if (appSettings.resolution == FRAMESIZE_QQVGA) {
                        appSettings.resolution = FRAMESIZE_QVGA;
                    } else if (appSettings.resolution == FRAMESIZE_QVGA) {
                        appSettings.resolution = FRAMESIZE_VGA;
                    } else {
                        appSettings.resolution = FRAMESIZE_QQVGA;
                    }
                    if (s) s->set_framesize(s, appSettings.resolution);
                    saveSettingsToNVS();
                } else if (menuSelection == 1) {
                    appSettings.brightness++;
                    if (appSettings.brightness > 2) appSettings.brightness = -2;
                    if (s) s->set_brightness(s, appSettings.brightness);
                    saveSettingsToNVS();
                } else if (menuSelection == 2) {
                    appSettings.contrast++;
                    if (appSettings.contrast > 2) appSettings.contrast = -2;
                    if (s) s->set_contrast(s, appSettings.contrast);
                    saveSettingsToNVS();
                } else if (menuSelection == 3) {
                    switchState(APP_MENU_SETTINGS);
                    return;
                }
                renderUI();
                break;
            }

            default: break;
        }
    }
}

// ============================================================================
// SYSTEM BOOTSTRAPPING (setup)
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(100); // Allow Serial connection to stabilize

    // Initialize Pin Modes
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_FLASH, OUTPUT);
    pinMode(TFT_BL, OUTPUT);

    digitalWrite(PIN_FLASH, LOW);
    digitalWrite(TFT_BL, HIGH); // Enable Screen Backlight

    // Hardware SPI & ST7789 Configuration
    SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
    tft.init(240, 320);
    tft.invertDisplay(false);
    tft.setSPISpeed(80000000); // Faster, smoother ST7789 transfers
    tft.setRotation(2); // Portrait Mode - 180° corrected for physical display mounting
    tft.fillScreen(COLOR_BG_DARK);

    // Load NVS Persistent Memory Configuration
    loadSettingsFromNVS();

    // Boot Screen Splash Animation
    switchState(APP_BOOTING);
    delay(1200);

    // Hardware Peripherals Initialization
    Wire.begin(I2C_SDA, I2C_SCL);
    initBMP280();
    yield(); // Yield CPU execution to reset Watchdog Timer

    initCamera();
    yield();

    // Connect to Local Wi-Fi Safely
    WiFi.mode(WIFI_STA); // Ensure Wi-Fi mode is strictly station mode
    WiFi.begin(appSettings.wifiSSID, appSettings.wifiPass);

    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 20000) {
        delay(10); // Non-blocking delay to feed ESP32 Watchdog Timer (TG1WDT)
    }
    sysStatus.wifiOK = (WiFi.status() == WL_CONNECTED);

    // Run POST Self-Diagnostics Screen
    runPOSTDiagnostics();

    switchState(APP_MENU_MAIN);
}

// ============================================================================
// MAIN EXECUTION LOOP (loop)
// ============================================================================
void loop() {
    unsigned long now = millis();

    // 1. NON-BLOCKING PHYSICAL BUTTON CONTROLLER
    int btnState = digitalRead(PIN_BUTTON);
    if (btnState == LOW) {
        if (!buttonIsPressed) {
            buttonIsPressed = true;
            buttonPressStart = now;
            buttonHandled = false;
        } else if (!buttonHandled && (now - buttonPressStart >= LONG_PRESS_MS)) {
            buttonHandled = true;
            handleButtonAction(true); // Long Press Event
        }
    } else {
        if (buttonIsPressed) {
            if (!buttonHandled && (now - buttonPressStart >= DEBOUNCE_MS)) {
                buttonHandled = true;
                handleButtonAction(false); // Short Press Event
            }
            buttonIsPressed = false;
        }
    }

    // 2. LIVE CAPTURE PREVIEW -- non-blocking, throttled.
    // Streams a small decoded camera frame to the panel with the crop
    // guide overlay every PREVIEW_FRAME_MS while the farmer is framing
    // the shot, without ever touching the AI/network pipeline.
    if (currentState == APP_LIVE_PREVIEW &&
        (now - lastPreviewFrame >= PREVIEW_FRAME_MS)) {
        lastPreviewFrame = now;
        renderLivePreviewFrame();
    }

    // 3. AI PROCESSING SCREEN -- non-blocking.
    // The network request runs entirely on a background task (see
    // netWorkerTask). This loop only (a) animates the spinner while the
    // task is running and (b) reacts once when the task finishes, so the
    // UI never freezes and never flickers during a scan.
    if (currentState == APP_PROCESSING) {
        if (netTaskState == NET_RUNNING) {
            if (now - lastAnimTime >= 90) {
                lastAnimTime = now;
                animFrame++;
                updateProcessingSpinner();
            }
        } else if (netTaskState == NET_DONE_OK) {
            netTaskState = NET_IDLE;
            if (historyCount < 3) {
                historyRAM[historyCount++] = currentReport;
            } else {
                historyRAM[0] = historyRAM[1]; historyRAM[1] = historyRAM[2]; historyRAM[2] = currentReport;
            }
            switchState(APP_DISPLAY_RESULT);
        } else if (netTaskState == NET_DONE_FAIL) {
            netTaskState = NET_IDLE;
            playSoundPreset("error");
            switchState(APP_ERROR_SCREEN);
        }
    }

    // 4. FRAMEBUFFER-BASED SMOOTH, CONTINUOUSLY LOOPING REPORT SCROLLING
    // Only the already-rendered viewport is streamed; no fillScreen(), no
    // repeated text/card rendering and therefore no page-cutting effect.
    // The report now auto-repeats: scroll down, pause at the bottom so the
    // farmer can finish reading, then scroll smoothly back to the top and
    // pause there too, instead of freezing once it hits the bottom.
    if ((currentState == APP_DISPLAY_RESULT || currentState == APP_HISTORY_VIEW) &&
        reportFrameReady && (now - lastReportFrame >= REPORT_FRAME_MS)) {

        lastReportFrame = now;

        switch (scrollPhase) {
            case SCROLL_DOWN:
                scrollY += SCROLL_STEP_PX;
                if (scrollY >= maxScrollY) {
                    scrollY = maxScrollY;
                    scrollPhase = SCROLL_PAUSE_BOTTOM;
                    scrollPauseStart = now;
                }
                pushReportViewport(scrollY);
                break;

            case SCROLL_PAUSE_BOTTOM:
                if (now - scrollPauseStart >= SCROLL_PAUSE_MS) {
                    scrollPhase = (maxScrollY > 0) ? SCROLL_UP : SCROLL_PAUSE_TOP;
                    if (scrollPhase == SCROLL_PAUSE_TOP) scrollPauseStart = now;
                }
                break;

            case SCROLL_UP:
                scrollY -= SCROLL_STEP_PX;
                if (scrollY <= 0) {
                    scrollY = 0;
                    scrollPhase = SCROLL_PAUSE_TOP;
                    scrollPauseStart = now;
                }
                pushReportViewport(scrollY);
                break;

            case SCROLL_PAUSE_TOP:
                if (now - scrollPauseStart >= SCROLL_PAUSE_MS) {
                    scrollPhase = (maxScrollY > 0) ? SCROLL_DOWN : SCROLL_PAUSE_TOP;
                    if (scrollPhase == SCROLL_DOWN) scrollPauseStart = now;
                }
                break;
        }
    }
}

