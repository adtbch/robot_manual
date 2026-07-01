/*
 * =====================================================================
 * FILE    : waypoint.ino
 * PERAN   : Zone 1 waypoints + AllianceColor global state + NVS.
 *
 * ZONE 1 WAYPOINTS:
 *   [0] Ambil tongkat
 *   [1] Prepare
 *   [2] Stab
 *
 * ALLIANCE COLOR:
 *   Toggle via tombol BOOT (GPIO 0), indikator RGB LED.
 *   Disimpan ke NVS (Preferences) — permanen saat power cycle.
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "config.h"
#include "forest.h"
#include "odom.h"
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>

// =====================================================================
//  STRUCT
// =====================================================================

struct Zone1Waypoint {
    float   x;
    float   y;
    int16_t speed_rpm;
};

// =====================================================================
//  ZONE 1 WAYPOINTS — x,y placeholder, isi nanti
// =====================================================================

constexpr int ZONE1_WP_COUNT = 3;

const Zone1Waypoint ZONE1_WP[ZONE1_WP_COUNT] = {
    {0.0f, 0.0f, 100},  // [0] ambil tongkat
    {0.0f, 0.0f, 100},  // [1] prepare
    {0.0f, 0.0f, 100},  // [2] stab
};

// =====================================================================
//  ALLIANCE COLOR — global state + NVS
// =====================================================================

AllianceColor gAllianceColor = AllianceColor::RED;

namespace {
constexpr const char* NVS_NAMESPACE    = "alliance";
constexpr const char* NVS_KEY_COLOR    = "color";

// BOOT button
constexpr uint8_t BOOT_BTN_PIN = 0;
bool gBootBtnPrev = false;

// RGB LED (WS2812B NeoPixel)
constexpr uint8_t  RGB_LED_PIN  = 48;
constexpr uint8_t  RGB_NUM_LEDS = 1;
constexpr uint8_t  LED_BRIGHTNESS = 50;

Adafruit_NeoPixel strip(RGB_NUM_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

Preferences gPrefs;

void nvsLoadAllianceColor() {
    gPrefs.begin(NVS_NAMESPACE, true);  // read-only
    uint8_t val = gPrefs.getUChar(NVS_KEY_COLOR, 1);  // default 1 = RED
    gPrefs.end();

    if (val <= static_cast<uint8_t>(AllianceColor::RED)) {
        gAllianceColor = static_cast<AllianceColor>(val);
    } else {
        gAllianceColor = AllianceColor::RED;
    }
}

void nvsSaveAllianceColor() {
    gPrefs.begin(NVS_NAMESPACE, false);  // read-write
    gPrefs.putUChar(NVS_KEY_COLOR, static_cast<uint8_t>(gAllianceColor));
    gPrefs.end();
}

void updateRgbLed() {
    if (gAllianceColor == AllianceColor::BLUE) {
        strip.setPixelColor(0, strip.Color(0, 0, 255));
    } else {
        strip.setPixelColor(0, strip.Color(255, 0, 0));
    }
    strip.show();
}

void toggleAllianceColor() {
    gAllianceColor = (gAllianceColor == AllianceColor::BLUE)
        ? AllianceColor::RED
        : AllianceColor::BLUE;
    nvsSaveAllianceColor();
    updateRgbLed();
    odomApplyAlliance();
    forestApplyAlliance(gAllianceColor);

    Serial.printf("Alliance: %s\n", allianceLabel(gAllianceColor));
}

} // anonymous namespace

// =====================================================================
//  SETUP — panggil dari setup()
// =====================================================================

void setupAlliance() {
    pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

    strip.begin();
    strip.show();
    strip.setBrightness(LED_BRIGHTNESS);

    nvsLoadAllianceColor();
    updateRgbLed();

    Serial.printf("Alliance loaded: %s\n",
        gAllianceColor == AllianceColor::BLUE ? "BLUE" : "RED");
}

// =====================================================================
//  TICK — panggil di loop(), edge detect BOOT button
// =====================================================================

void allianceTick() {
    const bool btnNow = (digitalRead(BOOT_BTN_PIN) == LOW);
    if (btnNow && !gBootBtnPrev) {
        toggleAllianceColor();
    }
    gBootBtnPrev = btnNow;
}
