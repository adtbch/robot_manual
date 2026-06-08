/*
 * =====================================================================
 * FILE    : servo_presets.ino
 * PERAN   : Penyimpanan preset sudut servo (X+Dpad) via NVS.
 *           Save/reset dikirim dari Controller via command byte.
 * =====================================================================
 */

#include "robot_config.h"

// ============================================================
// NVS KEYS
// ============================================================
static const char* NVS_SERVO_PRESET = "servo_preset";

// ============================================================
// DEFAULT PRESETS (derajat)
// ============================================================
static const int8_t DEFAULT_PRESET_UP    = 95;
static const int8_t DEFAULT_PRESET_DOWN  = 0;
static const int8_t DEFAULT_PRESET_LEFT  = 5;
static const int8_t DEFAULT_PRESET_RIGHT = 185;

// ============================================================
// STATE
// ============================================================
static int8_t gPresetUp    = DEFAULT_PRESET_UP;
static int8_t gPresetDown  = DEFAULT_PRESET_DOWN;
static int8_t gPresetLeft  = DEFAULT_PRESET_LEFT;
static int8_t gPresetRight = DEFAULT_PRESET_RIGHT;

// ============================================================
// NVS FUNCTIONS
// ============================================================

void servoPresetsLoad() {
    Preferences prefs;
    prefs.begin(NVS_SERVO_PRESET, true);  // read-only
    gPresetUp    = prefs.getChar("p_up",    DEFAULT_PRESET_UP);
    gPresetDown  = prefs.getChar("p_down",  DEFAULT_PRESET_DOWN);
    gPresetLeft  = prefs.getChar("p_left",  DEFAULT_PRESET_LEFT);
    gPresetRight = prefs.getChar("p_right", DEFAULT_PRESET_RIGHT);
    prefs.end();
}

void servoPresetsSave() {
    Preferences prefs;
    prefs.begin(NVS_SERVO_PRESET, false);  // read-write
    prefs.putChar("p_up",    gPresetUp);
    prefs.putChar("p_down",  gPresetDown);
    prefs.putChar("p_left",  gPresetLeft);
    prefs.putChar("p_right", gPresetRight);
    prefs.end();
}

void servoPresetsReset() {
    gPresetUp    = DEFAULT_PRESET_UP;
    gPresetDown  = DEFAULT_PRESET_DOWN;
    gPresetLeft  = DEFAULT_PRESET_LEFT;
    gPresetRight = DEFAULT_PRESET_RIGHT;
    servoPresetsSave();
    Serial.println("[PRESETS] Reset ke default & disimpan ke NVS");
}

// ============================================================
// SAVE PRESET BY DIRECTION
// ============================================================

void servoPresetsSaveUp(int angle) {
    gPresetUp = (int8_t)angle;
    servoPresetsSave();
    Serial.printf("[PRESETS] Up   = %d° (disimpan)\n", angle);
}

void servoPresetsSaveDown(int angle) {
    gPresetDown = (int8_t)angle;
    servoPresetsSave();
    Serial.printf("[PRESETS] Down = %d° (disimpan)\n", angle);
}

void servoPresetsSaveLeft(int angle) {
    gPresetLeft = (int8_t)angle;
    servoPresetsSave();
    Serial.printf("[PRESETS] Left = %d° (disimpan)\n", angle);
}

void servoPresetsSaveRight(int angle) {
    gPresetRight = (int8_t)angle;
    servoPresetsSave();
    Serial.printf("[PRESETS] Right= %d° (disimpan)\n", angle);
}

// ============================================================
// GET PRESET BY DIRECTION
// ============================================================

int servoPresetsGetUp()    { return (int)gPresetUp; }
int servoPresetsGetDown()  { return (int)gPresetDown; }
int servoPresetsGetLeft()  { return (int)gPresetLeft; }
int servoPresetsGetRight() { return (int)gPresetRight; }
