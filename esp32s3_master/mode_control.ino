/*
 * =====================================================================
 * FILE    : mode_control.ino
 * PERAN   : Manajemen mode robot (GRIPPING / ARM_BOX).
 *           Toggle via tombol Options, disimpan permanen di NVS
 *           (Preferences) sehingga tetap bertahan saat reset/power loss.
 *
 * NVS KEY:
 *   "robot_mode" : uint8_t (0 = GRIPPING, 1 = ARM_BOX)
 * =====================================================================
 */

#include "robot_config.h"

// =====================================================================
//  VARIABEL GLOBAL (dideklarasikan extern di robot_config.h)
// =====================================================================

RobotMode currentMode        = MODE_GRIPPING;

// =====================================================================
//  VARIABEL LOKAL
// =====================================================================

static Preferences nvsPrefs;
static const char* kNvsNamespace = "robot_cfg";
static const char* kNvsKeyMode   = "robot_mode";

// =====================================================================
//  FUNGSI: LOAD MODE DARI NVS
// =====================================================================

/**
 * Baca mode terakhir dari NVS (Preferences).
 * Jika belum ada value tersimpan, gunakan default MODE_GRIPPING.
 */
static void mode_load() {
    nvsPrefs.begin(kNvsNamespace, false);  // read-write
    uint8_t saved = nvsPrefs.getUChar(kNvsKeyMode, MODE_GRIPPING);
    nvsPrefs.end();

    if (saved <= MODE_ARM_BOX) {
        currentMode = static_cast<RobotMode>(saved);
    } else {
        currentMode = MODE_GRIPPING;
    }
    Serial.printf("[MODE] Load dari NVS: %s (val=%u)\n", mode_name(), currentMode);
}

// =====================================================================
//  FUNGSI: SAVE MODE KE NVS
// =====================================================================

/**
 * Simpan mode saat ini ke NVS agar tetap tersimpan saat reset.
 */
static void mode_save() {
    nvsPrefs.begin(kNvsNamespace, false);
    nvsPrefs.putUChar(kNvsKeyMode, currentMode);
    nvsPrefs.end();
    Serial.printf("[MODE] Save ke NVS: %s (val=%u)\n", mode_name(), currentMode);
}

// =====================================================================
//  FUNGSI: INISIALISASI MODE
// =====================================================================

/**
 * Load mode dari NVS. Dipanggil di setup().
 */
void mode_init() {
    mode_load();
}

// =====================================================================
//  FUNGSI: TOGGLE MODE
// =====================================================================

/**
 * Ganti mode: GRIPPING ↔ ARM_BOX, lalu simpan ke NVS.
 */
void mode_toggle() {
    if (currentMode == MODE_GRIPPING) {
        currentMode = MODE_ARM_BOX;
    } else {
        currentMode = MODE_GRIPPING;
    }
    mode_save();
    Serial.printf("[MODE] >>> BERUBAH KE: %s <<<\n", mode_name());
}

// =====================================================================
//  FUNGSI: NAMA MODE (untuk serial print)
// =====================================================================

/**
 * Kembalikan nama mode dalam format string.
 */
const char* mode_name() {
    switch (currentMode) {
        case MODE_GRIPPING: return "GRIPPING";
        case MODE_ARM_BOX:  return "ARM_BOX";
        default:            return "UNKNOWN";
    }
}
