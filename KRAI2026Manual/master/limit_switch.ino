/*
 * =====================================================================
 * FILE    : limit_switch.ino
 * PERAN   : Baca limit switch dengan double-read untuk anti-noise.
 *           NON-BLOCKING — pakai Jeda untuk debounce 2ms.
 *
 * SAFETY  : Limit switch harus double-read dengan 2ms delay.
 *           Motor noise bisa cause false trigger.
 *
 * CARA PAKAI:
 *   1. Panggil updateLimitSwitches() di setiap loop()
 *   2. Baca hasilnya dengan readLimitSwitch(index)
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "limit_switch.h"

// =====================================================================
//  STATE
// =====================================================================

namespace {

constexpr uint8_t limitPins[LIMIT_COUNT] = {
    LIMIT_SWITCH_1,
    LIMIT_SWITCH_2,
    LIMIT_SWITCH_3,
    LIMIT_SWITCH_4
};

// Status validated (hasil double-read)
bool limitState[LIMIT_COUNT] = {};

// State machine per switch untuk double-read
struct LimitReadState {
    bool waitingForSecond = false;   // sedang tunggu bacaan kedua
    bool firstValue       = false;   // hasil baca pertama
    Jeda jeda;                       // timer 2ms debounce
};

LimitReadState readState[LIMIT_COUNT];

} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupLimits() {
    for (size_t i = 0; i < LIMIT_COUNT; i++) {
        pinMode(limitPins[i], INPUT_PULLUP);  // active LOW
    }
}

// =====================================================================
//  UPDATE — panggil di setiap loop()
// =====================================================================

void updateLimitSwitches() {
    for (size_t i = 0; i < LIMIT_COUNT; i++) {
        if (!readState[i].waitingForSecond) {
            // Baca pertama — simpan, mulai timer
            readState[i].firstValue = (digitalRead(limitPins[i]) == LOW);
            readState[i].jeda.reset();
            readState[i].waitingForSecond = true;
        } else {
            // Tunggu 2ms via Jeda (non-blocking)
            if (readState[i].jeda.check(2)) {
                // Baca kedua — bandingkan
                bool second = (digitalRead(limitPins[i]) == LOW);
                limitState[i] = (readState[i].firstValue && second);
                readState[i].waitingForSecond = false;
            }
        }
    }
}

// =====================================================================
//  READ — return status terakhir
// =====================================================================

bool readLimitSwitch(uint8_t index) {
    if (index >= LIMIT_COUNT) {
        return false;
    }
    return limitState[index];
}
