/*
 * =====================================================================
 * FILE    : limit_switch.ino
 * PERAN   : Baca limit switch langsung via digitalRead().
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
    LIMIT_PIN_Y_BAWAH,
    LIMIT_PIN_X_MUNDUR,
    LIMIT_PIN_ARMBOX_DEPAN,
    LIMIT_PIN_ARMBOX_BELAKANG,
};

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
//  READ — langsung baca GPIO
// =====================================================================

bool readLimitSwitch(uint8_t limitId) {
    if (limitId >= LIMIT_COUNT) return false;
    return (digitalRead(limitPins[limitId]) == LOW);
}
