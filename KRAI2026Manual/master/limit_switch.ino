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
    LIMIT_SWITCH_X1,
    LIMIT_SWITCH_X2,
    LIMIT_SWITCH_X3,
    LIMIT_SWITCH_X4
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

bool readLimitSwitch(uint8_t index) {
    if (index >= LIMIT_COUNT) return false;
    return (digitalRead(limitPins[index]) == LOW);
}
