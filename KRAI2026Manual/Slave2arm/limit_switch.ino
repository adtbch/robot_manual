/*
 * =====================================================================
 * FILE    : limit_switch.ino
 * PERAN   : Baca limit switch langsung via digitalRead().
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#include "limit_switch.h"

// =====================================================================
//  STATE
// =====================================================================

namespace {

constexpr uint8_t limitPins[] = {
    LIMIT_PIN_ARMBOX_DEPAN,
    LIMIT_PIN_ARMBOX_BELAKANG,
    LIMIT_PIN_ARMBOX_TURUN,
    // LIMIT_PIN_UNUSED,  // ponytail: deadcode
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
    if (index >= LIMIT_COUNT) {
        return false;
    }
    return (digitalRead(limitPins[index]) == LOW);
}
