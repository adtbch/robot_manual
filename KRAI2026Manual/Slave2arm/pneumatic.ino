/*
 * =====================================================================
 * FILE    : pneumatic.ino
 * PERAN   : Control pneumatic valve via digital output.
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#include "pneumatic.h"

// =====================================================================
//  STATE
// =====================================================================

namespace {

struct PneumaticConfig {
    char id;
    uint8_t pin;
};

constexpr PneumaticConfig pneumaticValves[PNEUMATIC_COUNT] = {
    {'r', PNEUMATIC_R_PIN},
    {'l', PNEUMATIC_L_PIN},
    // ponytail: 2 valves deadcode dulu
};

bool pneumaticActive[PNEUMATIC_COUNT] = {};

int findPneumaticIndex(char id) {
    for (size_t i = 0; i < PNEUMATIC_COUNT; i++) {
        if (pneumaticValves[i].id == id) return (int)i;
    }
    return -1;
}

} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupPneumatic() {
    for (size_t i = 0; i < PNEUMATIC_COUNT; i++) {
        pinMode(pneumaticValves[i].pin, OUTPUT);
        digitalWrite(pneumaticValves[i].pin, LOW);  // default OFF
    }
}

// =====================================================================
//  CONTROL
// =====================================================================

void pneumaticOn(char id) {
    int idx = findPneumaticIndex(id);
    if (idx < 0) return;
    digitalWrite(pneumaticValves[idx].pin, HIGH);
    pneumaticActive[idx] = true;
}

void pneumaticOff(char id) {
    int idx = findPneumaticIndex(id);
    if (idx < 0) return;
    digitalWrite(pneumaticValves[idx].pin, LOW);
    pneumaticActive[idx] = false;
}

void pneumaticToggle(char id) {
    int idx = findPneumaticIndex(id);
    if (idx < 0) return;
    if (pneumaticActive[idx]) {
        pneumaticOff(id);
    } else {
        pneumaticOn(id);
    }
}

bool pneumaticState(char id) {
    int idx = findPneumaticIndex(id);
    if (idx < 0) return false;
    return pneumaticActive[idx];
}

void pneumaticAllOff() {
    for (size_t i = 0; i < PNEUMATIC_COUNT; i++) {
        pneumaticOff(pneumaticValves[i].id);
    }
}
