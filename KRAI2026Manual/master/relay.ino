/*
 * =====================================================================
 * FILE    : relay.ino
 * PERAN   : Control relay via digital output.
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "relay.h"

// =====================================================================
//  STATE
// =====================================================================

namespace {

bool relayActive = false;

} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupRelay() {
    pinMode(RELAY_1_PIN, OUTPUT);
    digitalWrite(RELAY_1_PIN, LOW);  // default OFF
}

// =====================================================================
//  CONTROL
// =====================================================================

void relayOn() {
    digitalWrite(RELAY_1_PIN, HIGH);
    relayActive = true;
}

void relayOff() {
    digitalWrite(RELAY_1_PIN, LOW);
    relayActive = false;
}

void relayToggle() {
    if (relayActive) {
        relayOff();
    } else {
        relayOn();
    }
}

bool relayState() {
    return relayActive;
}
