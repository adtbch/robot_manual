/*
 * =====================================================================
 * FILE    : proximity.ino
 * PERAN   : Baca proximity sensor (digital).
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "proximity.h"

// =====================================================================
//  SETUP
// =====================================================================

void setupProximity() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // full range 0-3.3V
    pinMode(PROXIMITY_1_PIN, INPUT);
}

// =====================================================================
//  READ
// =====================================================================

bool readProximity() {
    return (analogRead(PROXIMITY_1_PIN) < PROX_THRESHOLD);  // < threshold = detect (0V)
}
