/*
 * =====================================================================
 * FILE    : encoder.ino
 * PERAN   : Baca posisi encoder via library ESP32Encoder.
 *           2 encoder: Encoder1, Encoder2.
 *
 * LIBRARY : ESP32Encoder (lucky68t)
 *           attachHalfQuad() — 4x resolution, half-quad wiring
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#include "encoder.h"
#include <ESP32Encoder.h>

// =====================================================================
//  STATE
// =====================================================================

namespace {

ESP32Encoder encoders[ENCODER_COUNT];

// x→enc0, y→enc1, k→no encoder; numeric 0/1 still valid for serial `enc`
int encoderIdxFromId(char id) {
    if (id == 'x' || id == 0) return 0;
    if (id == 'y' || id == 1) return 1;
    if (id == 'k') return -1;
    if (id >= 0 && (size_t)id < ENCODER_COUNT) return (int)id;
    return -1;
}

} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupEncoders() {
    encoders[0].attachHalfQuad(ENCODER1_PIN_A, ENCODER1_PIN_B);
    encoders[0].clearCount();

    encoders[1].attachHalfQuad(ENCODER2_PIN_A, ENCODER2_PIN_B);
    encoders[1].clearCount();
}

// =====================================================================
//  READ
// =====================================================================

long getEncoderCount(char encoderIndex) {
    const int idx = encoderIdxFromId(encoderIndex);
    if (idx < 0) return 0;
    return (long)encoders[idx].getCount();
}

// =====================================================================
//  RESET
// =====================================================================

void resetEncoderCount(char encoderIndex) {
    const int idx = encoderIdxFromId(encoderIndex);
    if (idx < 0) return;
    encoders[idx].clearCount();
}
