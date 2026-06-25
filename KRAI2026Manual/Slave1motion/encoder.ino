/*
 * =====================================================================
 * FILE    : encoder.ino
 * PERAN   : Baca posisi encoder via ESP32Encoder library.
 *           4 encoder untuk mecanum drive + RPM calculation.
 *
 * LIBRARY : ESP32Encoder (lucky68t)
 *           attachHalfQuad() — 4x resolution
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#include "encoder.h"
#include <ESP32Encoder.h>

// =====================================================================
//  STATE
// =====================================================================

namespace {

ESP32Encoder encoders[ENCODER_COUNT];

// RPM cache (updated in convertEncoderToRPM)
float motorVelocityRpm[ENCODER_COUNT] = {};
bool rpmInitialized = false;

} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupEncoders() {
    encoders[0].attachHalfQuad(ENCODER_FR_A, ENCODER_FR_B);
    encoders[0].clearCount();

    encoders[1].attachHalfQuad(ENCODER_FL_A, ENCODER_FL_B);
    encoders[1].clearCount();

    encoders[2].attachHalfQuad(ENCODER_BR_A, ENCODER_BR_B);
    encoders[2].clearCount();

    encoders[3].attachHalfQuad(ENCODER_BL_A, ENCODER_BL_B);
    encoders[3].clearCount();
}

// =====================================================================
//  RPM CALCULATION — panggil setiap 40ms
// =====================================================================

void convertEncoderToRPM() {
    static float rpmFiltered[ENCODER_COUNT] = {};
    static int64_t prevCount[ENCODER_COUNT] = {};
    static uint32_t lastTimeMs = 0;

    if (!rpmInitialized) {
        for (size_t i = 0; i < ENCODER_COUNT; i++) {
            prevCount[i] = encoders[i].getCount();
        }
        lastTimeMs = millis();
        rpmInitialized = true;
        return;
    }

    uint32_t nowMs = millis();
    uint32_t elapsedMs = nowMs - lastTimeMs;
    if (elapsedMs == 0) elapsedMs = 1;
    lastTimeMs = nowMs;

    for (size_t i = 0; i < ENCODER_COUNT; i++) {
        int64_t currentCount = encoders[i].getCount();
        int64_t delta = currentCount - prevCount[i];
        prevCount[i] = currentCount;

        // RPM calculation: (delta * 60000) / (elapsed_ms * PPR)
        float rpmRaw = ((float)delta * 60000.0f) / ((float)elapsedMs * (float)ENCODER_PPR);

        // IIR filter (alpha = 0.45)
        if (delta == 0) {
            rpmFiltered[i] = 0.0f;
        } else {
            constexpr float ALPHA = 0.45f;
            rpmFiltered[i] = ALPHA * rpmRaw + (1.0f - ALPHA) * rpmFiltered[i];
        }

        motorVelocityRpm[i] = rpmFiltered[i];
    }
}

// =====================================================================
//  VELOCITY GETTERS
// =====================================================================

float getEncoderVelocityRpm(int motorIdx) {
    if (motorIdx < 0 || (size_t)motorIdx >= ENCODER_COUNT) {
        return 0.0f;
    }
    return motorVelocityRpm[motorIdx];
}

float getEncoderVelocityRadS(int motorIdx) {
    return getEncoderVelocityRpm(motorIdx) * RPM_TO_RAD_PER_SEC;
}

// =====================================================================
//  YAW RATE dari 4 encoder (rad/s)
// =====================================================================

float getEncoderYawRateRads() {
    float vFR = getEncoderVelocityRadS(0) * WHEEL_RADIUS_M;
    float vFL = getEncoderVelocityRadS(1) * WHEEL_RADIUS_M;
    float vBR = getEncoderVelocityRadS(2) * WHEEL_RADIUS_M;
    float vBL = getEncoderVelocityRadS(3) * WHEEL_RADIUS_M;

    float denom = 2.0f * (ROBOT_LX + ROBOT_LY);
    if (fabs(denom) < 0.001f) return 0.0f;

    return (-vFL + vFR + vBL - vBR) / denom;
}

// =====================================================================
//  CONFIDENCE berdasarkan delta tick
// =====================================================================

float getEncoderConfidence() {
    // Ambil max absolute delta dari semua encoder
    int64_t maxTick = 0;
    for (size_t i = 0; i < ENCODER_COUNT; i++) {
        // Kita tidak simpan last_delta, jadi pakai getCount() langsung
        // Confidence dihitung dari RPM magnitude
        float rpm = fabsf(motorVelocityRpm[i]);
        int64_t approxTick = (int64_t)(rpm / 60.0f * ENCODER_PPR * RPM_INTERVAL_MS / 1000.0f);
        if (approxTick > maxTick) maxTick = approxTick;
    }

    if (maxTick < 2)   return 0.0f;
    if (maxTick < 5)   return 0.2f;
    if (maxTick < 15)  return 0.5f;
    if (maxTick < 40)  return 0.7f;
    return 0.9f;
}
