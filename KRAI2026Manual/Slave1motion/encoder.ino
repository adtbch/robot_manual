#include "encoder.h"
#include <ESP32Encoder.h>

// =====================================================================
//  STATE
// =====================================================================

namespace {

// Internal (ISR) — pin arrays
static const uint8_t intPinA[INT_ENCODER_COUNT] = {INT_ENC_FR_A, INT_ENC_FL_A, INT_ENC_BR_A, INT_ENC_BL_A};
static const uint8_t intPinB[INT_ENCODER_COUNT] = {INT_ENC_FR_B, INT_ENC_FL_B, INT_ENC_BR_B, INT_ENC_BL_B};

volatile int64_t intEncCount[INT_ENCODER_COUNT] = {0};
portMUX_TYPE intEncMux = portMUX_INITIALIZER_UNLOCKED;

float motorVelocityRpm[INT_ENCODER_COUNT] = {0};

// Precomputed: delta * 60000 / PPR / dt = delta * (60000 / PPR) / dt
constexpr float PPR_RPM_FACTOR = 60000.0f / ENCODER_PPR;

// External (PCNT)
ESP32Encoder extEncoders[EXT_ENCODER_COUNT];

inline int64_t readIntEncCount(int idx) {
    int64_t val;
    portENTER_CRITICAL(&intEncMux);
    val = intEncCount[idx];
    portEXIT_CRITICAL(&intEncMux);
    return val;
}

} // anonymous namespace

// =====================================================================
//  INTERNAL (ISR)
// =====================================================================

void IRAM_ATTR intEncoderISR(void* arg) {
    int idx = (int)(size_t)arg;

    uint32_t g0 = REG_READ(GPIO_IN_REG);
    uint32_t g1 = REG_READ(GPIO_IN1_REG);
    int pb = intPinB[idx];
    bool bHigh = (pb < 32) ? (g0 >> pb) & 1 : (g1 >> (pb - 32)) & 1;

    portENTER_CRITICAL_ISR(&intEncMux);
    if (bHigh)  intEncCount[idx]--;
    else        intEncCount[idx]++;
    portEXIT_CRITICAL_ISR(&intEncMux);
}

static void setupIntEncoders() {
    for (int i = 0; i < INT_ENCODER_COUNT; i++) {
        pinMode(intPinA[i], INPUT_PULLUP);
        pinMode(intPinB[i], INPUT_PULLUP);
        attachInterruptArg(digitalPinToInterrupt(intPinA[i]), intEncoderISR, (void*)(size_t)i, RISING);
    }
}

// =====================================================================
//  EXTERNAL (PCNT)
// =====================================================================

static void setupExtEncoders() {
    static const uint8_t extPinA[EXT_ENCODER_COUNT] = {EXT_ENC_FR_A, EXT_ENC_FL_A, EXT_ENC_BR_A, EXT_ENC_BL_A};
    static const uint8_t extPinB[EXT_ENCODER_COUNT] = {EXT_ENC_FR_B, EXT_ENC_FL_B, EXT_ENC_BR_B, EXT_ENC_BL_B};

    for (int i = 0; i < EXT_ENCODER_COUNT; i++) {
        extEncoders[i].attachHalfQuad(extPinA[i], extPinB[i]);
        extEncoders[i].clearCount();
    }
}

// =====================================================================
//  PUBLIC API
// =====================================================================

void setupEncoders() {
    setupIntEncoders();
    setupExtEncoders();
}

void convertEncoderToRPM() {
    static int64_t prevCount[INT_ENCODER_COUNT] = {0};
    static uint32_t lastMs = 0;
    static bool inited = false;

    uint32_t now = millis();
    uint32_t dt = now - lastMs;
    if (dt == 0) dt = 1;
    lastMs = now;

    if (!inited) {
        for (int i = 0; i < INT_ENCODER_COUNT; i++)
            prevCount[i] = readIntEncCount(i);
        inited = true;
        return;
    }

    for (int i = 0; i < INT_ENCODER_COUNT; i++) {
        int64_t curr = readIntEncCount(i);
        int64_t delta = curr - prevCount[i];
        prevCount[i] = curr;

        float rpm = (float)delta * PPR_RPM_FACTOR / (float)dt;

        if (delta == 0) {
            motorVelocityRpm[i] = 0.0f;
        } else {
            constexpr float ALPHA = 0.45f;
            motorVelocityRpm[i] = ALPHA * rpm + (1.0f - ALPHA) * motorVelocityRpm[i];
        }
    }
}

float getEncoderVelocityRpm(int idx) {
    if (idx < 0 || idx >= INT_ENCODER_COUNT) return 0.0f;
    return motorVelocityRpm[idx];
}

float getEncoderVelocityRadS(int idx) {
    return getEncoderVelocityRpm(idx) * RPM_TO_RAD_PER_SEC;
}

int64_t getExtEncoderCount(int idx) {
    if (idx < 0 || idx >= EXT_ENCODER_COUNT) return 0;
    return extEncoders[idx].getCount();
}

void resetExtEncoderCount(int idx) {
    if (idx < 0 || idx >= EXT_ENCODER_COUNT) return;
    extEncoders[idx].clearCount();
}

float getEncoderYawRateRads() {
    float vFR = getEncoderVelocityRadS(0) * WHEEL_RADIUS_M;
    float vFL = getEncoderVelocityRadS(1) * WHEEL_RADIUS_M;
    float vBR = getEncoderVelocityRadS(2) * WHEEL_RADIUS_M;
    float vBL = getEncoderVelocityRadS(3) * WHEEL_RADIUS_M;

    float halfWheelbase = ROBOT_LX + ROBOT_LY;
    if (fabsf(halfWheelbase) < 0.001f) return 0.0f;

    return (-vFL + vFR + vBL - vBR) / (2.0f * halfWheelbase);
}

float getEncoderConfidence() {
    int64_t maxTick = 0;
    for (int i = 0; i < INT_ENCODER_COUNT; i++) {
        float rpm = fabsf(motorVelocityRpm[i]);
        int64_t t = (int64_t)(rpm / 60.0f * ENCODER_PPR * RPM_INTERVAL_MS / 1000.0f);
        if (t > maxTick) maxTick = t;
    }

    if (maxTick < 2)   return 0.0f;
    if (maxTick < 5)   return 0.2f;
    if (maxTick < 15)  return 0.5f;
    if (maxTick < 40)  return 0.7f;
    return 0.9f;
}
