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

// Odometry state (X-config omni wheels)
int64_t prevExtCount[EXT_ENCODER_COUNT] = {0};

inline int64_t readIntEncCount(int idx) {
    int64_t val;
    portENTER_CRITICAL(&intEncMux);
    val = intEncCount[idx];
    portEXIT_CRITICAL(&intEncMux);
    return val;
}

} // anonymous namespace

// Odometry state — extern di encoder.h
float odomX = 0.0f;        // meter
float odomY = 0.0f;        // meter
float odomTheta = 0.0f;    // derajat

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

// =====================================================================
//  EXTERNAL (PCNT) — ODOMETRY
// =====================================================================

int64_t getExtEncoderCount(int idx) {
    if (idx < 0 || idx >= EXT_ENCODER_COUNT) return 0;
    return extEncoders[idx].getCount();
}

void resetExtEncoderCount(int idx) {
    if (idx < 0 || idx >= EXT_ENCODER_COUNT) return;
    extEncoders[idx].clearCount();
}

// X-config omni wheels:
//   FR axis 45°, FL axis 135°, BR axis -45°, BL axis -135°
//   vx = (vFR - vFL + vBR - vBL) * cos45
//   vy = (vFR + vFL - vBR - vBL) * sin45
//   ω  = (-vFR - vFL + vBR + vBL) / (4 * L)
//   cos45 = sin45 ≈ 0.7071

void updateOdometry() {
    static uint32_t lastMs = 0;
    uint32_t nowMs = millis();
    float dt = (lastMs == 0) ? 0.0f : (nowMs - lastMs) * 0.001f;
    lastMs = nowMs;
    if (dt <= 0.0f || dt > 0.5f) return;  // skip first call & large gaps

    constexpr float COS45 = 0.70710678f;
    constexpr float WHEEL_CIRC_M = 2.0f * PI * WHEEL_RADIUS_M;
    constexpr float TICKS_TO_M = WHEEL_CIRC_M / (float)EXT_ENCODER_PPR;

    float v[4];  // linear velocity per wheel (m/s)
    for (int i = 0; i < EXT_ENCODER_COUNT; i++) {
        int64_t curr = getExtEncoderCount(i);
        int64_t delta = curr - prevExtCount[i];
        prevExtCount[i] = curr;
        v[i] = (float)delta * TICKS_TO_M / dt;
    }

    // Forward kinematics X-config
    float vx = (v[0] - v[1] + v[2] - v[3]) * COS45;
    float vy = (v[0] + v[1] - v[2] - v[3]) * COS45;

    // Rotasi vx/vy berdasarkan heading saat ini
    float thetaRad = odomTheta * (PI / 180.0f);
    float cosT = cosf(thetaRad);
    float sinT = sinf(thetaRad);
    float vxWorld = vx * cosT - vy * sinT;
    float vyWorld = vx * sinT + vy * cosT;

    // Integrasi posisi
    odomX += vxWorld * dt;
    odomY += vyWorld * dt;
}

void resetOdometry() {
    odomX = 0.0f;
    odomY = 0.0f;
    odomTheta = 0.0f;
    for (int i = 0; i < EXT_ENCODER_COUNT; i++) {
        prevExtCount[i] = getExtEncoderCount(i);
    }
}

void setOdomTheta(float theta) {
    odomTheta = theta;
}
