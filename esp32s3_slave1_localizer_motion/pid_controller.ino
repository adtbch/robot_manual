// ============================================================
// PID Controller - Velocity (RPM) feedback based
// ============================================================

#include "robot_config.h"

// PID state per motor (dinamis sesuai jumlah motor)
std::vector<PIDState> pidStates;

// ============================================================
// NVS PID Load/Save
// ============================================================

void pidLoadFromNVS(int motorIdx, float &kp, float &ki, float &kd) {
    if (motorIdx < 0 || (size_t)motorIdx >= motors.size()) {
        kp = 0.1f; ki = 0.0f; kd = 0.0f;
        return;
    }

    Preferences prefs;
    prefs.begin(PID_NVS_NAMESPACE, true);  // read-only

    char keyKp[16], keyKi[16], keyKd[16];
    snprintf(keyKp, sizeof(keyKp), "kp_%d", motorIdx);
    snprintf(keyKi, sizeof(keyKi), "ki_%d", motorIdx);
    snprintf(keyKd, sizeof(keyKd), "kd_%d", motorIdx);

    kp = prefs.getFloat(keyKp, 0.1f);
    ki = prefs.getFloat(keyKi, 0.0f);
    kd = prefs.getFloat(keyKd, 0.0f);

    prefs.end();
}

void pidSaveToNVS(int motorIdx, float kp, float ki, float kd) {
    if (motorIdx < 0 || (size_t)motorIdx >= motors.size()) {
        return;
    }

    Preferences prefs;
    prefs.begin(PID_NVS_NAMESPACE, false);  // read-write

    char keyKp[16], keyKi[16], keyKd[16];
    snprintf(keyKp, sizeof(keyKp), "kp_%d", motorIdx);
    snprintf(keyKi, sizeof(keyKi), "ki_%d", motorIdx);
    snprintf(keyKd, sizeof(keyKd), "kd_%d", motorIdx);

    prefs.putFloat(keyKp, kp);
    prefs.putFloat(keyKi, ki);
    prefs.putFloat(keyKd, kd);

    prefs.end();
}

// ============================================================
// PID Initialization - dipanggil setelah SetupMotors()
// ============================================================

void pidControllerInit() {
    pidStates.clear();
    pidStates.reserve(motors.size());

    for (size_t i = 0; i < motors.size(); i++) {
        PIDState state;
        pidLoadFromNVS(i, state.kp, state.ki, state.kd);
        state.reset();
        pidStates.push_back(state);
    }
}

// ============================================================
// Get motor velocity in RPM (from encoder)
// ============================================================

float getMotorVelocityRpm(int motorIdx) {
    if (motorIdx < 0 || (size_t)motorIdx >= encoders.size()) {
        return 0.0f;
    }

    return getEncoderVelocityRpm(motorIdx);
}

// ============================================================
// Get motor velocity in rad/s (from encoder)
// ============================================================

float getMotorVelocityRadS(int motorIdx) {
    float rpm = getMotorVelocityRpm(motorIdx);
    return rpm * kRpmToRadPerSec;
}

// ============================================================
// Update PID gains for a specific motor
// ============================================================

void pidSetGains(int motorIdx, float kp, float ki, float kd) {
    if (motorIdx < 0 || (size_t)motorIdx >= pidStates.size()) {
        return;
    }

    pidStates[motorIdx].kp = constrain(kp, kpMin, kpMax);
    pidStates[motorIdx].ki = constrain(ki, kiMin, kiMax);
    pidStates[motorIdx].kd = constrain(kd, kdMin, kdMax);
}

// ============================================================
// Reset PID state (clear integral and derivative) for one motor
// ============================================================

void pidResetOne(int motorIdx) {
    if (motorIdx < 0 || (size_t)motorIdx >= pidStates.size()) {
        return;
    }
    pidStates[motorIdx].reset();
}

void pidReloadFromNVS() {
    for (size_t i = 0; i < pidStates.size(); i++) {
        pidLoadFromNVS(i, pidStates[i].kp, pidStates[i].ki, pidStates[i].kd);
        pidResetOne(i);
    }
}

// ============================================================
// Stop all motors (set PWM = 0)
// ============================================================

void motorStopAll() {
    for (size_t i = 0; i < motors.size(); i++) {
        pwmMotor(i, 0);
    }
}

// ============================================================
// pidCompute — satu-satunya PID compute function
// Dipakai oleh autoTuner DAN rpmMotor (unifikasi)
// Anti-windup proporsional terhadap Ki, bukan hardcoded
// ============================================================

int pidCompute(int motorIdx, float targetRPM, float dt) {
    if (motorIdx < 0 || (size_t)motorIdx >= pidStates.size()) {
        return 0;
    }

    PIDState &pid = pidStates[motorIdx];

    // DETEKSI PERUBAHAN TARGET BESAR: reset integral jika target berubah > 20%
    // Ini mencegah "overshoot" saat tiba-tiba turun dari 100 RPM ke 30 RPM
    if (pid.lastTargetRPM != 0.0f && fabs(targetRPM - pid.lastTargetRPM) > fabs(pid.lastTargetRPM) * 0.20f) {
        pid.integral = 0.0f;  // Reset integral agar tidak ada windup dari target lama
    }
    pid.lastTargetRPM = targetRPM;

    float currentRPM = getEncoderVelocityRpm(motorIdx);
    float error = targetRPM - currentRPM;

    // Proportional
    float pOut = pid.kp * error;

    // Integral dengan anti-windup proporsional Ki
    float integralLimit = (pid.ki > 0.0001f) ? (float)maxPwm / pid.ki : 2000.0f;
    pid.integral += error * dt;
    pid.integral = constrain(pid.integral, -integralLimit, integralLimit);
    float iOut = pid.ki * pid.integral;

    // Derivative on MEASUREMENT (bukan error) → mencegah derivative kick
    float dOut = 0.0f;
    if (dt > 0.0f && pid.lastTime > 0.0f) {
        // Negative derivative karena jika currentRPM naik, kita kurangi output
        dOut = -pid.kd * (currentRPM - pid.lastError) / dt;
    }

    float output = pOut + iOut + dOut;
    int pwmOutput = (int)constrain(output, (float)minPwm, (float)maxPwm);

    // Simpan currentRPM sebagai "lastError" untuk derivative next cycle
    pid.lastError = currentRPM;
    pid.lastTime  = millis();

    return pwmOutput;
}

// ============================================================
// rpmMotorControl — kontrol 4 motor sekaligus via pidCompute
// ============================================================

void rpmMotorControl(int targetRPM0, int targetRPM1, int targetRPM2, int targetRPM3) {
    static uint32_t lastTickMs = 0;
    uint32_t nowMs = millis();
    float dt = (lastTickMs > 0) ? (nowMs - lastTickMs) / 1000.0f : 0.04f;
    lastTickMs = nowMs;
    dt = constrain(dt, 0.01f, 0.1f);

    int targets[4] = {targetRPM0, targetRPM1, targetRPM2, targetRPM3};
    for (size_t i = 0; i < motors.size() && i < 4; i++) {
        int pwmOut = pidCompute((int)i, (float)targets[i], dt);
        pwmMotor((int)i, pwmOut);
    }
}

void rpmMotorControlTargets(const std::vector<float> &targetRpm) {
    static uint32_t lastTickMs = 0;
    uint32_t nowMs = millis();
    float dt = (lastTickMs > 0) ? (nowMs - lastTickMs) / 1000.0f : 0.04f;
    lastTickMs = nowMs;
    dt = constrain(dt, 0.01f, 0.1f);

    for (size_t i = 0; i < motors.size(); i++) {
        float target = (i < targetRpm.size()) ? targetRpm[i] : 0.0f;
        int pwmOut = pidCompute((int)i, target, dt);
        pwmMotor((int)i, pwmOut);
    }
}
