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

    // Reset our dynamic vector-based pidData as well!
    if ((size_t)motorIdx < pidData.size()) {
        pidData[motorIdx].integral = 0.0;
        pidData[motorIdx].previousError = 0.0;
        pidData[motorIdx].error = 0.0;
        pidData[motorIdx].lastTime = 0;
    }
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
