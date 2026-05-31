// ============================================================
// PID Controller - Velocity (RPM) feedback based
// ============================================================

#include "robot_config.h"

struct PIDState {
    float kp, ki, kd;
    float integral;
    float lastError;
    float lastTime;
    
    void reset() {
        integral = 0.0f;
        lastError = 0.0f;
        lastTime = 0.0f;
    }
};

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

// ============================================================
// Main PID control loop - compute output PWM
// Returns PWM value [-1023, 1023] based on velocity error
// ============================================================

int pidCompute(int motorIdx, float targetRPM, float dt) {
    if (motorIdx < 0 || (size_t)motorIdx >= pidStates.size()) {
        return 0;
    }
    
    PIDState &pid = pidStates[motorIdx];
    
    // Baca current velocity
    float currentRPM = getMotorVelocityRpm(motorIdx);
    float error = targetRPM - currentRPM;
    
    // Proportional term
    float pOut = pid.kp * error;
    
    // Integral term (dengan anti-windup)
    pid.integral += error * dt;
    pid.integral = constrain(pid.integral, -100.0f, 100.0f);  // Clamp integral
    float iOut = pid.ki * pid.integral;
    
    // Derivative term
    float dOut = 0.0f;
    if (dt > 0.0f) {
        float dError = (error - pid.lastError) / dt;
        dOut = pid.kd * dError;
    }
    
    // Total output
    float output = pOut + iOut + dOut;
    int pwmOutput = (int)constrain(output, minPwm, maxPwm);
    
    // Update state
    pid.lastError = error;
    pid.lastTime = millis();
    
    return pwmOutput;
}

// ============================================================
// RPM Motor Control - main control interface
// Runs PID loop untuk semua motor dengan target RPM
// ============================================================

void rpmMotorControl(int targetRPM0, int targetRPM1, int targetRPM2, int targetRPM3) {
    static uint32_t lastTickMs = 0;
    uint32_t nowMs = millis();
    float dt = (lastTickMs > 0) ? (nowMs - lastTickMs) / 1000.0f : 0.04f;
    lastTickMs = nowMs;
    
    // Clamp dt ke range wajar (10ms - 100ms)
    dt = constrain(dt, 0.01f, 0.1f);
    
    int targets[4] = {targetRPM0, targetRPM1, targetRPM2, targetRPM3};
    
    for (size_t i = 0; i < motors.size() && i < 4; i++) {
        int pwmOut = pidCompute(i, targets[i], dt);
        pwmMotor(i, pwmOut);
    }
}

void rpmMotorControlTargets(const std::vector<float> &targetRpm) {
    static uint32_t lastTickMs = 0;
    uint32_t nowMs = millis();
    float dt = (lastTickMs > 0) ? (nowMs - lastTickMs) / 1000.0f : 0.04f;
    lastTickMs = nowMs;

    dt = constrain(dt, 0.01f, 0.1f);

    for (size_t i = 0; i < motors.size(); i++) {
        float target = 0.0f;
        if (i < targetRpm.size()) {
            target = targetRpm[i];
        }
        int pwmOut = pidCompute((int)i, target, dt);
        pwmMotor((int)i, pwmOut);
    }
}

// ============================================================
// Reload PID gains dari NVS (setelah auto-tuning selesai)
// ============================================================

void pidReloadFromNVS() {
    for (size_t i = 0; i < pidStates.size(); i++) {
        pidLoadFromNVS(i, pidStates[i].kp, pidStates[i].ki, pidStates[i].kd);
        pidStates[i].reset();
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
