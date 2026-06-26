/*
 * =====================================================================
 * FILE    : pid.ino
 * PERAN   : PID controller + NVS + RPM motor control.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#include "pid.h"
#include <Preferences.h>

// =====================================================================
//  STATE
// =====================================================================

namespace {

float rpmTarget[MOTOR_COUNT] = {};

constexpr const char* YAW_PID_NVS_NS = "yaw_pid";
constexpr float YAW_PID_DEFAULT_KP = 2.5f;
constexpr float YAW_PID_DEFAULT_KI = 0.01f;
constexpr float YAW_PID_DEFAULT_KD = 0.1f;

} // anonymous namespace

PIDState pidStates[MOTOR_COUNT];
PIDState pidKinematicYaw;

// =====================================================================
//  NVS LOAD/SAVE — MOTOR PID
// =====================================================================

void pidLoadFromNVS(int motorIdx, float &kp, float &ki, float &kf, float &deadband) {
    if (motorIdx < 0 || (size_t)motorIdx >= MOTOR_COUNT) {
        kp = 0.1f; ki = 0.0f; kf = 0.0f; deadband = 0.0f;
        return;
    }
    Preferences prefs;
    prefs.begin(PID_NVS_NAMESPACE, true);
    char key[16];
    snprintf(key, sizeof(key), "kp_%d", motorIdx); kp = prefs.getFloat(key, 0.1f);
    snprintf(key, sizeof(key), "ki_%d", motorIdx); ki = prefs.getFloat(key, 0.0f);
    snprintf(key, sizeof(key), "kf_%d", motorIdx); kf = prefs.getFloat(key, 0.0f);
    snprintf(key, sizeof(key), "db_%d", motorIdx); deadband = prefs.getFloat(key, 0.0f);
    prefs.end();
}

void pidSaveToNVS(int motorIdx, float kp, float ki, float kf, float deadband) {
    if (motorIdx < 0 || (size_t)motorIdx >= MOTOR_COUNT) return;
    Preferences prefs;
    prefs.begin(PID_NVS_NAMESPACE, false);
    char key[16];
    snprintf(key, sizeof(key), "kp_%d", motorIdx); prefs.putFloat(key, kp);
    snprintf(key, sizeof(key), "ki_%d", motorIdx); prefs.putFloat(key, ki);
    snprintf(key, sizeof(key), "kf_%d", motorIdx); prefs.putFloat(key, kf);
    snprintf(key, sizeof(key), "db_%d", motorIdx); prefs.putFloat(key, deadband);
    prefs.end();
}

// =====================================================================
//  NVS LOAD/SAVE — YAW PID
// =====================================================================

void initYawPid() {
    Preferences prefs;
    prefs.begin(YAW_PID_NVS_NS, true);
    pidKinematicYaw.kp = prefs.getFloat("kp", YAW_PID_DEFAULT_KP);
    pidKinematicYaw.ki = prefs.getFloat("ki", YAW_PID_DEFAULT_KI);
    pidKinematicYaw.kd = prefs.getFloat("kd", YAW_PID_DEFAULT_KD);
    pidKinematicYaw.reset();
    pidKinematicYaw.lastTarget = 0.0f;
    prefs.end();
    Serial.printf("Yaw PID loaded: Kp=%.3f Ki=%.3f Kd=%.3f\n",
        pidKinematicYaw.kp, pidKinematicYaw.ki, pidKinematicYaw.kd);
}

void saveYawPid() {
    Preferences prefs;
    prefs.begin(YAW_PID_NVS_NS, false);
    prefs.putFloat("kp", pidKinematicYaw.kp);
    prefs.putFloat("ki", pidKinematicYaw.ki);
    prefs.putFloat("kd", pidKinematicYaw.kd);
    prefs.end();
    Serial.printf("Yaw PID saved: Kp=%.3f Ki=%.3f Kd=%.3f\n",
        pidKinematicYaw.kp, pidKinematicYaw.ki, pidKinematicYaw.kd);
}

void showYawPid() {
    Serial.printf("Yaw PID: Kp=%.3f Ki=%.3f Kd=%.3f\n",
        pidKinematicYaw.kp, pidKinematicYaw.ki, pidKinematicYaw.kd);
}

// =====================================================================
//  PID INIT
// =====================================================================

void pidControllerInit() {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        pidLoadFromNVS(i, pidStates[i].kp, pidStates[i].ki, pidStates[i].kf, pidStates[i].deadband);
        pidStates[i].kd = 0.0f; // Motor PID does not use Kd
        pidStates[i].reset();
    }
}

// =====================================================================
//  PID GAINS
// =====================================================================

void pidSetGains(int motorIdx, float kp, float ki, float kf, float deadband) {
    if (motorIdx < 0 || (size_t)motorIdx >= MOTOR_COUNT) return;
    pidStates[motorIdx].kp = constrain(kp, KP_MIN, KP_MAX);
    pidStates[motorIdx].ki = constrain(ki, KI_MIN, KI_MAX);
    pidStates[motorIdx].kf = constrain(kf, KF_MIN, KF_MAX);
    pidStates[motorIdx].deadband = constrain(deadband, DEADBAND_MIN, DEADBAND_MAX);
    pidStates[motorIdx].kd = 0.0f;
}

void pidResetOne(int motorIdx) {
    if (motorIdx < 0 || (size_t)motorIdx >= MOTOR_COUNT) return;
    pidStates[motorIdx].reset();
}

void pidReloadFromNVS() {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        pidLoadFromNVS(i, pidStates[i].kp, pidStates[i].ki, pidStates[i].kf, pidStates[i].deadband);
        pidStates[i].kd = 0.0f;
        pidResetOne(i);
    }
}

// =====================================================================
//  PID COMPUTE — Generic
// =====================================================================

int pidCompute(PIDState &pid, float target, float current, float dt) {
    // Target change detection
    if (pid.lastTarget != 0.0f && fabs(target - pid.lastTarget) > fabs(pid.lastTarget) * 0.20f) {
        pid.integral = 0.0f;
    }
    pid.lastTarget = target;

    float error = target - current;

    // Proportional
    float pOut = pid.kp * error;

    // Integral with anti-windup
    float integralLimit = (pid.ki > 0.0001f) ? (float)PWM_MAX / pid.ki : 2000.0f;
    pid.integral += error * dt;
    pid.integral = constrain(pid.integral, -integralLimit, integralLimit);
    float iOut = pid.ki * pid.integral;

    // Derivative on measurement
    float dOut = 0.0f;
    if (pid.kd > 0.0f && dt > 0.0f && pid.lastTime > 0.0f) {
        dOut = -pid.kd * (current - pid.lastError) / dt;
    }

    // Feed-Forward with Deadband Compensation (Coulomb Friction)
    float ffOut = 0.0f;
    if (target > 0.5f) {
        ffOut = (pid.kf * target) + pid.deadband;
    } else if (target < -0.5f) {
        ffOut = (pid.kf * target) - pid.deadband;
    }

    float output = pOut + iOut + dOut + ffOut;
    int pwmOutput = (int)constrain(output, (float)PWM_MIN, (float)PWM_MAX);

    pid.lastError = current;
    pid.lastTime = millis();

    return pwmOutput;
}

int pidCompute(int motorIdx, float targetRPM, float dt) {
    if (motorIdx < 0 || (size_t)motorIdx >= MOTOR_COUNT) return 0;
    float currentRPM = getEncoderVelocityRpm(motorIdx);
    return pidCompute(pidStates[motorIdx], targetRPM, currentRPM, dt);
}

// =====================================================================
//  PID COMPUTE YAW — with shortest-path wrapping
// =====================================================================

int pidComputeYaw(PIDState &pid, float target, float current, float dt) {
    float error = target - current;
    if (error > -2.0f && error < 2.0f) error = 0.0f;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    float pOut = pid.kp * error;

    float integralLimit = (pid.ki > 0.0001f) ? 200.0f / pid.ki : 2000.0f;
    pid.integral += error * dt;
    pid.integral = constrain(pid.integral, -integralLimit, integralLimit);
    float iOut = pid.ki * pid.integral;

    float dOut = 0.0f;
    if (dt > 0.0f && pid.lastTime > 0.0f) {
        float diff = current - pid.lastError;
        while (diff > 180.0f) diff -= 360.0f;
        while (diff < -180.0f) diff += 360.0f;
        dOut = -pid.kd * diff / dt;
    }

    pid.lastError = current;
    pid.lastTime = millis();

    float output = pOut + iOut + dOut;
    return (int)constrain(output, -500.0f, 500.0f);
}

// =====================================================================
//  RPM MOTOR CONTROL
// =====================================================================

namespace {

float activeTarget[MOTOR_COUNT] = {};
int lastRequested[MOTOR_COUNT] = {};

int rampedPidPwm(int motorIdx, int requestedRpm, float dt) {
    if (motorIdx < 0 || motorIdx >= (int)MOTOR_COUNT) return 0;

    if (requestedRpm == 0) {
        activeTarget[motorIdx] = 0.0f;
        lastRequested[motorIdx] = 0;
        pidResetOne(motorIdx);
        return 0;
    }

    int prevReq = lastRequested[motorIdx];
    bool signChanged = (prevReq != 0) && ((prevReq > 0) != (requestedRpm > 0));

    if (signChanged) {
        pidResetOne(motorIdx);
        activeTarget[motorIdx] = 0.0f;
    }
    lastRequested[motorIdx] = requestedRpm;

    float delta = (float)requestedRpm - activeTarget[motorIdx];
    if (fabsf(delta) < 2.0f) {
        activeTarget[motorIdx] = (float)requestedRpm;
    } else {
        float step = fabsf(delta) * 0.5f;
        step = fmaxf(step, 1.0f);
        step = fminf(step, fabsf(delta));
        activeTarget[motorIdx] += (delta > 0.0f) ? step : -step;
    }

    return pidCompute(motorIdx, activeTarget[motorIdx], dt);
}

} // anonymous namespace

void rpmMotor(int rpm1, int rpm2, int rpm3, int rpm4) {
    static uint32_t lastTickMs = 0;
    uint32_t nowMs = millis();

    if (lastTickMs > 0 && (nowMs - lastTickMs) < 40) return;

    float dt = (lastTickMs > 0) ? (nowMs - lastTickMs) / 1000.0f : 0.04f;
    lastTickMs = nowMs;
    dt = constrain(dt, 0.01f, 0.1f);

    rpm1 = constrain(rpm1, (int)RPM_MIN, (int)RPM_MAX);
    rpm2 = constrain(rpm2, (int)RPM_MIN, (int)RPM_MAX);
    rpm3 = constrain(rpm3, (int)RPM_MIN, (int)RPM_MAX);
    rpm4 = constrain(rpm4, (int)RPM_MIN, (int)RPM_MAX);

    int pFr = rampedPidPwm(0, rpm1, dt);
    int pFl = rampedPidPwm(1, rpm2, dt);
    int pBr = rampedPidPwm(2, rpm3, dt);
    int pBl = rampedPidPwm(3, rpm4, dt);

    pwmMotor(0, pFr);
    pwmMotor(1, pFl);
    pwmMotor(2, pBr);
    pwmMotor(3, pBl);
}
