// ============================================================
// PID Controller - Velocity (RPM) feedback based
// ============================================================

#include "robot_config.h"

// PID state per motor (dinamis sesuai jumlah motor)
std::vector<PIDState> pidStates;

// PID State untuk koreksi Yaw pada Field-Centric
PIDState pidKinematicYaw;

// NVS namespace untuk yaw PID
static constexpr const char* YAW_PID_NVS_NS = "yaw_pid";

// Default values
static constexpr float YAW_PID_DEFAULT_KP = 2.5f;
static constexpr float YAW_PID_DEFAULT_KI = 0.01f;
static constexpr float YAW_PID_DEFAULT_KD = 0.1f;


// ============================================================
// NVS PID Load/Save
// ============================================================

// Load yaw PID dari NVS (jika belum ada, pakai default)
void initYawPid() {
    Preferences prefs;
    prefs.begin(YAW_PID_NVS_NS, true);  // read-only

    pidKinematicYaw.kp = prefs.getFloat("kp", YAW_PID_DEFAULT_KP);
    pidKinematicYaw.ki = prefs.getFloat("ki", YAW_PID_DEFAULT_KI);
    pidKinematicYaw.kd = prefs.getFloat("kd", YAW_PID_DEFAULT_KD);
    pidKinematicYaw.reset();
    pidKinematicYaw.lastTarget = 0.0f;

    prefs.end();

    Serial.printf("Yaw PID loaded: Kp=%.3f Ki=%.3f Kd=%.3f\n",
        pidKinematicYaw.kp, pidKinematicYaw.ki, pidKinematicYaw.kd);
}

// Simpan yaw PID ke NVS
void saveYawPid() {
    Preferences prefs;
    prefs.begin(YAW_PID_NVS_NS, false);  // read-write

    prefs.putFloat("kp", pidKinematicYaw.kp);
    prefs.putFloat("ki", pidKinematicYaw.ki);
    prefs.putFloat("kd", pidKinematicYaw.kd);

    prefs.end();

    Serial.printf("Yaw PID saved: Kp=%.3f Ki=%.3f Kd=%.3f\n",
        pidKinematicYaw.kp, pidKinematicYaw.ki, pidKinematicYaw.kd);
}

// Print current yaw PID values
void showYawPid() {
    Serial.printf("Yaw PID: Kp=%.3f Ki=%.3f Kd=%.3f\n",
        pidKinematicYaw.kp, pidKinematicYaw.ki, pidKinematicYaw.kd);
}

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

// Modular generic PID compute function
int pidCompute(PIDState &pid, float target, float current, float dt) {
    // DETEKSI PERUBAHAN TARGET BESAR: reset integral jika target berubah > 20%
    if (pid.lastTarget != 0.0f && fabs(target - pid.lastTarget) > fabs(pid.lastTarget) * 0.20f) {
        pid.integral = 0.0f;  // Reset integral agar tidak ada windup dari target lama
    }
    pid.lastTarget = target;

    float error = target - current;

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
        dOut = -pid.kd * (current - pid.lastError) / dt;
    }

    float output = pOut + iOut + dOut;
    int pwmOutput = (int)constrain(output, (float)minPwm, (float)maxPwm);

    // Simpan current sebagai "lastError" untuk derivative next cycle
    pid.lastError = current;
    pid.lastTime  = millis();

    return pwmOutput;
}

// Overload untuk kecocokan kode motor eksis
int pidCompute(int motorIdx, float targetRPM, float dt) {
    if (motorIdx < 0 || (size_t)motorIdx >= pidStates.size()) {
        return 0;
    }
    float currentRPM = getEncoderVelocityRpm(motorIdx);
    return pidCompute(pidStates[motorIdx], targetRPM, currentRPM, dt);
}

// ============================================================
// pidComputeYaw — PID khusus yaw dengan shortest-path wrapping
// ============================================================
int pidComputeYaw(PIDState &pid, float target, float current, float dt) {
  // 1. Error shortest path [-180, 180]
  float error = target - current;
  if (error < 2 && error > -2) error = 0;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;

  // 2. Proportional
  float pOut = pid.kp * error;

  // 3. Integral dengan anti-windup
  float integralLimit = (pid.ki > 0.0001f) ? 200.0f / pid.ki : 2000.0f;
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -integralLimit, integralLimit);
  float iOut = pid.ki * pid.integral;

  // 4. Derivative on Measurement (diff juga di-wrap agar tidak spike)
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
  // Constrain ke ±maxPwm (sama format return dengan pidCompute motor)
  return (int)constrain(output, (float)-500, (float)500);
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
