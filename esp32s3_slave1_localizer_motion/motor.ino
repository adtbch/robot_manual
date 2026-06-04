#include "robot_config.h"
extern std::vector<PIDState> pidStates;

// Motor config vector (definition)
std::vector<MotorConfig> motors = {
    {motorDepanKanan_A, motorDepanKanan_B, 0},  // 0: front_right_wheel
    {motorDepanKiri_A, motorDepanKiri_B, 1},   // 1: front_left_wheel (biasanya dibalik)
    {motorBelakangKanan_A, motorBelakangKanan_B, 2}, // 2: back_right_wheel
    {motorBelakangKiri_A, motorBelakangKiri_B, 3}    // 3: back_left_wheel (biasanya dibalik)
};

void SetupMotors() {
    for (size_t i = 0; i < motors.size(); i++) {
        pinMode(motors[i].pin_direction, OUTPUT);
        digitalWrite(motors[i].pin_direction, LOW);

        ledcSetup(motors[i].ledc_channel, pwmFrequency, pwmResolution);
        ledcAttachPin(motors[i].pin_pwm, motors[i].ledc_channel);
        ledcWrite(motors[i].ledc_channel, 0);
    }
}

void pwmMotor(int idMotor, int pwmValue) {
    if (idMotor < 0 || (size_t)idMotor >= motors.size()) {
        return;
    }

    pwmValue = constrain(pwmValue, minPwm, maxPwm);

    if (pwmValue > 0) {
        ledcWrite(motors[idMotor].ledc_channel, pwmValue);
        digitalWrite(motors[idMotor].pin_direction, LOW);
    } else if (pwmValue < 0) {
        ledcWrite(motors[idMotor].ledc_channel, maxPwm + pwmValue);
        digitalWrite(motors[idMotor].pin_direction, HIGH);
    } else {
        ledcWrite(motors[idMotor].ledc_channel, 0);
        digitalWrite(motors[idMotor].pin_direction, LOW);
    }
}

// ============================================================
// RPM Motor Control for 4 Wheels
// Compatible interface: tetap rpmMotor(rpm1, rpm2, rpm3, rpm4)
// Internal target ramp + PID reset policy untuk perpindahan target ekstrem
// ============================================================

static int rampedPidPwm(int motorIdx, int requestedRpm, float dt) {
    static float activeTarget[4] = {0, 0, 0, 0};
    static int lastRequested[4] = {0, 0, 0, 0};

    if (motorIdx < 0 || motorIdx >= 4) return 0;

    // Target 0 = stop bersih, buang semua history PID motor itu
    if (requestedRpm == 0) {
        activeTarget[motorIdx] = 0.0f;
        lastRequested[motorIdx] = 0;
        pidResetOne(motorIdx);
        return 0;
    }

    const int prevReq = lastRequested[motorIdx];
    const bool signChanged = (prevReq != 0) && ((prevReq > 0) != (requestedRpm > 0));

    // Ganti arah = mulai kontrol dari state bersih (reset PID & target mulai dari 0)
    if (signChanged) {
        pidResetOne(motorIdx);
        activeTarget[motorIdx] = 0.0f;
    }
    lastRequested[motorIdx] = requestedRpm;

    // RAMP — percentage of delta (50% per tick)
    // Dekat target  → langkah kecil  (smooth finish)
    // Jauh target   → langkah besar  (fast catch-up)
    // Self-regulating, tidak perlu tuning ramp rate manual
    float delta = (float)requestedRpm - activeTarget[motorIdx];
    if (fabsf(delta) < 2.0f) {
        // Sudah cukup dekat (< 2 RPM) → set langsung
        activeTarget[motorIdx] = (float)requestedRpm;
    } else {
        // Bergerak 50% dari sisa jarak tiap tick
        float step = fabsf(delta) * 0.5f;
        step = fmaxf(step, 1.0f);           // minimal 1 RPM agar tidak stagnasi
        step = fminf(step, fabsf(delta));  // jangan overshoot
        activeTarget[motorIdx] += (delta > 0.0f) ? step : -step;
    }

    return pidCompute(motorIdx, activeTarget[motorIdx], dt);
}

void rpmMotor(int rpm1, int rpm2, int rpm3, int rpm4) {
    // Guard 40ms: pidCompute hanya dijalankan 40Hz agar integral konsisten
    // UNIFIKASI: gunakan satu timer untuk guard DAN dt computation
    static uint32_t lastTickMs = 0;
    uint32_t nowMs = millis();
    
    // Guard: skip jika belum 40ms
    if (lastTickMs > 0 && (nowMs - lastTickMs) < 40) return;
    
    // Compute dt (konsisten dengan guard 40ms)
    float dt = (lastTickMs > 0) ? (nowMs - lastTickMs) / 1000.0f : 0.04f;
    lastTickMs = nowMs;
    dt = constrain(dt, 0.01f, 0.1f);

    // Constrain input RPMs
    rpm1 = constrain(rpm1, (int)minrpm, (int)maxrpm);
    rpm2 = constrain(rpm2, (int)minrpm, (int)maxrpm);
    rpm3 = constrain(rpm3, (int)minrpm, (int)maxrpm);
    rpm4 = constrain(rpm4, (int)minrpm, (int)maxrpm);

    // PID pakai target yang sudah diramp internal; caller tetap kirim target RPM langsung
    int pwmMotorDepanKanan    = rampedPidPwm(0, rpm1, dt);
    int pwmMotorDepanKiri     = rampedPidPwm(1, rpm2, dt);
    int pwmMotorBelakangKanan = rampedPidPwm(2, rpm3, dt);
    int pwmMotorBelakangKiri  = rampedPidPwm(3, rpm4, dt);

    // Apply PWM
    pwmMotor(0, pwmMotorDepanKanan);
    pwmMotor(1, pwmMotorDepanKiri);
    pwmMotor(2, pwmMotorBelakangKanan);
    pwmMotor(3, pwmMotorBelakangKiri);
}
