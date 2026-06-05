#include "armbox_config.h"

// Definisi global state PID untuk Motor W
PIDState pidW = {2.5f, 0.05f, 0.1f, 0.0f, 0.0f, 0.0f};

int pidCompute(PIDState &pid, float target, float current, float dt) {
    // 1) DETEKSI PERUBAHAN TARGET BESAR: reset integral jika target berubah > 20%
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
    if (dt > 0.0f) {
        // Negative derivative karena jika current naik, kita kurangi output
        dOut = -pid.kd * (current - pid.lastError) / dt;
    }

    float output = pOut + iOut + dOut;
    int pwmOutput = (int)constrain(output, (float)minPwm, (float)maxPwm);

    // Simpan current sebagai "lastError" untuk derivative next cycle
    pid.lastError = current;

    return pwmOutput;
}
