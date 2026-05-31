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
// RPM Motor Control for 4 Wheels using the new computePID (vector-based)
// ============================================================

void rpmMotor(int rpm1, int rpm2, int rpm3, int rpm4) {
    // Guard 40ms: computePID hanya dijalankan 40Hz agar integral konsisten
    // dari mana pun fungsi ini dipanggil (auto-tuner, serial, atau loop utama).
    static unsigned long lastRpmTick = 0;
    if (millis() - lastRpmTick < 40) return;
    lastRpmTick = millis();

    // Gunakan konstanta dari robot_config.h
    const float minrpm_local = -500.0f;
    const float maxrpm_local = 500.0f;

    // Constrain input RPMs to valid ranges
    rpm1 = constrain(rpm1, (int)minrpm_local, (int)maxrpm_local);
    rpm2 = constrain(rpm2, (int)minrpm_local, (int)maxrpm_local);
    rpm3 = constrain(rpm3, (int)minrpm_local, (int)maxrpm_local);
    rpm4 = constrain(rpm4, (int)minrpm_local, (int)maxrpm_local);

    // Initializing PWM variables
    int pwmMotorDepanKanan = 0;
    int pwmMotorDepanKiri = 0;
    int pwmMotorBelakangKanan = 0;
    int pwmMotorBelakangKiri = 0;

    // Make sure pidData is initialized to prevent bounds issues
    if (pidData.size() < 4) {
        pidData.resize(4);
    }

    // ==========================================
    // MOTOR 1 (Indeks 0) - Front Right
    // ==========================================
    if (pidStates.size() > 0) {
        double kp = pidStates[0].kp;
        double ki = pidStates[0].ki;
        double kd = pidStates[0].kd;
        double minintegral = (ki > 0.0001) ? -1023.0 / ki : -2000.0;
        double maxintegral = (ki > 0.0001) ? 1023.0 / ki : 2000.0;

        float currentRpm = getEncoderVelocityRpm(0);

        if (rpm1 > 0) {
            pwmMotorDepanKanan = (int)computePID(0, (double)rpm1, (double)currentRpm, kp, ki, kd, minintegral, maxintegral);
            pwmMotorDepanKanan = constrain(pwmMotorDepanKanan, zeroPwm, maxPwm);
        } else if (rpm1 < 0) {
            pwmMotorDepanKanan = (int)computePID(0, (double)abs(rpm1), (double)currentRpm, kp, ki, kd, minintegral, maxintegral);
            pwmMotorDepanKanan = -pwmMotorDepanKanan;
            pwmMotorDepanKanan = constrain(pwmMotorDepanKanan, minPwm, zeroPwm);
        } else {
            pwmMotorDepanKanan = 0;
            pidData[0].error = 0.0;
            pidData[0].integral = 0.0;
        }
    }

    // ==========================================
    // MOTOR 2 (Indeks 1) - Front Left
    // ==========================================
    if (pidStates.size() > 1) {
        double kp = pidStates[1].kp;
        double ki = pidStates[1].ki;
        double kd = pidStates[1].kd;
        double minintegral = (ki > 0.0001) ? -1023.0 / ki : -2000.0;
        double maxintegral = (ki > 0.0001) ? 1023.0 / ki : 2000.0;

        float currentRpm = getEncoderVelocityRpm(1);

        if (rpm2 > 0) {
            pwmMotorDepanKiri = (int)computePID(1, (double)rpm2, (double)currentRpm, kp, ki, kd, minintegral, maxintegral);
            pwmMotorDepanKiri = constrain(pwmMotorDepanKiri, zeroPwm, maxPwm);
        } else if (rpm2 < 0) {
            pwmMotorDepanKiri = (int)computePID(1, (double)abs(rpm2), (double)currentRpm, kp, ki, kd, minintegral, maxintegral);
            pwmMotorDepanKiri = -pwmMotorDepanKiri;
            pwmMotorDepanKiri = constrain(pwmMotorDepanKiri, minPwm, zeroPwm);
        } else {
            pwmMotorDepanKiri = 0;
            pidData[1].error = 0.0;
            pidData[1].integral = 0.0;
        }
    }

    // ==========================================
    // MOTOR 3 (Indeks 2) - Back Right
    // ==========================================
    if (pidStates.size() > 2) {
        double kp = pidStates[2].kp;
        double ki = pidStates[2].ki;
        double kd = pidStates[2].kd;
        double minintegral = (ki > 0.0001) ? -1023.0 / ki : -2000.0;
        double maxintegral = (ki > 0.0001) ? 1023.0 / ki : 2000.0;

        float currentRpm = getEncoderVelocityRpm(2);

        if (rpm3 > 0) {
            pwmMotorBelakangKanan = (int)computePID(2, (double)rpm3, (double)currentRpm, kp, ki, kd, minintegral, maxintegral);
            pwmMotorBelakangKanan = constrain(pwmMotorBelakangKanan, zeroPwm, maxPwm);
        } else if (rpm3 < 0) {
            pwmMotorBelakangKanan = (int)computePID(2, (double)abs(rpm3), (double)currentRpm, kp, ki, kd, minintegral, maxintegral);
            pwmMotorBelakangKanan = -pwmMotorBelakangKanan;
            pwmMotorBelakangKanan = constrain(pwmMotorBelakangKanan, minPwm, zeroPwm);
        } else {
            pwmMotorBelakangKanan = 0;
            pidData[2].error = 0.0;
            pidData[2].integral = 0.0;
        }
    }

    // ==========================================
    // MOTOR 4 (Indeks 3) - Back Left
    // ==========================================
    if (pidStates.size() > 3) {
        double kp = pidStates[3].kp;
        double ki = pidStates[3].ki;
        double kd = pidStates[3].kd;
        double minintegral = (ki > 0.0001) ? -1023.0 / ki : -2000.0;
        double maxintegral = (ki > 0.0001) ? 1023.0 / ki : 2000.0;

        float currentRpm = getEncoderVelocityRpm(3);

        if (rpm4 > 0) {
            pwmMotorBelakangKiri = (int)computePID(3, (double)rpm4, (double)currentRpm, kp, ki, kd, minintegral, maxintegral);
            pwmMotorBelakangKiri = constrain(pwmMotorBelakangKiri, zeroPwm, maxPwm);
        } else if (rpm4 < 0) {
            pwmMotorBelakangKiri = (int)computePID(3, (double)abs(rpm4), (double)currentRpm, kp, ki, kd, minintegral, maxintegral);
            pwmMotorBelakangKiri = -pwmMotorBelakangKiri;
            pwmMotorBelakangKiri = constrain(pwmMotorBelakangKiri, minPwm, zeroPwm);
        } else {
            pwmMotorBelakangKiri = 0;
            pidData[3].error = 0.0;
            pidData[3].integral = 0.0;
        }
    }

    // Apply the computed PWM values to each motor physically
    pwmMotor(0, pwmMotorDepanKanan);
    pwmMotor(1, pwmMotorDepanKiri);
    pwmMotor(2, pwmMotorBelakangKanan);
    pwmMotor(3, pwmMotorBelakangKiri);
}
