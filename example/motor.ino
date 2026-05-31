
#include "robot_config.h"

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
    uint16_t duty = (uint16_t)abs(pwmValue);

    if (pwmValue > 0) {
        ledcWrite(motors[idMotor].ledc_channel, duty);
        digitalWrite(motors[idMotor].pin_direction, LOW);
    } else if (pwmValue < 0) {
        ledcWrite(motors[idMotor].ledc_channel, duty);
        digitalWrite(motors[idMotor].pin_direction, HIGH);
    } else {
        ledcWrite(motors[idMotor].ledc_channel, 0);
        digitalWrite(motors[idMotor].pin_direction, LOW);
    }
}