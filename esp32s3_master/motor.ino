#include "robot_config.h"

// Motor config vector (definition)
std::vector<MotorConfig> motors = {
    {motorAxisX_A, motorAxisX_B, 0},  
    {motorAxisY_A, motorAxisY_B, 1},   
};

// Motor state objects (definition)

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

void motorStopAll() {
    for (size_t i = 0; i < motors.size(); i++) {
        pwmMotor(i, 0);
    }
} 