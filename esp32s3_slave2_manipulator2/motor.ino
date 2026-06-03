#include "armbox_config.h"

// Motor config vector (definition)
std::vector<MotorConfig> motors = {
  // Sumbu W (Putar): limitDir=+1
  {motorAxisW_A, motorAxisW_B, 0,
   encoderMotorAxisW_A, encoderMotorAxisW_B, limitSwitchAxisW, +1},
  // Sumbu Z (Naik Turun): limitDir=-1
  {motorAxisZ_A, motorAxisZ_B, 1, encoderMotorAxisZ_A, encoderMotorAxisZ_B, limitSwitchAxisZ, -1},
  // Sumbu Y (Maju Mundur): limitDir=-1
  {motorAxisY_A, motorAxisY_B, 2, encoderMotorAxisY_A, encoderMotorAxisY_B, limitSwitchAxisY, -1},
};

MotorState motorArm = {false, false, false};

void SetupMotors() {
  for (size_t i = 0; i < motors.size(); i++) {
    pinMode(motors[i].pinDirection, OUTPUT);
    digitalWrite(motors[i].pinDirection, LOW);

    ledcSetup(motors[i].ledc_channel, pwmFrequency, pwmResolution);
    ledcAttachPin(motors[i].pwmPin, motors[i].ledc_channel);
    ledcWrite(motors[i].ledc_channel, 0);
    // pwmMotor(i, 0);
  }
  Serial.println("  Motors initialized");
}

void pwmMotor(int idMotor, int pwmValue) {
  if (idMotor < 0 || (size_t)idMotor >= motors.size()) return;

  pwmValue = constrain(pwmValue, minPwm, maxPwm);

  if (pwmValue > 0) {
    ledcWrite(motors[idMotor].ledc_channel, pwmValue);
    digitalWrite(motors[idMotor].pinDirection, LOW);
  } else if (pwmValue < 0) {
    ledcWrite(motors[idMotor].ledc_channel, maxPwm + pwmValue);
    digitalWrite(motors[idMotor].pinDirection, HIGH);
  } else {
    ledcWrite(motors[idMotor].ledc_channel, 0);
    digitalWrite(motors[idMotor].pinDirection, LOW);
  }
}

void motorStopAll() {
  for (size_t i = 0; i < motors.size(); i++) {
    pwmMotor(i, 0);
  }
}
