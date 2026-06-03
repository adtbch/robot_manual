#include "armbox_config.h"

// Servo config vector (definition)
std::vector<ServoConfig> servos = {
  {SERVO_PIN, SERVO_CHANNEL}
};

// Current servo angle
int currentServoAngle = servoHomeAngle;

void setupServos() {
  for (size_t i = 0; i < servos.size(); i++) {
    ledcSetup(servos[i].ledc_channel, servoFrequency, servoResolution);
    ledcAttachPin(servos[i].servoPin, servos[i].ledc_channel);
  }
  setServoAngle(0, servoHomeAngle);
  Serial.println("  Servo initialized");
}

void setServoAngle(int idServo, int angle) {
  if (idServo < 0 || (size_t)idServo >= servos.size()) return;

  angle = constrain(angle, servoMinAngle, servoMaxAngle);

  long pulseWidth = map(angle, servoMinAngle, servoMaxAngle, servoMinPulseUs, servoMaxPulseUs);
  long duty = (pulseWidth * ((1 << servoResolution) - 1)) / 20000;

  ledcWrite(servos[idServo].ledc_channel, duty);
  currentServoAngle = angle;
}

int getCurrentServoAngle() {
  return currentServoAngle;
}
