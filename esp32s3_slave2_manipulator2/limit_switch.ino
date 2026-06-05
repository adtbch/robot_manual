#include "armbox_config.h"

void setupLimits() {
  for (size_t i = 0; i < motors.size(); i++) {
    pinMode(motors[i].limitPin, INPUT_PULLUP);
  }
  Serial.println("  Limit switches initialized");
}

void checkLimitSwitches() {
  for (size_t i = 0; i < motors.size(); i++) {
    if (getLastPwmValue(i) <= 0) {
      // Debounce: baca 2x, hanya trigger jika keduanya LOW
      bool firstRead = (digitalRead(motors[i].limitPin) == LOW);
      if (firstRead) {
        delay(2);
        bool secondRead = (digitalRead(motors[i].limitPin) == LOW);
        if (secondRead) {
          pwmMotor(i, 0);
          stopMotorTarget(i);
          resetEncoderCount(i);
        }
      }
    }
  }
}
