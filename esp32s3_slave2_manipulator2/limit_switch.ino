#include "armbox_config.h"

void setupLimits() {
  for (size_t i = 0; i < motors.size(); i++) {
    pinMode(motors[i].limitPin, INPUT_PULLUP);
  }
  Serial.println("  Limit switches initialized");
}

static unsigned long lastLimitCheck_ms[MOTOR_COUNT] = {0};
static constexpr unsigned long LIMIT_DEBOUNCE_MS = 2;

void checkLimitSwitches() {
  unsigned long now_ms = millis();
  for (size_t i = 0; i < motors.size(); i++) {
    if (getLastPwmValue(i) <= 0) {
      bool current = (digitalRead(motors[i].limitPin) == LOW);
      if (current) {
        if (lastLimitCheck_ms[i] == 0) {
          lastLimitCheck_ms[i] = now_ms;
        } else if (now_ms - lastLimitCheck_ms[i] >= LIMIT_DEBOUNCE_MS) {
          bool secondRead = (digitalRead(motors[i].limitPin) == LOW);
          if (secondRead) {
            pwmMotor(i, 0);
            stopMotorTarget(i);
            resetEncoderCount(i);
          }
          lastLimitCheck_ms[i] = 0;
        }
      } else {
        lastLimitCheck_ms[i] = 0;
      }
    }
  }
}
