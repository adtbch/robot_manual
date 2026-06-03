#include "armbox_config.h"

void setupLimits() {
  for (size_t i = 0; i < motors.size(); i++) {
    pinMode(motors[i].limitPin, INPUT_PULLUP);
  }
  Serial.println("  Limit switches initialized");
}
