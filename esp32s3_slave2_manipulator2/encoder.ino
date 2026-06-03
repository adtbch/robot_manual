#include "armbox_config.h"

// Encoder config vector (definition)
std::vector<EncoderConfig> encoders = {
  {encoderMotorAxisW_A, encoderMotorAxisW_B, 0},
  {encoderMotorAxisZ_A, encoderMotorAxisZ_B, 0},
  {encoderMotorAxisY_A, encoderMotorAxisY_B, 0}
};

void IRAM_ATTR Encoder(void *arg) {
  size_t idx = (size_t)(uintptr_t)arg;
  if (idx >= encoders.size()) return;

  int encA = digitalRead(encoders[idx].encoderPinA);
  int encB = digitalRead(encoders[idx].encoderPinB);

  if (encA == encB) {
    encoders[idx].count--;
  } else {
    encoders[idx].count++;
  }
}

void setupEncoders() {
  for (size_t i = 0; i < encoders.size(); i++) {
    pinMode(encoders[i].encoderPinA, INPUT_PULLUP);
    pinMode(encoders[i].encoderPinB, INPUT_PULLUP);

    attachInterruptArg(
      digitalPinToInterrupt(encoders[i].encoderPinA),
      Encoder,
      (void *)(uintptr_t)i,
      CHANGE
    );
  }
  Serial.println("  Encoders initialized");
}

void resetEncoderCount(uint8_t motorIndex) {
  if (motorIndex >= encoders.size()) return;
  encoders[motorIndex].count = 0;
}

long getEncoderCount(uint8_t motorIndex) {
  if (motorIndex >= encoders.size()) return 0;
  return encoders[motorIndex].count;
}

void printAllEncoders() {
  Serial.printf("Encoders: W=%ld | Z=%ld | Y=%ld\n",
                encoders[0].count, encoders[1].count, encoders[2].count);
}
