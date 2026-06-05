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

void updateEncoderCounts() {
  // Baca dari volatile ISR count → global vars
  encoderMotorW = encoders[0].count;
  encoderMotorZ = encoders[1].count;
  encoderMotorY = encoders[2].count;
}

void printAllEncoders() {
  Serial.printf("Encoders: W=%ld | Z=%ld | Y=%ld\n",
                encoderMotorW, encoderMotorZ, encoderMotorY);
}
