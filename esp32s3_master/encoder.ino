#include "robot_config.h"

// ============================================================
// Per-motor velocity storage (updated in convertEncoderToRPM)
// ============================================================
// Encoder config vector (definition)
std::vector<EncoderConfig> encoders = {
  {encoderMotorAxisX_A, encoderMotorAxisX_B, 0},  // 0: motor axis X
  {encoderMotorAxisY_A, encoderMotorAxisY_B, 0}   // 1: motor axis Y
};


void IRAM_ATTR Encoder(void *arg) {
  size_t idx = (size_t)(uintptr_t)arg;
  if (idx >= encoders.size()) {
    return;
  }

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
}

// Reset encoder count untuk motor tertentu (setelah homing)
void resetEncoderCount(uint8_t motorIndex) {
  if (motorIndex >= encoders.size()) {
    return;
  }
  encoders[motorIndex].count = 0;
}

// Baca posisi encoder motor tertentu
long getEncoderCount(uint8_t motorIndex) {
  if (motorIndex >= encoders.size()) {
    return 0;
  }
  return encoders[motorIndex].count;
}

