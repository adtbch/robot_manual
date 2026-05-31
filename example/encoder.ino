#include "robot_config.h"

const unsigned long intervalRpm = 100;
unsigned long millisRpm = 0;

// ============================================================
// Per-motor velocity storage (updated in convertEncoderToRPM)
// ============================================================
static std::vector<float> motorVelocityRpm;

// Encoder config vector (definition)
std::vector<EncoderConfig> encoders = {
  {encoderMotorDepanKanan_A, encoderMotorDepanKanan_B, 0},  // 0: front_right_wheel
  {encoderMotorDepanKiri_A, encoderMotorDepanKiri_B, 0},   // 1: front_left_wheel (biasanya dibalik)
  {encoderMotorBelakangKanan_A, encoderMotorBelakangKanan_B, 0}, // 2: back_right_wheel
  {encoderMotorBelakangKiri_A, encoderMotorBelakangKiri_B, 0}    // 3: back_left_wheel (biasanya dibalik)
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

bool checkInterval(unsigned long interval, unsigned long &lastTime) {
  if (millis() - lastTime >= interval) {
    lastTime = millis();
    return true;
  }
  return false;
}

void convertEncoderToRPM() {
    static std::vector<float> rpm_filtered;
    if (rpm_filtered.size() != encoders.size()) {
    rpm_filtered.assign(encoders.size(), 0.0f);
    }

  updateMotorVelocityCache();

    if (checkInterval(intervalRpm, millisRpm)) {
    for (size_t i = 0; i < encoders.size(); i++) {
        long enc;
        noInterrupts();
        enc = encoders[i].count;
        encoders[i].count = 0;
        interrupts();

        float rpm_raw = ((float)enc * 60000.0f) / ((float)intervalRpm * (float)encoderMotorPpr);

        const float alpha = 0.3f;
        rpm_filtered[i] = alpha * rpm_raw + (1.0f - alpha) * rpm_filtered[i];

        int rpm_final = (int)round(rpm_filtered[i]);

        Serial.print("Encoder Motor "); Serial.print(i); Serial.print(": ");
        Serial.print(enc); Serial.print(" pulses | RPM: "); Serial.println(rpm_final);

            if (abs(enc) > 0) {
                Serial.printf("Motor %d - RPM Raw: %.1f | RPM Filtered: %.1f\n", i, rpm_raw, rpm_filtered[i]);
            }
      
      motorVelocityRpm[i] = rpm_filtered[i];
        }
    }
}

// ============================================================
// Velocity getter functions (untuk PID controller & auto-tuner)
// ============================================================

void updateMotorVelocityCache() {
  if (motorVelocityRpm.size() != encoders.size()) {
    motorVelocityRpm.assign(encoders.size(), 0.0f);
  }
}

float getEncoderVelocityRpm(int motorIdx) {
  if (motorIdx < 0 || (size_t)motorIdx >= motorVelocityRpm.size()) {
    return 0.0f;
  }
  return motorVelocityRpm[motorIdx];
}

float getEncoderVelocityRadS(int motorIdx) {
  float rpm = getEncoderVelocityRpm(motorIdx);
  return rpm * kRpmToRadPerSec;
}
