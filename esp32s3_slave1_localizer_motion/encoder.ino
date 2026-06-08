#include "robot_config.h"

// ============================================================
// Per-motor velocity storage (updated in convertEncoderToRPM)
// ============================================================
const unsigned long intervalRpm = 40; // Dipercepat dari 100ms ke 40ms untuk respon instan
unsigned long millisRpm = 0;
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
        long current_count;
        noInterrupts();
        current_count = encoders[i].count;
        interrupts();

        // 1. Kalkulasi Delta (Akurat 100%, pulsa tidak pernah hilang)
        long delta = current_count - encoders[i].prev_count;
        encoders[i].last_delta = delta;  // simpan untuk confidence filter
        encoders[i].prev_count = current_count;

        // 2. Dapatkan waktu aktual untuk kalkulasi dinamis
        unsigned long now = millis();
        unsigned long elapsed_ms = now - encoders[i].last_time;
        if (elapsed_ms == 0) elapsed_ms = 1; // Mencegah pembagian dengan nol
        encoders[i].last_time = now;

        float rpm_raw = ((float)delta * 60000.0f) / ((float)elapsed_ms * (float)encoderMotorPpr);

        // LOGIKA ZERO-BYPASS: Jika delta pulsa benar-benar 0 (diam), bypass filter dan paksa RPM = 0 instan
        if (delta == 0) {
            rpm_filtered[i] = 0.0f;
        } else {
            // Filter IIR dengan alpha 0.45f agar lebih responsif tanpa kehilangan kestabilan
            const float alpha = 0.45f;
            rpm_filtered[i] = alpha * rpm_raw + (1.0f - alpha) * rpm_filtered[i];
        }

        int rpm_final = (int)round(rpm_filtered[i]);

        // Debug encoder (matikan sementara untuk fokus debug WSN-31)
        // Serial.print("Encoder Motor "); Serial.print(i); Serial.print(": ");
        // Serial.print(current_count); Serial.print(" pulses | RPM: "); Serial.println(rpm_final);
        // if (abs(delta) > 0) {
        //     Serial.printf("Motor %d - RPM Raw: %.1f | RPM Filtered: %.1f\n", i, rpm_raw, rpm_filtered[i]);
        // }
      
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

// ============================================================
// Yaw rate dari 4 encoder (rad/s) — untuk complementary filter
// Formula dari kinematik balik Mecanum:
//   yaw_rate = (-vFL + vFR + vBL -vBR) / (2 * (Lx + Ly))
// ============================================================

float getEncoderYawRateRads() {
    // Linear wheel velocities in m/s
    float vFR = getEncoderVelocityRadS(0) * radiusRoda;  // Front Right
    float vFL = getEncoderVelocityRadS(1) * radiusRoda;  // Front Left
    float vBR = getEncoderVelocityRadS(2) * radiusRoda;  // Back Right
    float vBL = getEncoderVelocityRadS(3) * radiusRoda;  // Back Left

    float denom = 2.0f * (ROBOT_Lx + ROBOT_Ly);
    if (fabs(denom) < 0.001f) return 0.0f;

    return (-vFL + vFR + vBL - vBR) / denom;
}

// ============================================================
// Confidence encoder berdasarkan delta tick maksimum
// Lebih banyak tick → SNR lebih baik → lebih percaya encoder
// ============================================================

float getEncoderConfidence() {
    long maxTick = 0;
    for (size_t i = 0; i < encoders.size(); i++) {
        long absTick = labs(encoders[i].last_delta);
        if (absTick > maxTick) maxTick = absTick;
    }

    if (maxTick < 2)   return 0.0f;   // noise floor → percaya gyro saja
    if (maxTick < 5)   return 0.2f;   // sangat lambat
    if (maxTick < 15)  return 0.5f;   // lambat
    if (maxTick < 40)  return 0.7f;   // sedang
    return 0.9f;                       // cepat, akurat (risiko slip kecil)
}
