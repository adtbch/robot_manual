// ============================================================
// ARM ENCODER - Pembacaan Encoder Motor X & Z
// ============================================================
// File: arm_encoder.ino
// Fungsi: ISR encoder untuk sumbu X dan Z (interrupt-based)
// ============================================================

#include "arm_config.h"

// ============================================================
// ISR ENCODER - IRAM_ATTR agar berjalan di IRAM (lebih cepat)
// ============================================================

// ISR Encoder Motor X
void IRAM_ATTR armEncoderX_ISR() {
  // Baca fasa B untuk menentukan arah
  if (digitalRead(motorX.encPinB) == HIGH) {
    motorX.encoder.count++;
  } else {
    motorX.encoder.count--;
  }
}

// ISR Encoder Motor Z
void IRAM_ATTR armEncoderZ_ISR() {
  // Baca fasa B untuk menentukan arah
  if (digitalRead(motorZ.encPinB) == HIGH) {
    motorZ.encoder.count++;
  } else {
    motorZ.encoder.count--;
  }
}

// ============================================================
// INISIALISASI ENCODER
// ============================================================

bool armEncoderInit() {
  Serial.println("=== ARM ENCODER INIT ===");

  // Setup encoder Motor X
  pinMode(motorX.encPinA, INPUT_PULLUP);
  pinMode(motorX.encPinB, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(motorX.encPinA),
    armEncoderX_ISR,
    RISING
  );
  motorX.encoder.count = 0;

  // Setup encoder Motor Z
  pinMode(motorZ.encPinA, INPUT_PULLUP);
  pinMode(motorZ.encPinB, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(motorZ.encPinA),
    armEncoderZ_ISR,
    RISING
  );
  motorZ.encoder.count = 0;

  Serial.printf("Encoder X: A=%d B=%d\n", MOTOR_X_ENC_A, MOTOR_X_ENC_B);
  Serial.printf("Encoder Z: A=%d B=%d\n", MOTOR_Z_ENC_A, MOTOR_Z_ENC_B);
  Serial.println("========================");

  return true;
}
