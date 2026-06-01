// ============================================================
// ARM HOMING - Sequence Homing dengan Limit Switch
// ============================================================
// File: arm_homing.ino
// Fungsi: Homing sequence untuk sumbu X dan Z
//
// ALUR HOMING:
// 1. Gerakkan motor ke arah limit switch (lambat)
// 2. Tunggu limit switch tertekan
// 3. Stop motor, reset encoder ke 0
// 4. Gerakkan ke titik tengah (CENTER_POSITION)
// 5. Homing selesai
// ============================================================

#include "arm_config.h"

// ============================================================
// STATE MACHINE HOMING
// ============================================================

typedef enum {
  HOMING_IDLE = 0,
  HOMING_X_TO_LIMIT,       // Gerak sumbu X ke limit switch
  HOMING_X_WAIT_DEBOUNCE,  // Tunggu debounce limit switch X
  HOMING_X_BACKOFF,        // Mundur sedikit dari limit
  HOMING_X_TO_CENTER,      // Gerak sumbu X ke tengah
  HOMING_Z_TO_LIMIT,       // Gerak sumbu Z ke limit switch
  HOMING_Z_WAIT_DEBOUNCE,  // Tunggu debounce limit switch Z
  HOMING_Z_BACKOFF,        // Mundur sedikit dari limit
  HOMING_Z_TO_CENTER,      // Gerak sumbu Z ke tengah
  HOMING_DONE,             // Selesai
  HOMING_TIMEOUT_ERROR,    // Error timeout
} HomingState;

static HomingState gHomingState = HOMING_IDLE;
static uint32_t gHomingStartMs = 0;
static uint32_t gHomingStepMs = 0;

// Backoff pulsa setelah hit limit (mundur sedikit dari limit)
#define BACKOFF_PULSES  50

// ============================================================
// HOMING FUNCTIONS
// ============================================================

// Mulai sequence homing
bool armHomingStart() {
  if (gHomingState != HOMING_IDLE && gHomingState != HOMING_DONE
      && gHomingState != HOMING_TIMEOUT_ERROR) {
    Serial.println("Homing already in progress");
    return false;
  }

  Serial.println("");
  Serial.println("=================================");
  Serial.println("  ARM HOMING SEQUENCE DIMULAI");
  Serial.println("=================================");
  Serial.println("Step 1: Homing sumbu X ke limit...");

  // Reset state
  motorX.encoder.isHomed = false;
  motorZ.encoder.isHomed = false;
  motorX.state = MOTOR_HOMING;
  motorZ.state = MOTOR_IDLE;

  gHomingState = HOMING_X_TO_LIMIT;
  gHomingStartMs = millis();
  gHomingStepMs = millis();

  // Gerakkan X ke arah limit switch (negatif = kiri/arah limit)
  armMotorSetPWM(motorX, -HOMING_SPEED_PWM);

  return true;
}

// Cek apakah homing sedang berjalan
bool armHomingIsRunning() {
  return (gHomingState != HOMING_IDLE
          && gHomingState != HOMING_DONE
          && gHomingState != HOMING_TIMEOUT_ERROR);
}

// Cek apakah homing sudah selesai
bool armHomingIsComplete() {
  return gHomingState == HOMING_DONE;
}

// Tick homing (panggil di loop() setiap iterasi)
void armHomingTick() {
  if (gHomingState == HOMING_IDLE
      || gHomingState == HOMING_DONE
      || gHomingState == HOMING_TIMEOUT_ERROR) {
    return;
  }

  const uint32_t now = millis();

  // Cek timeout global
  if (now - gHomingStartMs > HOMING_TIMEOUT_MS) {
    armMotorStopAll();
    gHomingState = HOMING_TIMEOUT_ERROR;
    Serial.println("ERROR: Homing timeout!");
    return;
  }

  switch (gHomingState) {

    // ----------------------------------------
    case HOMING_X_TO_LIMIT:
      // Bergerak ke limit switch X
      if (armLimitSwitchPressed(motorX)) {
        armMotorStop(motorX);
        gHomingState = HOMING_X_WAIT_DEBOUNCE;
        gHomingStepMs = now;
        Serial.println("Limit switch X hit! Menunggu debounce...");
      }
      break;

    // ----------------------------------------
    case HOMING_X_WAIT_DEBOUNCE:
      // Tunggu debounce sebelum reset encoder
      if (now - gHomingStepMs >= DEBOUNCE_DELAY_MS) {
        // Konfirmasi limit masih tertekan
        if (armLimitSwitchPressed(motorX)) {
          // Reset encoder X ke 0 (ini adalah posisi home)
          armEncoderReset(motorX);
          motorX.encoder.homePosition = 0;

          // Backoff: mundur dari limit switch
          gHomingState = HOMING_X_BACKOFF;
          gHomingStepMs = now;
          armMotorSetPWM(motorX, HOMING_SPEED_PWM);  // Mundur dari limit
          Serial.println("Encoder X direset ke 0. Backoff dari limit...");
        } else {
          // Limit tidak tertekan lagi, ulangi homing X
          gHomingState = HOMING_X_TO_LIMIT;
          armMotorSetPWM(motorX, -HOMING_SPEED_PWM);
        }
      }
      break;

    // ----------------------------------------
    case HOMING_X_BACKOFF:
      // Mundur sampai encoder mencapai BACKOFF_PULSES
      if (armEncoderGetCount(motorX) >= BACKOFF_PULSES
          || !armLimitSwitchPressed(motorX)) {
        armMotorStop(motorX);
        motorX.encoder.isHomed = true;
        motorX.state = MOTOR_IDLE;

        // Mulai gerak ke tengah
        gHomingState = HOMING_X_TO_CENTER;
        motorX.encoder.targetPosition = motorX.encoder.centerPosition;
        motorX.state = MOTOR_MOVING;
        Serial.printf("Backoff X selesai. Menuju tengah (pos=%ld)...\n",
                      motorX.encoder.centerPosition);
      }
      break;

    // ----------------------------------------
    case HOMING_X_TO_CENTER:
      // Tunggu motor X sampai di tengah
      if (motorX.state == MOTOR_IDLE) {
        // Motor X selesai ke tengah, mulai homing Z
        gHomingState = HOMING_Z_TO_LIMIT;
        gHomingStepMs = now;
        motorZ.state = MOTOR_HOMING;

        // Gerakkan Z ke arah limit switch
        armMotorSetPWM(motorZ, -HOMING_SPEED_PWM);
        Serial.println("Motor X di tengah! Step 2: Homing sumbu Z ke limit...");
      }
      break;

    // ----------------------------------------
    case HOMING_Z_TO_LIMIT:
      if (armLimitSwitchPressed(motorZ)) {
        armMotorStop(motorZ);
        gHomingState = HOMING_Z_WAIT_DEBOUNCE;
        gHomingStepMs = now;
        Serial.println("Limit switch Z hit! Menunggu debounce...");
      }
      break;

    // ----------------------------------------
    case HOMING_Z_WAIT_DEBOUNCE:
      if (now - gHomingStepMs >= DEBOUNCE_DELAY_MS) {
        if (armLimitSwitchPressed(motorZ)) {
          armEncoderReset(motorZ);
          motorZ.encoder.homePosition = 0;

          gHomingState = HOMING_Z_BACKOFF;
          gHomingStepMs = now;
          armMotorSetPWM(motorZ, HOMING_SPEED_PWM);
          Serial.println("Encoder Z direset ke 0. Backoff dari limit...");
        } else {
          gHomingState = HOMING_Z_TO_LIMIT;
          armMotorSetPWM(motorZ, -HOMING_SPEED_PWM);
        }
      }
      break;

    // ----------------------------------------
    case HOMING_Z_BACKOFF:
      if (armEncoderGetCount(motorZ) >= BACKOFF_PULSES
          || !armLimitSwitchPressed(motorZ)) {
        armMotorStop(motorZ);
        motorZ.encoder.isHomed = true;
        motorZ.state = MOTOR_IDLE;

        gHomingState = HOMING_Z_TO_CENTER;
        motorZ.encoder.targetPosition = motorZ.encoder.centerPosition;
        motorZ.state = MOTOR_MOVING;
        Serial.printf("Backoff Z selesai. Menuju tengah (pos=%ld)...\n",
                      motorZ.encoder.centerPosition);
      }
      break;

    // ----------------------------------------
    case HOMING_Z_TO_CENTER:
      if (motorZ.state == MOTOR_IDLE) {
        gHomingState = HOMING_DONE;
        Serial.println("=================================");
        Serial.println("  ARM HOMING SELESAI!");
        Serial.printf("  X pos: %ld\n", armEncoderGetCount(motorX));
        Serial.printf("  Z pos: %ld\n", armEncoderGetCount(motorZ));
        Serial.println("=================================");
      }
      break;

    default:
      break;
  }
}
