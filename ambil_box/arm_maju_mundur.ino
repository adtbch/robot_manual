/*
 * File: arm_maju_mundur.ino
 * Deskripsi: Kontrol motor maju mundur (horizontal) dengan encoder, limit switch dan PID
 */

#include "armbox_config.h"

#if defined(ESP32)
static void initMotorMajuMundurPwm() {
  ledcSetup(MOTOR_MAJU_MUNDUR_RPWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR_MAJU_MUNDUR_RPWM, MOTOR_MAJU_MUNDUR_RPWM_CHANNEL);
  ledcWrite(MOTOR_MAJU_MUNDUR_RPWM_CHANNEL, 0);
  
  ledcSetup(MOTOR_MAJU_MUNDUR_LPWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR_MAJU_MUNDUR_LPWM, MOTOR_MAJU_MUNDUR_LPWM_CHANNEL);
  ledcWrite(MOTOR_MAJU_MUNDUR_LPWM_CHANNEL, 0);
}

static void writeMotorMajuMundurPwm(int rpwm, int lpwm) {
  ledcWrite(MOTOR_MAJU_MUNDUR_RPWM_CHANNEL, rpwm);
  ledcWrite(MOTOR_MAJU_MUNDUR_LPWM_CHANNEL, lpwm);
}
#else
static void initMotorMajuMundurPwm() {
  pinMode(MOTOR_MAJU_MUNDUR_RPWM, OUTPUT);
  pinMode(MOTOR_MAJU_MUNDUR_LPWM, OUTPUT);
  analogWrite(MOTOR_MAJU_MUNDUR_RPWM, 0);
  analogWrite(MOTOR_MAJU_MUNDUR_LPWM, 0);
}

static void writeMotorMajuMundurPwm(int rpwm, int lpwm) {
  analogWrite(MOTOR_MAJU_MUNDUR_RPWM, rpwm);
  analogWrite(MOTOR_MAJU_MUNDUR_LPWM, lpwm);
}
#endif

void setMotorMajuMundurRaw(int speedSigned) {
  int pwmValue = constrain(abs(speedSigned), 0, PWM_MAX);

  if (speedSigned > 0) {
    writeMotorMajuMundurPwm(pwmValue, 0);
  } else if (speedSigned < 0) {
    writeMotorMajuMundurPwm(0, pwmValue);
  } else {
    writeMotorMajuMundurPwm(0, 0);
  }
}

// Inisialisasi motor maju mundur
void initArmMajuMundur() {
  initMotorMajuMundurPwm();
  pinMode(MOTOR_MAJU_MUNDUR_LIMIT, INPUT_PULLUP);
  stopMotorMajuMundur();
  Serial.println("✓ Motor Maju Mundur initialized");
}

// Fungsi untuk maju
void motorMaju(int speed) {
  // Batasi speed
  speed = constrain(speed, 0, PWM_MAX);
  setMotorMajuMundurRaw(speed);
  
  Serial.print("Motor Maju Mundur: Maju, Speed: ");
  Serial.println(speed);
}

// Fungsi untuk mundur
void motorMundur(int speed) {
  // Cek limit switch
  if (digitalRead(MOTOR_MAJU_MUNDUR_LIMIT) == LOW) {
    stopMotorMajuMundur();
    Serial.println("Motor Maju Mundur: Limit switch aktif!");
    return;
  }
  
  // Batasi speed
  speed = constrain(speed, 0, PWM_MAX);
  setMotorMajuMundurRaw(-speed);
  
  Serial.print("Motor Maju Mundur: Mundur, Speed: ");
  Serial.println(speed);
}

// Fungsi untuk stop motor maju mundur
void stopMotorMajuMundur() {
  setMotorMajuMundurRaw(0);
  Serial.println("Motor Maju Mundur: Stop");
}

// Fungsi untuk maju/mundur ke posisi tertentu (menggunakan encoder)
void majuMundurKePosisi(long targetPos, int speed) {
  Serial.print("Motor Maju Mundur: Menuju posisi ");
  Serial.println(targetPos);
  
  unsigned long timeout = millis() + 10000; // Timeout 10 detik
  
  while (abs(getEncoderMajuMundur() - targetPos) > POSITION_TOLERANCE) {
    // Timeout protection
    if (millis() > timeout) {
      Serial.println("Motor Maju Mundur: Timeout!");
      break;
    }
    
    // Kontrol sederhana berdasarkan error
    long error = targetPos - getEncoderMajuMundur();
    int motorSpeed = speed;
    
    // Slow down saat mendekati target
    if (abs(error) < 100) {
      motorSpeed = map(abs(error), 0, 100, 80, speed);
    }
    
    // Arah motor berdasarkan error
    if (error > 0) {
      motorMaju(motorSpeed);
    } else {
      // Cek limit switch sebelum mundur
      if (digitalRead(MOTOR_MAJU_MUNDUR_LIMIT) == LOW) {
        stopMotorMajuMundur();
        Serial.println("Motor Maju Mundur: Limit tercapai!");
        break;
      }
      motorMundur(motorSpeed);
    }
    
    delay(10);
  }
  
  stopMotorMajuMundur();
  Serial.println("Motor Maju Mundur: Posisi tercapai");
}

// Fungsi untuk mendapatkan posisi encoder
long getPosisiMajuMundur() {
  return getEncoderMajuMundur();
}
