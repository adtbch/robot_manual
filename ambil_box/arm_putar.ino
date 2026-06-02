/*
 * File: arm_putar.ino
 * Deskripsi: Kontrol motor putar (rotasi) dengan encoder, limit switch dan PID
 */

#include "armbox_config.h"

#if defined(ESP32)
static void initMotorPutarPwm() {
  ledcSetup(MOTOR_PUTAR_RPWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR_PUTAR_RPWM, MOTOR_PUTAR_RPWM_CHANNEL);
  ledcWrite(MOTOR_PUTAR_RPWM_CHANNEL, 0);
  
  ledcSetup(MOTOR_PUTAR_LPWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR_PUTAR_LPWM, MOTOR_PUTAR_LPWM_CHANNEL);
  ledcWrite(MOTOR_PUTAR_LPWM_CHANNEL, 0);
}

static void writeMotorPutarPwm(int rpwm, int lpwm) {
  ledcWrite(MOTOR_PUTAR_RPWM_CHANNEL, rpwm);
  ledcWrite(MOTOR_PUTAR_LPWM_CHANNEL, lpwm);
}
#else
static void initMotorPutarPwm() {
  pinMode(MOTOR_PUTAR_RPWM, OUTPUT);
  pinMode(MOTOR_PUTAR_LPWM, OUTPUT);
  analogWrite(MOTOR_PUTAR_RPWM, 0);
  analogWrite(MOTOR_PUTAR_LPWM, 0);
}

static void writeMotorPutarPwm(int rpwm, int lpwm) {
  analogWrite(MOTOR_PUTAR_RPWM, rpwm);
  analogWrite(MOTOR_PUTAR_LPWM, lpwm);
}
#endif

void setMotorPutarRaw(int speedSigned) {
  int pwmValue = constrain(abs(speedSigned), 0, PWM_MAX);

  if (speedSigned > 0) {
    writeMotorPutarPwm(pwmValue, 0);
  } else if (speedSigned < 0) {
    writeMotorPutarPwm(0, pwmValue);
  } else {
    writeMotorPutarPwm(0, 0);
  }
}

// Inisialisasi motor putar
void initArmPutar() {
  initMotorPutarPwm();
  pinMode(MOTOR_PUTAR_LIMIT, INPUT_PULLUP);
  stopMotorPutar();
  Serial.println("✓ Motor Putar initialized");
}

// Fungsi untuk memutar motor ke kanan (CW)
void putarKanan(int speed) {
  // Cek limit switch
  if (digitalRead(MOTOR_PUTAR_LIMIT) == LOW) {
    stopMotorPutar();
    Serial.println("Motor Putar: Limit switch aktif!");
    return;
  }
  
  // Batasi speed
  speed = constrain(speed, 0, PWM_MAX);
  setMotorPutarRaw(speed);
  
  Serial.print("Motor Putar: Kanan, Speed: ");
  Serial.println(speed);
}

// Fungsi untuk memutar motor ke kiri (CCW)
void putarKiri(int speed) {
  // Batasi speed
  speed = constrain(speed, 0, PWM_MAX);
  setMotorPutarRaw(-speed);
  
  Serial.print("Motor Putar: Kiri, Speed: ");
  Serial.println(speed);
}

// Fungsi untuk stop motor putar
void stopMotorPutar() {
  setMotorPutarRaw(0);
  Serial.println("Motor Putar: Stop");
}

// Fungsi untuk putar ke posisi tertentu (menggunakan encoder)
void putarKePosisi(long targetPos, int speed) {
  Serial.print("Motor Putar: Menuju posisi ");
  Serial.println(targetPos);
  
  unsigned long timeout = millis() + 10000; // Timeout 10 detik
  
  while (abs(getEncoderPutar() - targetPos) > POSITION_TOLERANCE) {
    // Timeout protection
    if (millis() > timeout) {
      Serial.println("Motor Putar: Timeout!");
      break;
    }
    
    // Kontrol sederhana berdasarkan error
    long error = targetPos - getEncoderPutar();
    int motorSpeed = speed;
    
    // Slow down saat mendekati target
    if (abs(error) < 100) {
      motorSpeed = map(abs(error), 0, 100, 80, speed);
    }
    
    // Arah motor berdasarkan error
    if (error > 0) {
      putarKanan(motorSpeed);
    } else {
      putarKiri(motorSpeed);
    }
    
    // Cek limit switch
    if (digitalRead(MOTOR_PUTAR_LIMIT) == LOW) {
      stopMotorPutar();
      Serial.println("Motor Putar: Limit tercapai!");
      break;
    }
    
    delay(10);
  }
  
  stopMotorPutar();
  Serial.println("Motor Putar: Posisi tercapai");
}

// Fungsi untuk mendapatkan posisi encoder
long getPosisiPutar() {
  return getEncoderPutar();
}
