/*
 * File: arm_naik_turun.ino
 * Deskripsi: Kontrol motor naik turun (vertikal) dengan encoder, limit switch dan PID
 */

#include "armbox_config.h"

#if defined(ESP32)
static void initMotorNaikTurunPwm() {
  ledcSetup(MOTOR_NAIK_TURUN_RPWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR_NAIK_TURUN_RPWM, MOTOR_NAIK_TURUN_RPWM_CHANNEL);
  ledcWrite(MOTOR_NAIK_TURUN_RPWM_CHANNEL, 0);
  
  ledcSetup(MOTOR_NAIK_TURUN_LPWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR_NAIK_TURUN_LPWM, MOTOR_NAIK_TURUN_LPWM_CHANNEL);
  ledcWrite(MOTOR_NAIK_TURUN_LPWM_CHANNEL, 0);
}

static void writeMotorNaikTurunPwm(int rpwm, int lpwm) {
  ledcWrite(MOTOR_NAIK_TURUN_RPWM_CHANNEL, rpwm);
  ledcWrite(MOTOR_NAIK_TURUN_LPWM_CHANNEL, lpwm);
}
#else
static void initMotorNaikTurunPwm() {
  pinMode(MOTOR_NAIK_TURUN_RPWM, OUTPUT);
  pinMode(MOTOR_NAIK_TURUN_LPWM, OUTPUT);
  analogWrite(MOTOR_NAIK_TURUN_RPWM, 0);
  analogWrite(MOTOR_NAIK_TURUN_LPWM, 0);
}

static void writeMotorNaikTurunPwm(int rpwm, int lpwm) {
  analogWrite(MOTOR_NAIK_TURUN_RPWM, rpwm);
  analogWrite(MOTOR_NAIK_TURUN_LPWM, lpwm);
}
#endif

void setMotorNaikTurunRaw(int speedSigned) {
  int pwmValue = constrain(abs(speedSigned), 0, PWM_MAX);

  if (speedSigned > 0) {
    writeMotorNaikTurunPwm(pwmValue, 0);
  } else if (speedSigned < 0) {
    writeMotorNaikTurunPwm(0, pwmValue);
  } else {
    writeMotorNaikTurunPwm(0, 0);
  }
}

// Inisialisasi motor naik turun
void initArmNaikTurun() {
  initMotorNaikTurunPwm();
  pinMode(MOTOR_NAIK_TURUN_LIMIT, INPUT_PULLUP);
  stopMotorNaikTurun();
  Serial.println("✓ Motor Naik Turun initialized");
}

// Fungsi untuk naik
void motorNaik(int speed) {
  // Batasi speed
  speed = constrain(speed, 0, PWM_MAX);
  setMotorNaikTurunRaw(speed);
  
  Serial.print("Motor Naik Turun: Naik, Speed: ");
  Serial.println(speed);
}

// Fungsi untuk turun
void motorTurun(int speed) {
  // Cek limit switch
  if (digitalRead(MOTOR_NAIK_TURUN_LIMIT) == LOW) {
    stopMotorNaikTurun();
    Serial.println("Motor Naik Turun: Limit switch aktif!");
    return;
  }
  
  // Batasi speed
  speed = constrain(speed, 0, PWM_MAX);
  setMotorNaikTurunRaw(-speed);
  
  Serial.print("Motor Naik Turun: Turun, Speed: ");
  Serial.println(speed);
}

// Fungsi untuk stop motor naik turun
void stopMotorNaikTurun() {
  setMotorNaikTurunRaw(0);
  Serial.println("Motor Naik Turun: Stop");
}

// Fungsi untuk naik/turun ke posisi tertentu (menggunakan encoder)
void naikTurunKePosisi(long targetPos, int speed) {
  Serial.print("Motor Naik Turun: Menuju posisi ");
  Serial.println(targetPos);
  
  unsigned long timeout = millis() + 10000; // Timeout 10 detik
  
  while (abs(getEncoderNaikTurun() - targetPos) > POSITION_TOLERANCE) {
    // Timeout protection
    if (millis() > timeout) {
      Serial.println("Motor Naik Turun: Timeout!");
      break;
    }
    
    // Kontrol sederhana berdasarkan error
    long error = targetPos - getEncoderNaikTurun();
    int motorSpeed = speed;
    
    // Slow down saat mendekati target
    if (abs(error) < 100) {
      motorSpeed = map(abs(error), 0, 100, 80, speed);
    }
    
    // Arah motor berdasarkan error
    if (error > 0) {
      motorNaik(motorSpeed);
    } else {
      // Cek limit switch sebelum turun
      if (digitalRead(MOTOR_NAIK_TURUN_LIMIT) == LOW) {
        stopMotorNaikTurun();
        Serial.println("Motor Naik Turun: Limit tercapai!");
        break;
      }
      motorTurun(motorSpeed);
    }
    
    delay(10);
  }
  
  stopMotorNaikTurun();
  Serial.println("Motor Naik Turun: Posisi tercapai");
}

// Fungsi untuk mendapatkan posisi encoder
long getPosisiNaikTurun() {
  return getEncoderNaikTurun();
}
