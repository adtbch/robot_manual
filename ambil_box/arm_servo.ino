/*
 * File: arm_servo.ino
 * Deskripsi: Kontrol servo untuk gripper atau end effector
 */

#include "armbox_config.h"

#if defined(ESP32)
static const int SERVO_CHANNEL = 0;
static const int SERVO_PERIOD_US = 20000;
static const int SERVO_TIMER_MAX = (1 << SERVO_TIMER_BITS) - 1;
#endif

int currentServoAngle = SERVO_HOME_ANGLE;

static int clampServoAngle(int angle) {
  return constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
}

static int angleToPulseUs(int angle) {
  int clamped = clampServoAngle(angle);
  return map(clamped, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
}

#if defined(ESP32)
static void writeServoPulseUs(int pulseUs) {
  int bounded = constrain(pulseUs, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  uint32_t duty = (uint32_t)bounded * (uint32_t)SERVO_TIMER_MAX / (uint32_t)SERVO_PERIOD_US;
  ledcWrite(SERVO_CHANNEL, duty);
}
#else
static void writeServoPulseUs(int pulseUs) {
  int bounded = constrain(pulseUs, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(bounded);
  digitalWrite(SERVO_PIN, LOW);
  delayMicroseconds(20000 - bounded);
}

static void refreshServoPulses(int pulseUs, int cycles) {
  for (int i = 0; i < cycles; i++) {
    writeServoPulseUs(pulseUs);
  }
}
#endif

// Inisialisasi servo
void initArmServo() {
#if defined(ESP32)
  ledcSetup(SERVO_CHANNEL, SERVO_PWM_FREQ, SERVO_TIMER_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
  writeServoPulseUs(angleToPulseUs(SERVO_HOME_ANGLE));
#else
  pinMode(SERVO_PIN, OUTPUT);
  refreshServoPulses(angleToPulseUs(SERVO_HOME_ANGLE), 3);
#endif
  currentServoAngle = SERVO_HOME_ANGLE;
  
  Serial.println("Servo initialized");
  Serial.print("Servo: Posisi awal ");
  Serial.print(SERVO_HOME_ANGLE);
  Serial.println(" derajat");
}

// Fungsi untuk menggerakkan servo ke sudut tertentu
void setServoAngle(int angle) {
  int clamped = clampServoAngle(angle);
  int pulseUs = angleToPulseUs(clamped);

#if defined(ESP32)
  writeServoPulseUs(pulseUs);
#else
  refreshServoPulses(pulseUs, 2);
#endif
  currentServoAngle = clamped;
  
  Serial.print("Servo: Bergerak ke ");
  Serial.print(angle);
  Serial.println(" derajat");
  
  // Delay untuk memberikan waktu servo bergerak
  delay(15);
}

// Fungsi untuk menggerakkan servo secara smooth
void moveServoSmooth(int targetAngle, int delayTime) {
  targetAngle = clampServoAngle(targetAngle);
  
  Serial.print("Servo: Smooth move ke ");
  Serial.print(targetAngle);
  Serial.println(" derajat");
  
  if (currentServoAngle < targetAngle) {
    // Bergerak naik
    for (int angle = currentServoAngle; angle <= targetAngle; angle++) {
      setServoAngle(angle);
      delay(delayTime);
    }
  } else {
    // Bergerak turun
    for (int angle = currentServoAngle; angle >= targetAngle; angle--) {
      setServoAngle(angle);
      delay(delayTime);
    }
  }
  
  Serial.println("Servo: Smooth move selesai");
}

// Fungsi untuk buka gripper
void bukaGripper() {
  Serial.println("Gripper: Membuka...");
  moveServoSmooth(SERVO_MAX_ANGLE, 10);
  Serial.println("Gripper: Terbuka");
}

// Fungsi untuk tutup gripper
void tutupGripper() {
  Serial.println("Gripper: Menutup...");
  moveServoSmooth(SERVO_MIN_ANGLE, 10);
  Serial.println("Gripper: Tertutup");
}

// Fungsi untuk posisi home servo
void servoHome() {
  Serial.println("Servo: Kembali ke home...");
  moveServoSmooth(SERVO_HOME_ANGLE, 10);
  Serial.println("Servo: Home position");
}

// Fungsi untuk mendapatkan sudut servo saat ini
int getCurrentServoAngle() {
  return currentServoAngle;
}

// Fungsi untuk gripper setengah buka
void gripperSetengah() {
  int halfAngle = (SERVO_MAX_ANGLE + SERVO_MIN_ANGLE) / 2;
  Serial.println("Gripper: Setengah buka...");
  moveServoSmooth(halfAngle, 10);
  Serial.println("Gripper: Posisi setengah");
}

// Fungsi untuk detach servo (hemat energi)
void detachServo() {
#if defined(ESP32)
  ledcDetachPin(SERVO_PIN);
#else
  pinMode(SERVO_PIN, INPUT);
#endif
  Serial.println("Servo: Detached");
}

// Fungsi untuk attach servo kembali
void attachServo() {
#if defined(ESP32)
  ledcSetup(SERVO_CHANNEL, SERVO_PWM_FREQ, SERVO_TIMER_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
  writeServoPulseUs(angleToPulseUs(currentServoAngle));
#else
  pinMode(SERVO_PIN, OUTPUT);
  refreshServoPulses(angleToPulseUs(currentServoAngle), 3);
#endif
  Serial.println("Servo: Attached");
}
