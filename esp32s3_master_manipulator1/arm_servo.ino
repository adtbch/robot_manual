// ============================================================
// ARM SERVO CONTROL - Servo Rotasi & Capit
// ============================================================
// File: arm_servo.ino
// Fungsi: Kontrol 2 servo untuk rotasi capit dan buka/tutup capit
// ============================================================

#include "arm_config.h"
#include <ESP32Servo.h>

// ============================================================
// GLOBAL VARIABLES
// ============================================================

// Objek servo
Servo servoRotate;   // Servo rotasi capit
Servo servoGripper;  // Servo buka/tutup capit

// State servo
static int gRotateAngle = ROTATE_CENTER_ANGLE;
static int gGripperAngle = GRIPPER_OPEN_ANGLE;
static bool gGripperIsOpen = true;

// ============================================================
// INISIALISASI SERVO
// ============================================================

bool armServoInit() {
  Serial.println("=== ARM SERVO INIT ===");

  // Setup servo rotasi
  servoRotate.setPeriodHertz(SERVO_FREQUENCY);
  servoRotate.attach(SERVO_ROTATE_PIN, SERVO_MIN_US, SERVO_MAX_US);
  
  // Setup servo capit
  servoGripper.setPeriodHertz(SERVO_FREQUENCY);
  servoGripper.attach(SERVO_GRIPPER_PIN, SERVO_MIN_US, SERVO_MAX_US);

  // Set posisi awal (tengah untuk rotasi, terbuka untuk capit)
  servoRotate.write(ROTATE_CENTER_ANGLE);
  servoGripper.write(GRIPPER_OPEN_ANGLE);
  
  gRotateAngle = ROTATE_CENTER_ANGLE;
  gGripperAngle = GRIPPER_OPEN_ANGLE;
  gGripperIsOpen = true;

  Serial.printf("Servo Rotate Pin: %d (angle=%d)\n", SERVO_ROTATE_PIN, gRotateAngle);
  Serial.printf("Servo Gripper Pin: %d (angle=%d, open=%d)\n", 
                SERVO_GRIPPER_PIN, gGripperAngle, gGripperIsOpen);
  Serial.println("======================");

  return true;
}

// ============================================================
// KONTROL SERVO ROTASI
// ============================================================

// Set sudut servo rotasi (0-180 derajat)
void armServoRotateSetAngle(int angle) {
  // Clamp angle
  if (angle < ROTATE_MIN_ANGLE) angle = ROTATE_MIN_ANGLE;
  if (angle > ROTATE_MAX_ANGLE) angle = ROTATE_MAX_ANGLE;

  servoRotate.write(angle);
  gRotateAngle = angle;

  Serial.printf("[SERVO ROTATE] Angle set to %d degrees\n", angle);
}

// Get sudut servo rotasi saat ini
int armServoRotateGetAngle() {
  return gRotateAngle;
}

// ============================================================
// KONTROL SERVO CAPIT
// ============================================================

// Buka capit
void armServoGripperOpen() {
  servoGripper.write(GRIPPER_OPEN_ANGLE);
  gGripperAngle = GRIPPER_OPEN_ANGLE;
  gGripperIsOpen = true;

  Serial.println("[SERVO GRIPPER] Opened");
}

// Tutup capit
void armServoGripperClose() {
  servoGripper.write(GRIPPER_CLOSE_ANGLE);
  gGripperAngle = GRIPPER_CLOSE_ANGLE;
  gGripperIsOpen = false;

  Serial.println("[SERVO GRIPPER] Closed");
}

// Set sudut servo capit manual (0-180 derajat)
void armServoGripperSetAngle(int angle) {
  // Clamp angle
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  servoGripper.write(angle);
  gGripperAngle = angle;

  // Update status open/close berdasarkan angle
  gGripperIsOpen = (angle < (GRIPPER_OPEN_ANGLE + GRIPPER_CLOSE_ANGLE) / 2);

  Serial.printf("[SERVO GRIPPER] Angle set to %d degrees\n", angle);
}

// Get sudut servo capit saat ini
int armServoGripperGetAngle() {
  return gGripperAngle;
}

// Cek apakah capit terbuka
bool armServoGripperIsOpen() {
  return gGripperIsOpen;
}
