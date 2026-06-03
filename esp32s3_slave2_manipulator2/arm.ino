#include "armbox_config.h"

// ============================================================
// Arm Positioning (non-blocking)
// ============================================================

struct MotorTarget {
  long targetPosition;
  bool active;
};

static MotorTarget motorTargets[MOTOR_COUNT] = {
  {0, false},
  {0, false},
  {0, false}
};

void setMotorTarget(uint8_t motorIndex, long targetPosition) {
  if (motorIndex >= motors.size()) return;

  targetPosition = constrain(targetPosition, 0, MAX_ENCODER_POSITION);

  motorTargets[motorIndex].targetPosition = targetPosition;
  motorTargets[motorIndex].active = true;
}

void stopMotorTarget(uint8_t motorIndex) {
  if (motorIndex >= motors.size()) return;
  motorTargets[motorIndex].active = false;
  pwmMotor(motorIndex, 0);
}

void stopAllMotorTargets() {
  for (size_t i = 0; i < motors.size(); i++) {
    stopMotorTarget(i);
  }
}

void updateMotorPositioning() {
  for (size_t i = 0; i < motors.size(); i++) {
    if (!motorTargets[i].active) continue;

    long current = getEncoderCount(i);
    long error = motorTargets[i].targetPosition - current;

    if (abs(error) <= MOTOR_POSITION_TOLERANCE) {
      pwmMotor(i, 0);
      motorTargets[i].active = false;
      char axis = (i == MOTOR_W) ? 'W' : ((i == MOTOR_Z) ? 'Z' : 'Y');
      Serial.printf("Motor %c reached target: %ld\n", axis, current);
    } else if (error > 0) {
      pwmMotor(i, MOVE_SPEED);
    } else {
      pwmMotor(i, -MOVE_SPEED);
    }
  }
}

// ============================================================
// Homing (blocking - runs in setup)
// ============================================================

bool setHoming() {
  // Check limit switches dan update status homing
  if (digitalRead(motors[0].limitPin) == LOW) {
    motorArm.homed[0] = true;
    pwmMotor(0, 0);
  }
  if (digitalRead(motors[1].limitPin) == LOW) {
    motorArm.homed[1] = true;
    pwmMotor(1, 0);
  }
  if (digitalRead(motors[2].limitPin) == LOW) {
    motorArm.homed[2] = true;
    pwmMotor(2, 0);
  }
  
  // Gerakkan motor yang belum homing
  if (!motorArm.homed[0]) {
    pwmMotor(0, -HOMING_SPEED);
  }

  if (!motorArm.homed[1]) {
    pwmMotor(1, -HOMING_SPEED * 2.5);
  }

  if (!motorArm.homed[2]) {
    pwmMotor(2, -HOMING_SPEED);
  }

  // Return true hanya jika semua motor sudah selesai homing
  return (motorArm.homed[0] && motorArm.homed[1] && motorArm.homed[2]);
}

bool homingMotor(uint8_t id) {
  if (id >= motors.size()) return false;

  char axis = (id == MOTOR_W) ? 'W' : ((id == MOTOR_Z) ? 'Z' : 'Y');
  Serial.printf("Homing motor %c...\n", axis);
  motorArm.homed[id] = false;

  // Phase 1: Find limit switch
  unsigned long timeout = millis() + HOMING_TIMEOUT;
  while (digitalRead(motors[id].limitPin) == HIGH) {
    if (millis() > timeout) {
      pwmMotor(id, 0);
      Serial.println("  ERROR: Homing timeout!");
      return false;
    }
    pwmMotor(id, HOMING_SPEED * motors[id].limitDir);
  }
  pwmMotor(id, 0);

  // Phase 2: Backoff
  unsigned long backoffTime = millis() + 500;
  while (millis() < backoffTime) {
    pwmMotor(id, -HOMING_SPEED * motors[id].limitDir / 2);
  }
  pwmMotor(id, 0);

  // Phase 3: Slow approach
  while (digitalRead(motors[id].limitPin) == HIGH) {
    pwmMotor(id, HOMING_SPEED * motors[id].limitDir / 3);
  }
  pwmMotor(id, 0);

  // Reset encoder at home position
  resetEncoderCount(id);
  motorArm.homed[id] = true;
  Serial.printf("  Motor %c homed!\n", axis);
  return true;
}

bool homingAllMotors() {
  for (size_t i = 0; i < motors.size(); i++) {
    if (!homingMotor(i)) return false;
  }
  Serial.println("All motors homed!");
  return true;
}

bool isAllMotorHomed() {
  for (size_t i = 0; i < motors.size(); i++) {
    if (!motorArm.homed[i]) return false;
  }
  return true;
}

void printHomingStatus() {
  Serial.println("=== HOMING STATUS ===");
  for (size_t i = 0; i < motors.size(); i++) {
    char axis = (i == MOTOR_W) ? 'W' : ((i == MOTOR_Z) ? 'Z' : 'Y');
    Serial.printf("  Motor %c: %s\n", axis, motorArm.homed[i] ? "HOMED" : "NOT HOMED");
  }
}
