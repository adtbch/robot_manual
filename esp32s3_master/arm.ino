#include "robot_config.h"

bool readyArm = false;
MotorState motorArm = {false, false, false, false};

// Konstanta kontrol positioning
const int MOTOR_POSITION_TOLERANCE = 20;

// State target motor untuk positioning non-blocking
struct MotorTarget {
  long targetPosition;
  bool active;
};

static MotorTarget motorTargets[2] = {
  {0, false}, // 0: Motor X
  {0, false}  // 1: Motor Z
};

bool setHoming() {
    // Check limit switches dan update status homing
    if (digitalRead(limitSwitchAxisX) == LOW) {
        motorArm.xhoming = true;
        pwmMotor(0, 0); // Stop motor X
    }
    if (digitalRead(limitSwitchAxisY) == LOW) {
        motorArm.zhoming = true;
        pwmMotor(1, 0); // Stop motor Z
    }
    
    // Gerakkan motor yang belum homing
    if (!motorArm.xhoming) {
        pwmMotor(0, -HOMING_SPEED);
    }

    if (!motorArm.zhoming) {
        pwmMotor(1, -HOMING_SPEED);
    }

    // Return true hanya jika kedua motor sudah selesai homing
    return (motorArm.xhoming && motorArm.zhoming);
}

// Fungsi untuk set target motor dari serial / logic luar
void setMotorTarget(uint8_t motorIndex, long targetPosition) {
  if (motorIndex >= motors.size()) return;
  
  // Constrain target ke range aman
  if (motorIndex == 0) {
    targetPosition = constrain(targetPosition, 0, MAX_ENCODER_POSITION_X);
  } else if (motorIndex == 1) {
    targetPosition = constrain(targetPosition, 0, MAX_ENCODER_POSITION_Z);
  }

  motorTargets[motorIndex].targetPosition = targetPosition;
  motorTargets[motorIndex].active = true;
}

// Fungsi untuk cancel target motor (stop positioning)
void stopMotorTarget(uint8_t motorIndex) {
  if (motorIndex >= 2) return;
  motorTargets[motorIndex].active = false;
  pwmMotor(motorIndex, 0);
}

void stopAllMotorTargets() {
  stopMotorTarget(0);
  stopMotorTarget(1);
}

// Loop update positioning motor secara terus-menerus (non-blocking)
void updateMotorPositioning() {
  for (uint8_t i = 0; i < 2; i++) {
    if (!motorTargets[i].active) continue;

    long current = getEncoderCount(i);
    long error = motorTargets[i].targetPosition - current;

    if (abs(error) <= MOTOR_POSITION_TOLERANCE) {
      pwmMotor(i, 0);  // Target tercapai → stop
      motorTargets[i].active = false;
      Serial.printf("Motor %d reached target: %ld\n", i, current);
    } else if (error > 0) {
      pwmMotor(i, MOVE_SPEED); // Maju
    } else {
      pwmMotor(i, -MOVE_SPEED); // Mundur
    }
  }
}

// Fungsi untuk gerakkan arm ke posisi tengah menggunakan encoder feedback
bool moveToCenter() {
    setMotorTarget(0, CENTER_POSITION_X);
    setMotorTarget(1, CENTER_POSITION_Z);

    updateMotorPositioning();

    // Mengembalikan true jika kedua motor sudah tidak aktif (artinya sudah sampai target)
    return (!motorTargets[0].active && !motorTargets[1].active);
}

// Fungsi direct positioning instant (blocking/non-blocking helper)
bool moveTargetPosition(uint8_t motorIndex, long targetPosition) {
  setMotorTarget(motorIndex, targetPosition);
  updateMotorPositioning();
  return !motorTargets[motorIndex].active;
}
