#include "armbox_config.h"
#include <Preferences.h>

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

static unsigned long lastTimeW = 0;

static const char* PID_NVS_NS = "pid_w_cal";

void initPidW() {
  Preferences prefs;
  prefs.begin(PID_NVS_NS, true);
  pidW.kp = prefs.getFloat("kp", 2.5f);
  pidW.ki = prefs.getFloat("ki", 0.05f);
  pidW.kd = prefs.getFloat("kd", 0.1f);
  prefs.end();
  Serial.printf("Loaded PID W from NVS: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pidW.kp, pidW.ki, pidW.kd);
}

void setPidW(float p, float i, float d) {
  pidW.kp = p;
  pidW.ki = i;
  pidW.kd = d;
  Preferences prefs;
  prefs.begin(PID_NVS_NS, false);
  prefs.putFloat("kp", pidW.kp);
  prefs.putFloat("ki", pidW.ki);
  prefs.putFloat("kd", pidW.kd);
  prefs.end();
  Serial.printf("Saved PID W to NVS: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pidW.kp, pidW.ki, pidW.kd);
}

void showPidW() {
  Serial.printf("Current PID W: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pidW.kp, pidW.ki, pidW.kd);
}

// Helper: dapatkan safety limit per-axis
static long getAxisMaxPos(uint8_t motorIndex) {
  switch (motorIndex) {
    case MOTOR_W: return MAX_POS_W;
    case MOTOR_Y: return MAX_POS_Y;
    case MOTOR_Z: return MAX_POS_Z;
    default:      return MAX_ENCODER_POSITION;
  }
}

// Safety check: apakah posisi dalam batas aman per-axis?
bool isPositionSafe(uint8_t motorIndex, long pos) {
  long maxPos = getAxisMaxPos(motorIndex);
  return (pos >= 0 && pos <= maxPos);
}

void setMotorTarget(uint8_t motorIndex, long targetPosition) {
  if (motorIndex >= motors.size()) return;

  // Safety: constrain per-axis limit
  long maxPos = getAxisMaxPos(motorIndex);
  targetPosition = constrain(targetPosition, 0, maxPos);

  motorTargets[motorIndex].targetPosition = targetPosition;
  motorTargets[motorIndex].active = true;

  char axis = (motorIndex == MOTOR_W) ? 'W' : ((motorIndex == MOTOR_Z) ? 'Z' : 'Y');
  Serial.printf("Motor %c target: %ld (safe limit: %ld)\n", axis, targetPosition, maxPos);
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

    long current;
    if (i == MOTOR_W) current = encoderMotorW;
    else if (i == MOTOR_Z) current = encoderMotorZ;
    else current = encoderMotorY;

    long error = motorTargets[i].targetPosition - current;

    // Jika target = 0 dan limit belum tertekan, jangan stop berdasarkan toleransi
    // Biarkan limit switch yang hentikan motor
    bool atLimitTarget = (motorTargets[i].targetPosition == 0);
    bool limitActive = (digitalRead(motors[i].limitPin) == LOW);

    if (abs(error) <= MOTOR_POSITION_TOLERANCE && !(atLimitTarget && !limitActive)) {
      pwmMotor(i, 0);
      char axis = (i == MOTOR_W) ? 'W' : ((i == MOTOR_Z) ? 'Z' : 'Y');
      
      if (i == MOTOR_W) {
        // Motor W tetap active = true (holding position), tapi PWM dimatikan (deadband)
        pidW.integral = 0.0f;
        pidW.lastError = 0.0f;
        lastTimeW = 0;
      } else {
        // Motor Z & Y dimatikan targetnya
        motorTargets[i].active = false;
        Serial.printf("Motor %c reached target: %ld\n", axis, current);
      }
    } else {
      if (i == MOTOR_W) {
        // ================= PID UNTUK MOTOR W =================
        unsigned long now = millis();
        float dt = (lastTimeW == 0) ? 0.04f : (now - lastTimeW) / 1000.0f;
        lastTimeW = now;

        // Hitung PID menggunakan fungsi generic dari pid.ino
        int pwmOutput = pidCompute(pidW, motorTargets[i].targetPosition, current, dt);

        // Limit output PWM ke +/- 150 (setengah kecepatan)
        pwmOutput = constrain(pwmOutput, -MOVE_SPEED * 1.5, MOVE_SPEED * 1.5);

        pwmMotor(i, pwmOutput);
      } else {
        // ================= BANG-BANG UNTUK MOTOR LAIN =================
        if (error > 0) {
          pwmMotor(i, MOVE_SPEED);
        } else {
          pwmMotor(i, -MOVE_SPEED);
        }
      }
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
    pwmMotor(0, -HOMING_SPEED * 2.5);
  }

  if (!motorArm.homed[1]) {
    pwmMotor(1, -HOMING_SPEED * 2.5);
  }

  if (!motorArm.homed[2]) {
    pwmMotor(2, -HOMING_SPEED * 2.5);
  }
  
  setServoAngle(0, servoHomeAngle);
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

// ============================================================
// Multi-axis movement (non-blocking)
// ============================================================

// Set target 3 sumbu sekaligus dengan safety per-axis
void moveToPosition(long posW, long posZ, long posY) {
  setMotorTarget(MOTOR_W, posW);
  setMotorTarget(MOTOR_Z, posZ);
  setMotorTarget(MOTOR_Y, posY);
  Serial.printf("moveToPosition: W=%ld Z=%ld Y=%ld\n", posW, posZ, posY);
}

// Singleton helper: move satu sumbu, return false jika masih bergerak
bool moveTargetPosition(uint8_t motorIndex, long targetPosition) {
  // setMotorTarget(motorIndex, targetPosition);
  updateMotorPositioning();
  return !motorTargets[motorIndex].active;
}
