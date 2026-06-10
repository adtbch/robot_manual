/*
 * =====================================================================
 * FILE    : gripper_control.ino
 * PERAN   : 1. Toggle servo gripper buka/tutup dengan tombol Triangle
 *           2. Kontrol manual motor capit (axis X & Z) via D-pad
 *           3. Kontrol manual servo rotation (servo 0) via R2 + D-pad
 *
 *           ** HANYA AKTIF DI MODE_GRIPPING **
 *
 * SERVO GRIPPER (Triangle):
 *   - Boot → posisi BUKA (0 derajat)
 *   - Tekan Triangle → toggle (BUKA ↔ TUTUP)
 *
 * SERVO ROTATION:
 *   R2 + D-pad Up/Down → manual angle (step 10°)
 *   X + D-pad Right    → preset 185°
 *   X + D-pad Up       → preset 95°
 *   X + D-pad Left     → preset 5°
 *
 * MOTOR GRIPPER (D-pad tanpa R2/X):
 *   - D-pad Atas  → Motor 1 (axis X) maju
 *   - D-pad Bawah → Motor 1 (axis X) mundur
 *   - D-pad Kanan → Motor 0 (axis Z) maju
 *   - D-pad Kiri  → Motor 0 (axis Z) mundur
 *   - R1 hold = cepat, L1 hold = lambat
 * =====================================================================
 */

#include "robot_config.h"

// ============================================================
// SHARED DEFINES
// ============================================================
#define GRIPPER_SERVO_ID       1     // index di vector servos (servoGrib)
#define ROTATION_SERVO_ID      0     // index di vector servos (servoRotation)
#define GRIPPER_ANGLE_OPEN     0
#define GRIPPER_ANGLE_CLOSED   95
#define ROTATION_STEP          5     // derajat per langkah R2+Dpad
#define ROTATION_MIN           26
#define ROTATION_MAX           154
#define ROTATION_PRESET_LEFT   26
#define ROTATION_PRESET_UP     90
#define ROTATION_PRESET_RIGHT  154
#define GRIPPER_SPEED_DEFAULT  300
#define GRIPPER_SPEED_FAST     600
#define GRIPPER_SPEED_SLOW     200
#define GRIPPER_DEBOUNCE_MS    300

// ============================================================
// SHARED STATE
// ============================================================
static bool gGripperOpen = true;
static int  gRotationAngle = 0;         // sudut servo rotation saat ini
static unsigned long gLastToggleMs = 0;

// =====================================================================
//  SERVO GRIPPER — toggle buka/tutup dengan Triangle
// =====================================================================

void gripper_init() {
    setServoAngle(GRIPPER_SERVO_ID, GRIPPER_ANGLE_OPEN);
    gGripperOpen = true;
}

void gripper_tick(const ControlPacket &pkt) {
    if (currentMode != MODE_GRIPPING) return;

    static bool lastTriangle = false;
    bool triangleNow = (pkt.buttons & BTN_TRIANGLE) != 0;

    // Toggle servo gripper (Triangle)
    if (triangleNow && !lastTriangle) {
        unsigned long nowMs = millis();
        if (nowMs - gLastToggleMs >= GRIPPER_DEBOUNCE_MS) {
            gLastToggleMs = nowMs;
            gGripperOpen = !gGripperOpen;
            int angle = gGripperOpen ? GRIPPER_ANGLE_OPEN : GRIPPER_ANGLE_CLOSED;
            setServoAngle(GRIPPER_SERVO_ID, angle);
        }
    }

    lastTriangle = triangleNow;
}

// =====================================================================
//  MOTOR GRIPPER — manual jog via D-pad
// =====================================================================

void gripper_motor_tick(const ControlPacket &pkt) {
    if (currentMode != MODE_GRIPPING) {
        stopAllMotorTargets();
        return;
    }

    ActionInput ai = getActionInput(pkt);
    bool r2Held = ai.r2;

    // R2 + D-pad/analog → kontrol servo rotation manual
    if (r2Held) {
        static unsigned long lastServoMoveMs = 0;
        unsigned long nowMs = millis();
        if (nowMs - lastServoMoveMs >= 50) {
            lastServoMoveMs = nowMs;
            if (ai.right) {
                gRotationAngle = min(gRotationAngle + ROTATION_STEP, ROTATION_MAX);
                setServoAngle(ROTATION_SERVO_ID, gRotationAngle);
            } else if (ai.left) {
                gRotationAngle = max(gRotationAngle - ROTATION_STEP, ROTATION_MIN);
                setServoAngle(ROTATION_SERVO_ID, gRotationAngle);
            }
        }
        stopAllMotorTargets();
        return;
    }

    // X + D-pad/analog → preset servo rotation angle
    if (ai.x) {
        static bool lastUp = false, lastLeft = false, lastRight = false;
        bool upNow = ai.up, leftNow = ai.left, rightNow = ai.right;

        if (upNow && !lastUp) {
            gRotationAngle = ROTATION_PRESET_UP;
            setServoAngle(ROTATION_SERVO_ID, gRotationAngle);
        } else if (leftNow && !lastLeft) {
            gRotationAngle = ROTATION_PRESET_LEFT;
            setServoAngle(ROTATION_SERVO_ID, gRotationAngle);
        } else if (rightNow && !lastRight) {
            gRotationAngle = ROTATION_PRESET_RIGHT;
            setServoAngle(ROTATION_SERVO_ID, gRotationAngle);
        }

        lastUp = upNow;
        lastLeft = leftNow;
        lastRight = rightNow;
        stopAllMotorTargets();
        return;
    }

    // Tanpa R2/X → motor jog via D-pad/analog
    int speed = GRIPPER_SPEED_DEFAULT;
    if (ai.r1) {
        speed = GRIPPER_SPEED_FAST;
    } else if (ai.l1) {
        speed = GRIPPER_SPEED_SLOW;
    }

    // Limit switch (ACTIVE LOW — LOW = tersentuh)
    bool limitX = (digitalRead(limitSwitchAxisX) == LOW);
    bool limitY = (digitalRead(limitSwitchAxisY) == LOW);

    int enc0 = getEncoderCount(0);
    int enc1 = getEncoderCount(1);

    if (ai.up || ai.down || ai.left || ai.right) {
        stopAllMotorTargets();
    }

    // Motor 1: axis X (atas/bawah) — limitY = tidak bisa ke atas
    if (ai.up) {
        pwmMotor(1, limitY ? 0 : -speed);
    } else if (ai.down) {
        pwmMotor(1, enc1 >= 1257 ? 0 : speed);
    } else {
        pwmMotor(1, 0);
    }

    // Motor 0: axis Z (kiri/kanan) — limitX = tidak bisa ke kanan
    if (ai.right) {
        pwmMotor(0, limitX ? 0 : -speed*2);
    } else if (ai.left) {
        pwmMotor(0, enc0 >= 4695 ? 0 : speed*2);
    } else {
        pwmMotor(0, 0);
    }
}
