/*
 * =====================================================================
 * FILE    : gripper_control.ino
 * PERAN   : 1. Toggle servo gripper buka/tutup dengan tombol Triangle
 *           2. Kontrol manual motor capit (axis X & Z) via D-pad
 *           3. Kontrol manual servo rotation (servo 0) via R2 + D-pad
 *           4. X + D-pad → recall preset sudut (tersimpan di NVS)
 *           5. Save/Reset via command byte dari Controller
 *           6. Share + Circle/Square → save servo+motor (via command byte)
 *           7. Circle/Square → recall servo+motor sequence
 *
 *           ** HANYA AKTIF DI MODE_GRIPPING **
 *
 * SERVO GRIPPER (Triangle):
 *   - Boot → posisi BUKA (0 derajat)
 *   - Tekan Triangle → toggle (BUKA ↔ TUTUP)
 *
 * SERVO ROTATION:
 *   R2 + D-pad Right/Left → manual angle (step 5°)
 *   X + D-pad Up/Left/Right → recall preset (NVS)
 *   Save/Reset dikirim dari Controller via command byte
 *
 * SEMI-AUTO PRESETS:
 *   Share + Circle → simpan servo angle + motor height ke Circle slot
 *   Share + Square → simpan servo angle + motor height ke Square slot
 *   Circle → recall: servo dulu, baru motor
 *   Square → recall: servo dulu, baru motor
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
#define GRIPPER_ANGLE_CLOSED   85
#define ROTATION_STEP          5     // derajat per langkah R2+Dpad
#define ROTATION_MIN           0
#define ROTATION_MAX           185
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
//  SERVO GRIPPER — toggle buka/tutup dengan Circle
// =====================================================================

void gripper_init() {
    servoPresetsLoad();
    semiAutoPresetsLoad();
    setServoAngle(GRIPPER_SERVO_ID, GRIPPER_ANGLE_OPEN);
    gGripperOpen = true;
}

void gripper_tick(const ControlPacket &pkt) {
    if (currentMode != MODE_GRIPPING) return;

    bool circleNow = (pkt.buttons & BTN_CIRCLE)  != 0;
    bool squareNow = (pkt.buttons & BTN_SQUARE)  != 0;
    bool shareHeld = (pkt.buttons & BTN_SHARE)   != 0;
    bool triangleNow = (pkt.buttons & BTN_TRIANGLE) != 0;

    // Circle (tanpa Share) → recall semi-auto preset (jika tidak sedang sequence)
    if (circleNow && !shareHeld && !semiAutoPresetIsActive()) {
        static bool lastCircleRecall = false;
        if (!lastCircleRecall) {
            semiAutoPresetRecallCircle();
        }
        lastCircleRecall = true;
        return;
    }

    // Square (tanpa Share) → recall semi-auto preset (jika tidak sedang sequence)
    if (squareNow && !shareHeld && !semiAutoPresetIsActive()) {
        static bool lastSquareRecall = false;
        if (!lastSquareRecall) {
            semiAutoPresetRecallSquare();
        }
        lastSquareRecall = true;
        return;
    }

    // Toggle servo gripper (Triangle)
    static bool lastTriangle = false;
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

    bool r2Held = (pkt.buttons & BTN_R2) != 0;
    bool up     = (pkt.buttons & BTN_UP)    != 0;
    bool down   = (pkt.buttons & BTN_DOWN)  != 0;
    bool left   = (pkt.buttons & BTN_LEFT)  != 0;
    bool right  = (pkt.buttons & BTN_RIGHT) != 0;

    // R2 + D-pad Up/Down → kontrol servo rotation manual
    if (r2Held) {
        static unsigned long lastServoMoveMs = 0;
        unsigned long nowMs = millis();
        if (nowMs - lastServoMoveMs >= 50) {
            lastServoMoveMs = nowMs;
            if (right) {
                gRotationAngle = min(gRotationAngle + ROTATION_STEP, ROTATION_MAX);
                setServoAngle(ROTATION_SERVO_ID, gRotationAngle);
            } else if (left) {
                gRotationAngle = max(gRotationAngle - ROTATION_STEP, ROTATION_MIN);
                setServoAngle(ROTATION_SERVO_ID, gRotationAngle);
            }
        }
        stopAllMotorTargets();
        return;
    }

    // X + D-pad → recall preset servo rotation angle (Up/Left/Right saja)
    bool xHeld = (pkt.buttons & BTN_CROSS) != 0;
    if (xHeld) {
        static bool lastUp = false, lastLeft = false, lastRight = false;
        bool upNow = up, leftNow = left, rightNow = right;

        if (upNow && !lastUp) {
            gRotationAngle = servoPresetsGetUp();
            setServoAngle(ROTATION_SERVO_ID, gRotationAngle);
        } else if (leftNow && !lastLeft) {
            gRotationAngle = servoPresetsGetLeft();
            setServoAngle(ROTATION_SERVO_ID, gRotationAngle);
        } else if (rightNow && !lastRight) {
            gRotationAngle = servoPresetsGetRight();
            setServoAngle(ROTATION_SERVO_ID, gRotationAngle);
        }

        lastUp = upNow;
        lastLeft = leftNow;
        lastRight = rightNow;
        stopAllMotorTargets();
        return;
    }

    // Tanpa R2/X → motor jog via D-pad
    int speed = GRIPPER_SPEED_DEFAULT;
    if (pkt.buttons & BTN_R1) {
        speed = GRIPPER_SPEED_FAST;
    } else if (pkt.buttons & BTN_L1) {
        speed = GRIPPER_SPEED_SLOW;
    }

    // Limit switch (ACTIVE LOW — LOW = tersentuh)
    bool limitX = (digitalRead(limitSwitchAxisX) == LOW);
    bool limitY = (digitalRead(limitSwitchAxisY) == LOW);

    int enc0 = getEncoderCount(0);
    int enc1 = getEncoderCount(1);

    if (up || down || left || right) {
        stopAllMotorTargets();
    }

    // Motor 1: axis X (atas/bawah) — limitY = tidak bisa ke atas
    if (up) {
        pwmMotor(1, limitY ? 0 : -speed);
    } else if (down) {
        pwmMotor(1, enc1 >= 1257 ? 0 : speed);
    } else {
        pwmMotor(1, 0);
    }

    // Motor 0: axis Z (kiri/kanan) — limitX = tidak bisa ke kanan
    if (right) {
        pwmMotor(0, limitX ? 0 : -speed*2);
    } else if (left) {
        pwmMotor(0, enc0 >= 4695 ? 0 : speed*2);
    } else {
        pwmMotor(0, 0);
    }
}
