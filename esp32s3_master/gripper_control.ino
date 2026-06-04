/*
 * =====================================================================
 * FILE    : gripper_control.ino
 * PERAN   : 1. Toggle servo gripper buka/tutup dengan tombol Circle
 *           2. Kontrol manual motor capit (axis X & Z) via D-pad + X
 *
 * SERVO GRIPPER (Circle):
 *   - Boot → posisi BUKA (0 derajat)
 *   - Tekan Circle → toggle (BUKA ↔ TUTUP)
 *
 * MOTOR GRIPPER (X + D-pad):
 *   - Tahan X + D-pad Atas  → Motor 0 (axis X) maju
 *   - Tahan X + D-pad Bawah → Motor 0 (axis X) mundur
 *   - Tahan X + D-pad Kanan → Motor 1 (axis Z) maju
 *   - Tahan X + D-pad Kiri  → Motor 1 (axis Z) mundur
 *   - Lepas X → motor berhenti
 * =====================================================================
 */

#include "robot_config.h"

// ============================================================
// SHARED DEFINES
// ============================================================
#define GRIPPER_SERVO_ID       1     // index di vector servos (servoGrib)
#define GRIPPER_ANGLE_OPEN     0
#define GRIPPER_ANGLE_CLOSED   90
#define GRIPPER_SPEED_DEFAULT  300
#define GRIPPER_SPEED_FAST     600
#define GRIPPER_SPEED_SLOW     100
#define GRIPPER_DEBOUNCE_MS    300

// ============================================================
// SHARED STATE
// ============================================================
static bool gGripperOpen = true;
static unsigned long gLastToggleMs = 0;

// =====================================================================
//  SERVO GRIPPER — toggle buka/tutup dengan Circle
// =====================================================================

void gripper_init() {
    setServoAngle(GRIPPER_SERVO_ID, GRIPPER_ANGLE_OPEN);
    gGripperOpen = true;
    Serial.println("[GRIPPER] Init — posisi BUKA");
}

void gripper_tick(const ControlPacket &pkt) {
    static bool lastCircle = false;
    bool circleNow = (pkt.buttons & BTN_CIRCLE) != 0;

    if (circleNow && !lastCircle) {
        unsigned long nowMs = millis();
        if (nowMs - gLastToggleMs >= GRIPPER_DEBOUNCE_MS) {
            gLastToggleMs = nowMs;
            gGripperOpen = !gGripperOpen;

            int angle = gGripperOpen ? GRIPPER_ANGLE_OPEN : GRIPPER_ANGLE_CLOSED;
            setServoAngle(GRIPPER_SERVO_ID, angle);
        }
    }

    lastCircle = circleNow;
}

// =====================================================================
//  MOTOR GRIPPER — manual jog via X + D-pad
// =====================================================================

void gripper_motor_tick(const ControlPacket &pkt) {
    bool xHeld = (pkt.buttons & BTN_CROSS) != 0;

    if (!xHeld) {
        stopAllMotorTargets();
        return;
    }

    int speed = GRIPPER_SPEED_DEFAULT;
    if (pkt.buttons & BTN_R1) {
        speed = GRIPPER_SPEED_FAST;
    } else if (pkt.buttons & BTN_L1) {
        speed = GRIPPER_SPEED_SLOW;
    }

    bool up    = (pkt.buttons & BTN_UP)    != 0;
    bool down  = (pkt.buttons & BTN_DOWN)  != 0;
    bool left  = (pkt.buttons & BTN_LEFT)  != 0;
    bool right = (pkt.buttons & BTN_RIGHT) != 0;

    // Limit switch (ACTIVE LOW — LOW = tersentuh)
    bool limitX = (digitalRead(limitSwitchAxisX) == LOW);
    bool limitY = (digitalRead(limitSwitchAxisY) == LOW);

    int enc0 = getEncoderCount(0); // Axis Z (horizontal)
    int enc1 = getEncoderCount(1); // Axis X (vertikal)

    if (up || down || left || right) {
        stopAllMotorTargets();
        Serial.printf("[GRIP] %s%s%s%s spd=%d enc0=%ld enc1=%ld limX=%d limY=%d\n",
                      up ? "UP " : "", down ? "DN " : "",
                      left ? "L " : "", right ? "R " : "",
                      speed, enc0, enc1, limitX, limitY);
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
