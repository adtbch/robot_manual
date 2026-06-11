/*
 * =====================================================================
 * FILE    : armbox_control.ino
 * PERAN   : Kontrol Slave2 (Manipulator) via manipulator_serial
 *           saat MODE_ARM_BOX aktif.
 *
 *           ** HANYA AKTIF DI MODE_ARM_BOX **
 *
 * MAPPING:
 *   X + D-pad Up/Down    → Motor 1 (MOTOR_Z) naik/turun (manual PWM)
 *   Square + D-pad Up/Down → Motor Y (Maju Mundur) maju/mundur (manual PWM)
 *   R1 = cepat, L1 = lambat (untuk motor)
 *
 *   D-pad Down  → Motor 0 (MOTOR_W) posisi depan
 *   D-pad Left  → Motor 0 (MOTOR_W) posisi kiri
 *   D-pad Up    → Motor 0 (MOTOR_W) posisi belakang
 *   D-pad Right → Motor 0 (MOTOR_W) posisi kanan
 *
 *   R2 + D-pad Up    → Servo 0 tambah derajat
 *   R2 + D-pad Down  → Servo 0 kurang derajat
 *   R1 = step besar (10°), L1 = step kecil (2°), default (5°)
 *
 *   Triangle → Toggle relay1 (IN4) ON/OFF
 *
 * PROTOCOL KE SLAVE2:
 *   "pwmz <pwm>\n"  — motor 1 direct PWM (-1023..1023)
 *   "motorw <pos>\n" — motor 0 posisi target (encoder count)
 * =====================================================================
 */

#include "robot_config.h"

// ============================================================
// DEFINES
// ============================================================
#define ARMBOX_SPEED_DEFAULT  350
#define ARMBOX_SPEED_FAST     600
#define ARMBOX_SPEED_SLOW     200

#define SERVO_STEP_DEFAULT    40
#define SERVO_STEP_FAST       50
#define SERVO_STEP_SLOW       10
#define SERVO_MIN             70
#define SERVO_MAX             180

// Preset encoder count motor 0 (MOTOR_W) — SESUAIKAN DENGAN HARDWARE
// D-pad Down/Left/Up/Right → posisi motor
#define W_POS_DOWN    0       // posisi depan (default)
#define W_POS_LEFT    1290    // posisi kiri (test: 1300 = nengok kiri)
#define W_POS_UP      2640    // posisi belakang
#define W_POS_RIGHT   3940    // posisi kanan

// =====================================================================
//  HELPER: kirim perintah ke Slave2 via manipulator_serial
// =====================================================================

static void armbox_send(const char* cmd) {
    manipulator_serial.println(cmd);
}

// =====================================================================
//  TICK — dipanggil setiap loop() jika MODE_ARM_BOX
// =====================================================================

void armbox_control_tick(const ControlPacket &pkt) {
    if (currentMode != MODE_ARM_BOX) return;

    ActionInput ai = getActionInput(pkt);

    // ================================================================
    // RELAY 1 — toggle ON/OFF via Triangle
    // Saat OFF: pulse relay0 sebentar untuk buka sulenoid
    // ================================================================
    static bool lastTriangle = false;
    static bool relay1On = false;
    static bool pulsingRelay0 = false;
    static unsigned long pulseStartMs = 0;
    bool triangleNow = (pkt.buttons & BTN_TRIANGLE) != 0;

    // Non-blocking pulse relay0
    if (pulsingRelay0 && (millis() - pulseStartMs >= 100)) {
        armbox_send("relay0 1");  // relay0 OFF
        pulsingRelay0 = false;
    }

    if (triangleNow && !lastTriangle) {
        relay1On = !relay1On;

        // Set relay1 dulu
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "relay1 %d", relay1On ? 0 : 1);  // 0=ON, 1=OFF
        armbox_send(cmd);

        // Kalau baru saja OFF → pulse relay0 sebentar (buka sulenoid)
        if (!relay1On) {
            armbox_send("relay0 0");  // relay0 ON
            pulseStartMs = millis();
            pulsingRelay0 = true;
        }
    }
    lastTriangle = triangleNow;

    // ================================================================
    // RELAY 0 — pulse ON sebentar via Circle
    // ================================================================
    static bool lastCircle = false;
    static bool pulsingRelay0Circle = false;
    static unsigned long pulseCircleStartMs = 0;
    bool circleNow = (pkt.buttons & BTN_CIRCLE) != 0;

    // Non-blocking pulse relay0
    if (pulsingRelay0Circle && (millis() - pulseCircleStartMs >= 100)) {
        armbox_send("relay0 1");  // relay0 OFF
        pulsingRelay0Circle = false;
    }

    if (circleNow && !lastCircle) {
        armbox_send("relay0 0");  // relay0 ON
        pulseCircleStartMs = millis();
        pulsingRelay0Circle = true;
    }
    lastCircle = circleNow;

    // ================================================================
    // SERVO 0 — manual step via R2 + D-pad/analog Up/Down
    // ================================================================
    if (ai.r2) {
        static unsigned long lastServoMs = 0;
        static int servoAngle = 0;
        unsigned long nowMs = millis();

        int step = SERVO_STEP_DEFAULT;
        if (ai.r1) {
            step = SERVO_STEP_FAST;
        } else if (ai.l1) {
            step = SERVO_STEP_SLOW;
        }

        if (nowMs - lastServoMs >= 50) {
            lastServoMs = nowMs;
            if (ai.down) {
                servoAngle = min(servoAngle + step, SERVO_MAX);
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "servo0 %d", servoAngle);
                armbox_send(cmd);
            } else if (ai.up) {
                servoAngle = max(servoAngle - step, SERVO_MIN);
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "servo0 %d", servoAngle);
                armbox_send(cmd);
            }
        }
        return;
    }

    // ================================================================
    // MOTOR 1 (MOTOR_Z) — manual jog via X + D-pad/analog Up/Down
    // ================================================================
    static bool lastUp = false, lastDown = false;

    if (ai.x) {
        int speed = ARMBOX_SPEED_DEFAULT;
        if (ai.r1) {
            speed = ARMBOX_SPEED_FAST;
        } else if (ai.l1) {
            speed = ARMBOX_SPEED_SLOW;
        }

        if (ai.up) {
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "pwmz %d", speed);
            armbox_send(cmd);
        } else if (ai.down) {
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "pwmz %d", -speed);
            armbox_send(cmd);
        } else if (lastUp || lastDown) {
            armbox_send("pwmz 0");
        }

        lastUp = ai.up;
        lastDown = ai.down;
        return;
    }

    lastUp = false;
    lastDown = false;

    // ================================================================
    // MOTOR Y — manual jog via Square + D-pad/analog Up/Down
    // ================================================================
    static bool lastUpY = false, lastDownY = false;

    if (ai.square) {
        int speed = ARMBOX_SPEED_DEFAULT;
        if (ai.r1) {
            speed = ARMBOX_SPEED_FAST;
        } else if (ai.l1) {
            speed = ARMBOX_SPEED_SLOW;
        }

        if (ai.up) {
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "pwmy %d", speed);
            armbox_send(cmd);
        } else if (ai.down) {
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "pwmy %d", -speed);
            armbox_send(cmd);
        } else if (lastUpY || lastDownY) {
            armbox_send("pwmy 0");
        }

        lastUpY = ai.up;
        lastDownY = ai.down;
        return;
    }

    lastUpY = false;
    lastDownY = false;

    // ================================================================
    // MOTOR 0 (MOTOR_W) — preset via D-pad/analog (tanpa R2/X/Square)
    // ================================================================
    static bool lastUpW = false, lastDownW = false, lastLeftW = false, lastRightW = false;

    if (ai.up && !lastUpW) {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "motorw %d", W_POS_UP);
        armbox_send(cmd);
    } else if (ai.down && !lastDownW) {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "motorw %d", W_POS_DOWN);
        armbox_send(cmd);
    } else if (ai.left && !lastLeftW) {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "motorw %d", W_POS_LEFT);
        armbox_send(cmd);
    } else if (ai.right && !lastRightW) {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "motorw %d", W_POS_RIGHT);
        armbox_send(cmd);
    }

    lastUpW = ai.up;
    lastDownW = ai.down;
    lastLeftW = ai.left;
    lastRightW = ai.right;
}
