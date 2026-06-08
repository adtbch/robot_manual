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
 *   D-pad Down  → Motor 0 (MOTOR_W) 0°
 *   D-pad Left  → Motor 0 (MOTOR_W) 90°
 *   D-pad Up    → Motor 0 (MOTOR_W) 180°
 *   D-pad Right → Motor 0 (MOTOR_W) 270°
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

#define SERVO_STEP_DEFAULT    5
#define SERVO_STEP_FAST       10
#define SERVO_STEP_SLOW       2
#define SERVO_MIN             90
#define SERVO_MAX             180

// Konversi derajat → encoder count (sesuaikan dengan hardware)
// Contoh: 900 counts = 360° → 1 count = 0.4°
#define DEG_TO_COUNT(deg) ((long)((deg) * 900L / 360L))

// Preset derajat motor 0 (MOTOR_W): D-pad = 1 tombol = 1 derajat
#define W_DEG_DOWN    0
#define W_DEG_LEFT    180
#define W_DEG_UP      360
#define W_DEG_RIGHT   540

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

    bool xHeld   = (pkt.buttons & BTN_CROSS)   != 0;
    bool sqHeld  = (pkt.buttons & BTN_SQUARE)  != 0;
    bool up    = (pkt.buttons & BTN_UP)    != 0;
    bool down  = (pkt.buttons & BTN_DOWN)  != 0;
    bool left  = (pkt.buttons & BTN_LEFT)  != 0;
    bool right = (pkt.buttons & BTN_RIGHT) != 0;

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
    // SERVO 0 — manual step via R2 + D-pad Up/Down
    // ================================================================
    bool r2Held = (pkt.buttons & BTN_R2) != 0;

    if (r2Held) {
        static unsigned long lastServoMs = 0;
        static int servoAngle = 0;
        unsigned long nowMs = millis();

        int step = SERVO_STEP_DEFAULT;
        if (pkt.buttons & BTN_R1) {
            step = SERVO_STEP_FAST;
        } else if (pkt.buttons & BTN_L1) {
            step = SERVO_STEP_SLOW;
        }

        if (nowMs - lastServoMs >= 50) {
            lastServoMs = nowMs;
            if (up) {
                servoAngle = min(servoAngle + step, SERVO_MAX);
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "servo0 %d", servoAngle);
                armbox_send(cmd);
            } else if (down) {
                servoAngle = max(servoAngle - step, SERVO_MIN);
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "servo0 %d", servoAngle);
                armbox_send(cmd);
            }
        }
        return;  // R2 aktif → tidak proses motor
    }

    // ================================================================
    // MOTOR 1 (MOTOR_Z) — manual jog via X + D-pad Up/Down
    // ================================================================
    static bool lastUp = false, lastDown = false;

    if (xHeld) {
        int speed = ARMBOX_SPEED_DEFAULT;
        if (pkt.buttons & BTN_R1) {
            speed = ARMBOX_SPEED_FAST;
        } else if (pkt.buttons & BTN_L1) {
            speed = ARMBOX_SPEED_SLOW;
        }

        if (up) {
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "pwmz %d", speed);
            armbox_send(cmd);
        } else if (down) {
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "pwmz %d", -speed);
            armbox_send(cmd);
        } else if (lastUp || lastDown) {
            armbox_send("pwmz 0");
        }

        lastUp = up;
        lastDown = down;
        return;  // X aktif → tidak proses motor 0
    }

    lastUp = false;
    lastDown = false;

    // ================================================================
    // MOTOR Y — manual jog via Square + D-pad Up/Down
    // ================================================================
    static bool lastUpY = false, lastDownY = false;

    if (sqHeld) {
        int speed = ARMBOX_SPEED_DEFAULT;
        if (pkt.buttons & BTN_R1) {
            speed = ARMBOX_SPEED_FAST;
        } else if (pkt.buttons & BTN_L1) {
            speed = ARMBOX_SPEED_SLOW;
        }

        if (up) {
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "pwmy %d", speed);
            armbox_send(cmd);
        } else if (down) {
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "pwmy %d", -speed);
            armbox_send(cmd);
        } else if (lastUpY || lastDownY) {
            armbox_send("pwmy 0");
        }

        lastUpY = up;
        lastDownY = down;
        return;  // Square aktif → tidak proses motor 0
    }

    lastUpY = false;
    lastDownY = false;

    // ================================================================
    // MOTOR 0 (MOTOR_W) — preset via D-pad (tanpa R2/X/Square)
    // ================================================================
    static bool lastUpW = false, lastDownW = false, lastLeftW = false, lastRightW = false;

    if (up && !lastUpW) {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "motorw %d", W_DEG_UP);
        armbox_send(cmd);
    } else if (down && !lastDownW) {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "motorw %d", W_DEG_DOWN);
        armbox_send(cmd);
    } else if (left && !lastLeftW) {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "motorw %d", W_DEG_LEFT);
        armbox_send(cmd);
    } else if (right && !lastRightW) {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "motorw %d", W_DEG_RIGHT);
        armbox_send(cmd);
    }

    lastUpW = up;
    lastDownW = down;
    lastLeftW = left;
    lastRightW = right;
}
