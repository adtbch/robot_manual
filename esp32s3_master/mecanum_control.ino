/*
 * =====================================================================
 * FILE    : mecanum_control.ino
 * PERAN   : Map ControlPacket (dari ESP-NOW / motion_serial) ke
 *           perintah mecanum kinematik, kirim ke Slave via motion_serial.
 *
 * MAPPING:
 *   Left stick Y  (ly) → Vx  (maju/mundur)
 *   Left stick X  (lx) → Vy  (geser kiri/kanan)
 *   Right stick X (rx) → W   (rotasi)
 *
 * SPEED MODE:
 *   Default        → 100
 *   R1 hold (tahan) → 300 (cepat)
 *   L1 hold (tahan) → 50  (lambat)
 *
 * PROTOKOL KE SLAVE:
 *   "Vx Vy W\n"    (tanpa durasi — slave auto-stop 2 detik)
 *
 * SAFETY:
 *   - Deadzone: analog |val| < 15 dianggap 0
 *   - Update minimal tiap 100ms agar slave tidak timeout (2 detik)
 * =====================================================================
 */

#include "robot_config.h"

#define SLAVE_SERIAL Serial1

static const int8_t  DEADZONE      = 15;
static const int16_t ANALOG_MAX    = 127;
static const unsigned long SEND_INTERVAL_MS = 100;

// =====================================================================
//  STATE
// =====================================================================

static int16_t gLastVx = 0;
static int16_t gLastVy = 0;
static int16_t gLastW  = 0;
static unsigned long gLastSendMs = 0;

// =====================================================================
//  HELPER
// =====================================================================

static int8_t applyDeadzone(int8_t val) {
    if (abs(val) < DEADZONE) return 0;
    return val;
}

static int16_t getSpeedMode(uint32_t buttons) {
    if (buttons & BTN_R1) return SPEED_MODE_FAST;
    if (buttons & BTN_L1) return SPEED_MODE_SLOW;
    return SPEED_MODE_DEFAULT;
}

static int16_t scaleSpeed(int8_t val, int16_t maxSpeed) {
    return map(val, -ANALOG_MAX, ANALOG_MAX, -maxSpeed, maxSpeed);
}

// =====================================================================
//  KIRIM PERINTAH KE SLAVE
// =====================================================================

static void sendMecanumCommand(int16_t vx, int16_t vy, int16_t w) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "%d %d %d\n", vx, vy, w);
    SLAVE_SERIAL.print(cmd);
}

// =====================================================================
//  TICK — dipanggil setiap loop()
// =====================================================================

void mecanum_control_tick(const ControlPacket &pkt) {
    if (!pkt.connected) {
        if (gLastVx != 0 || gLastVy != 0 || gLastW != 0) {
            gLastVx = gLastVy = gLastW = 0;
            sendMecanumCommand(0, 0, 0);
            gLastSendMs = millis();
        }
        return;
    }

    int16_t maxSpeed = getSpeedMode(pkt.buttons);

    int8_t lx = applyDeadzone(pkt.lx);
    int8_t ly = applyDeadzone(pkt.ly);
    int8_t rx = applyDeadzone(pkt.rx);

    int16_t vx = scaleSpeed(-ly, maxSpeed);
    int16_t vy = scaleSpeed(lx, maxSpeed);
    int16_t w  = scaleSpeed(rx, maxSpeed);

    bool changed = (vx != gLastVx || vy != gLastVy || w != gLastW);
    unsigned long nowMs = millis();

    if (changed || (nowMs - gLastSendMs >= SEND_INTERVAL_MS)) {
        gLastVx = vx;
        gLastVy = vy;
        gLastW  = w;
        gLastSendMs = nowMs;
        sendMecanumCommand(vx, vy, w);
    }
}
