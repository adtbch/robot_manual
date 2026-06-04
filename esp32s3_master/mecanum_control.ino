/*
 * =====================================================================
 * FILE    : mecanum_control.ino
 * PERAN   : Map ControlPacket (dari ESP-NOW / motion_serial) ke
 *           perintah mecanum kinematik, kirim ke Slave via motion_serial.
 *
 * MAPPING:
 *   pkt.x  → Vy  (geser kiri/kanan lateral)   sudah -1023..1023
 *   pkt.y  → Vx  (maju/mundur)                sudah -1023..1023, Y diinvert di controller
 *   pkt.w  → W   (rotasi / sudut putar)       sudah -1023..1023
 *
 * SPEED MODE (via tombol):
 *   Default        → skala penuh dari controller
 *   R1 hold (tahan) → cepat (tidak scale ulang — sudah dari controller)
 *   L1 hold (tahan) → lambat (50% dari nilai pkt)
 *
 * PROTOKOL KE SLAVE:
 *   "Vx Vy W\n"    (tanpa durasi — slave auto-stop 2 detik)
 *
 * SAFETY:
 *   - Deadzone kecil 20 (dari -1023..1023) untuk noise tengah stik
 *   - Update minimal tiap 100ms agar slave tidak timeout (2 detik)
 * =====================================================================
 */

#include "robot_config.h"

#define SLAVE_SERIAL Serial1

static const int16_t DEADZONE_RAW    = 10;   // deadzone untuk raw int8_t -128..127
static const unsigned long SEND_INTERVAL_MS = 50;

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

static int8_t applyDeadzoneRaw(int8_t val) {
    if (abs(val) < DEADZONE_RAW) return 0;
    return val;
}

static int16_t getSpeedMode(uint32_t buttons) {
    if (buttons & BTN_R1) return SPEED_MODE_FAST;
    if (buttons & BTN_L1) return SPEED_MODE_SLOW;
    return SPEED_MODE_DEFAULT;
}

static int16_t scaleSpeed(int8_t val, int16_t maxSpeed) {
    return map((int16_t)val, -128, 127, -maxSpeed, maxSpeed);
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
    
    // Ambil dari pkt.lx/ly/rx (raw int8_t -128..127 dari controller), olah di sini
    int8_t rawVx = applyDeadzoneRaw(pkt.ly);    // maju/mundur (ly: push maju = negatif)
    int8_t rawVy = applyDeadzoneRaw(pkt.lx);    // geser kiri/kanan
    int8_t rawW  = applyDeadzoneRaw(pkt.rx);    // rotasi

    int16_t vx = scaleSpeed(-rawVx, maxSpeed);  // invert Y: push maju → vx positif
    int16_t vy = scaleSpeed(rawVy,  maxSpeed);
    int16_t w  = scaleSpeed(rawW,   maxSpeed);
    
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
