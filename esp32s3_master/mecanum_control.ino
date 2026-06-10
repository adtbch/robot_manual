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
 * L2 + ANALOG KANAN → ROTASI ABSOLUT:
 *   L2 + Stick Atas  → 0°
 *   L2 + Stick Kanan → 90°
 *   L2 + Stick Bawah → 180°
 *   L2 + Stick Kiri  → -90°
 *
 * PROTOKOL KE SLAVE:
 *   "Vx Vy W\n"          (tanpa durasi — slave auto-stop 2 detik)
 *   "ROTATE <yaw>\n"     (rotasi absolut ke sudut target)
 *
 * SAFETY:
 *   - Deadzone kecil 20 (dari -1023..1023) untuk noise tengah stik
 *   - Update minimal tiap 100ms agar slave tidak timeout (2 detik)
 * =====================================================================
 */

#include "robot_config.h"

#define SLAVE_SERIAL Serial1

InputMode currentInputMode = INPUT_DPAD;

static const int16_t DEADZONE_RAW    = 10;
static const int16_t DEADZONE_STICK  = 50;
static const unsigned long SEND_INTERVAL_MS = 2;

// =====================================================================
//  STATE
// =====================================================================

static int16_t gLastVx = 0;
static int16_t gLastVy = 0;
static int16_t gLastW  = 0;
static unsigned long gLastSendMs = 0;
static bool gL2RotationActive = false;

// --- Yaw angle saved in RAM (untuk L2 + manual increment) ---
static int16_t gSavedYaw = 0;
static bool gL2WasHeld = false;

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
//  ACTION INPUT — D-pad atau analog kiri tergantung inputMode
// =====================================================================

ActionInput getActionInput(const ControlPacket &pkt) {
    ActionInput ai = {};
    if (currentInputMode == INPUT_ANALOG) {
        int8_t lx = pkt.lx;
        int8_t ly = pkt.ly;
        ai.up    = (ly > 30);
        ai.down  = (ly < -30);
        ai.left  = (lx < -30);
        ai.right = (lx > 30);
        if (ai.up && ai.down)   { ai.up = false; ai.down = false; }
        if (ai.left && ai.right) { ai.left = false; ai.right = false; }
    } else {
        ai.up    = (pkt.buttons & BTN_UP)    != 0;
        ai.down  = (pkt.buttons & BTN_DOWN)  != 0;
        ai.left  = (pkt.buttons & BTN_LEFT)  != 0;
        ai.right = (pkt.buttons & BTN_RIGHT) != 0;
    }
    ai.r2     = (pkt.buttons & BTN_R2)     != 0;
    ai.x      = (pkt.buttons & BTN_CROSS)  != 0;
    ai.square = (pkt.buttons & BTN_SQUARE) != 0;
    ai.r1     = (pkt.buttons & BTN_R1)     != 0;
    ai.l1     = (pkt.buttons & BTN_L1)     != 0;
    return ai;
}

// =====================================================================
//  L2 + ANALOG KANAN → ROTASI ABSOLUT
// =====================================================================

static void sendRotateCommand(int yawTarget) {
    // Pakai format "0 0 <yaw> <duration>" yang sudah dipahami Slave1
    // Slave1 akan panggil driveFieldCentricWithYawCorrection(0, 0, yawTarget)
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "0 0 %d\n", yawTarget);
    SLAVE_SERIAL.print(cmd);
}

static int mapStickToYawTarget(int8_t rx, int8_t ry) {
    // Stick atasan → 0°, kanan → 90°, bawah → 180°, kiri → -90°
    // Menggunakan atan2 untuk menentukan sudut dari stick
    float angle = atan2f((float)rx, (float)(-ry)) * (180.0f / PI);

    // Snap ke 4 arah terdekat
    if (angle >= -45.0f && angle < 45.0f)    return 0;    // atas
    if (angle >= 45.0f && angle < 135.0f)    return 90;   // kanan
    if (angle >= -135.0f && angle < -45.0f)  return -90;  // kiri
    return 180;  // bawah (angle >= 135 atau < -135)
}

// =====================================================================
//  KIRIM PERINTAH KE SLAVE
// =====================================================================

static void sendMecanumCommand(int16_t vx, int16_t vy, int16_t yawTarget) {
    // Kirim sebagai "0 0 <yaw> <duration>" agar Slave1 pakai
    // driveFieldCentricWithYawCorrection (PID yaw lock)
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "%d %d %d 2000\n", vx, vy, yawTarget);
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
        }
        gL2RotationActive = false;
        return;
    }

    // ============================================================
    // SHARE TOGGLE — edge detection
    // ============================================================
    static bool lastShare = false;
    bool shareNow = (pkt.buttons & BTN_SHARE) != 0;
    if (shareNow && !lastShare) {
        currentInputMode = (currentInputMode == INPUT_DPAD) ? INPUT_ANALOG : INPUT_DPAD;
    }
    lastShare = shareNow;

    // ============================================================
    // TOUCHPAD → snap_yaw ke Slave1
    // ============================================================
    static bool lastTouchpad = false;
    bool touchpadNow = (pkt.buttons & BTN_TOUCHPAD) != 0;
    if (touchpadNow && !lastTouchpad) {
        SLAVE_SERIAL.print("snap_yaw\n");
    }
    lastTouchpad = touchpadNow;

    bool l2Held = (pkt.buttons & BTN_L2) != 0;

    // ============================================================
    // MOVEMENT INPUT — routing berdasarkan inputMode
    // ============================================================
    int16_t maxSpeed = getSpeedMode(pkt.buttons);
    int8_t rawVx, rawVy, rawW;

    if (currentInputMode == INPUT_ANALOG) {
        // ANALOG mode: D-pad → movement
        int8_t dpadVx = 0, dpadVy = 0;
        if (pkt.buttons & BTN_UP)    dpadVx =  127;
        if (pkt.buttons & BTN_DOWN)  dpadVx = -127;
        if (pkt.buttons & BTN_LEFT)  dpadVy = -127;
        if (pkt.buttons & BTN_RIGHT) dpadVy =  127;
        if (dpadVx != 0 && dpadVy != 0) { dpadVx = 0; dpadVy = 0; }
        rawVx = dpadVx;
        rawVy = dpadVy;
    } else {
        // DPAD mode: analog kiri → movement
        rawVx = applyDeadzoneRaw(pkt.ly);
        rawVy = applyDeadzoneRaw(pkt.lx);
    }

    rawW = applyDeadzoneRaw(pkt.rx);  // selalu dari analog kanan

    int16_t vx = scaleSpeed(rawVx, maxSpeed);
    int16_t vy = scaleSpeed(rawVy, maxSpeed);

    // Increment yaw: stick deflection → speed (1°..5° per tick)
    if (rawW > 0) {
        int inc = map(rawW, DEADZONE_RAW, 127, 1, 5);
        gSavedYaw += inc;
        if (gSavedYaw > 180) gSavedYaw = -179;
    } else if (rawW < 0) {
        int inc = map(-rawW, DEADZONE_RAW, 127, 1, 5);
        gSavedYaw -= inc;
        if (gSavedYaw < -179) gSavedYaw = 180;
    }

    // Kirim Vx, Vy, dan yaw absolut ke Slave1
    unsigned long nowMs = millis();
    bool changed = (vx != gLastVx || vy != gLastVy);
    if (changed || (nowMs - gLastSendMs >= SEND_INTERVAL_MS)) {
        gLastVx = vx;
        gLastVy = vy;
        gLastSendMs = nowMs;
        sendMecanumCommand(-vx, vy, gSavedYaw);
    }
    // ============================================================
    // L2 + ANALOG KANAN → ROTASI ABSOLUT
    // ============================================================
    if (l2Held) {
        int8_t rx = pkt.rx;
        int8_t ry = -pkt.ry;

         // Cek apakah stick cukup defleksi (melewati deadzone)
        if (abs(rx) > DEADZONE_STICK || abs(ry) > DEADZONE_STICK) {
            gSavedYaw = mapStickToYawTarget(rx, ry);
            sendMecanumCommand(vx, vy, gSavedYaw);
            gL2RotationActive = true;
            gLastVx = gLastVy = gLastW = 0;
            gLastSendMs = millis();
            return;
        }
    }
}
