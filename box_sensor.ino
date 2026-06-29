/*
 * =====================================================================
 * FILE    : box_sensor.ino
 * PERAN   : Auto pneumatic saat proximity detect.
 *           R — langsung ON/OFF (motor di slave2arm)
 *           L — perlu motor Y naik via master
 *
 * STATE PER SISI:
 *   BOX_IDLE    — tunggu proximity detect
 *   BOX_GRAB    — pneumatic ON, kirim command ke master (L only)
 *   BOX_WAIT_UP — tunggu done
 *
 * PROTOCOL:
 *   Slave2arm → Master: "boxl"
 *   Master → Slave2arm: "boxl done"
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#include "proximity.h"
#include "pneumatic.h"
#include "serial.h"

// =====================================================================
//  STATE — R dan L independent
// =====================================================================

enum BoxState { BOX_IDLE, BOX_GRAB, BOX_WAIT_UP, BOX_DONE };

namespace {
BoxState gBoxR = BOX_IDLE;
BoxState gBoxL = BOX_IDLE;
Jeda JedaBoxR;  // untuk delay non-blocking (jika perlu)
Jeda JedaBoxL;  // untuk delay non-blocking (jika perlu)
} // anonymous namespace

// =====================================================================
//  TICK — panggil di loop()
// =====================================================================

void boxSensorTick() {
    // ── Sisi R — langsung ON, tunggu prox hilang → OFF ──────────
    switch (gBoxR) {
        case BOX_IDLE:
            if (readProximity('r')) {
                pneumaticOn('r');
                JedaBoxR.reset();
                gBoxR = BOX_WAIT_UP;
            }
            break;
        case BOX_WAIT_UP:
            if(!JedaBoxR.check(300)) break;
            motorYSetTarget(500);  // naik ke level 1
            motorRunStart('x', -255);  // contoh nilai pwm    
            gBoxR = BOX_GRAB;
            break;
        case BOX_GRAB:
            break;
    }

    // ── Sisi L — pneumatic ON, kirim ke master, tunggu done ─────
    switch (gBoxL) {
        case BOX_IDLE:
            if (readProximity('l')) {
                pneumaticOn('l');
                JedaBoxL.reset();
                gBoxL = BOX_WAIT_UP;
            }
            break;
        case BOX_WAIT_UP:
            if(!JedaBoxL.check(300)) break;
            sendmotorYSetLevelCommand(4);
            motorRunStart('K', -255);  // contoh nilai pwm
            gBoxL = BOX_GRAB;
            break;
        case BOX_GRAB:
            break;
    }
}

// =====================================================================
//  CALLBACK — dipanggil dari serial.ino saat terima "boxl done"
// =====================================================================

void boxSensorDone(char side) {
    if (side == 'l') {
        if(gBoxL == BOX_GRAB) {
            pneumaticOff('l');
            gBoxL = BOX_DONE;
        } else if (gBoxL == BOX_DONE) {
            gBoxL = BOX_IDLE;
        }
    } else if (side == 'r') {
        if(gBoxR == BOX_GRAB) {
            pneumaticOff('r');
            gBoxR = BOX_DONE;
        } else if (gBoxR == BOX_DONE) {
            gBoxR = BOX_IDLE;
        }
    }
}

// =====================================================================
//  CONTROL
// =====================================================================

void boxSensorReset() {
    pneumaticAllOff();
    gBoxR = BOX_IDLE;
    gBoxL = BOX_IDLE;
}
