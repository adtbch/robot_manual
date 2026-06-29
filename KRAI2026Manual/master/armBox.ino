/*
 * =====================================================================
 * FILE    : armBox.ino
 * PERAN   : Auto arm box — kontrol pneumatic + motor Y/X/K via sensor
 *           proximity dari slave2arm.
 *
 * ARSITEKTUR:
 *   Slave2arm kirim sensor data (prox, limit, enc, pne) ke master.
 *   Master baca state, putuskan aksi, kirim command balik ke slave2.
 *
 *   Master kontrol:
 *     - Motor Y (lokal di master, encoder position)
 *     - Motor X/K (via slave2, continuous run + limit stop)
 *     - Pneumatic R/L (via slave2)
 *
 * FLOW:
 *   IDLE  → prox detect → pne ON → Jeda 300ms → motor commands → GRAB
 *   GRAB  → (tunggu trigger dari luar) → pne OFF → IDLE
 *
 *   armBoxDone('r'/'l') dipanggil dari luar saat box selesai.
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "motor.h"
#include "serial.h"
#include "limit_switch.h"

// =====================================================================
//  STATE
// =====================================================================

enum ArmBoxState { ARMBOX_IDLE, ARMBOX_WAIT, ARMBOX_GRAB, ARMBOX_DONE, ARMBOX_BACK };
constexpr long MOTOR_SPEED = 255;

namespace {
ArmBoxState gArmBoxR = ARMBOX_IDLE;
ArmBoxState gArmBoxL = ARMBOX_IDLE;
Jeda jedaArmBoxR;
Jeda jedaArmBoxL;
int8_t gMotorXRunSign = 1;
int8_t gMotorKRunSign = 1;

int8_t pickMotorRunSign(bool atDepan, bool atBelakang, int8_t &fallbackSign) {
    if (atDepan) {
        fallbackSign = -1;
        return -1;
    }
    if (atBelakang) {
        fallbackSign = +1;
        return +1;
    }
    const int8_t sign = fallbackSign;
    fallbackSign = -fallbackSign;
    return sign;
}

} // anonymous namespace

// =====================================================================
//  TICK — panggil di loop()
//  IDLE → WAIT → GRAB otomatis.
//  GRAB → IDLE via armBoxDone().
// =====================================================================

void armBoxTick() {
    // ── Sisi R ─────────────────────────────────────────────────
    switch (gArmBoxR) {
        case ARMBOX_IDLE:
            if (slave2ProxR()) {
                sendSlave2Command("pne r on");
                jedaArmBoxR.reset();
                gArmBoxR = ARMBOX_WAIT;
            }
            break;

        case ARMBOX_WAIT:
            if (!jedaArmBoxR.check(300)) break;
            sendSlave2Command("motortarget %ld", MOTOR_Y_LEVEL_4);
            sendSlave2Command("motor x -255");
            gArmBoxR = ARMBOX_GRAB;
            break;

        case ARMBOX_GRAB:
            break;
    }

    // ── Sisi L ─────────────────────────────────────────────────
    switch (gArmBoxL) {
        case ARMBOX_IDLE:
            if (slave2ProxL()) {
                sendSlave2Command("pne l on");
                jedaArmBoxL.reset();
                gArmBoxL = ARMBOX_WAIT;
            }
            break;

        case ARMBOX_WAIT:
            if (!jedaArmBoxL.check(300)) break;
            motorYSetTarget(MOTOR_Y_LEVEL_4);
            sendSlave2Command("motor k -255");
            gArmBoxL = ARMBOX_GRAB;
            break;

        case ARMBOX_GRAB:
            break;
    }
}

// =====================================================================
//  DONE — dipanggil dari luar saat box selesai diangkat/dipindah
// =====================================================================

void armBoxDone(char side) {
    if (side == 'r') {
        if (gArmBoxR == ARMBOX_GRAB) {
            sendSlave2Command("motortarget %ld", MOTOR_Y_LEVEL_5);
            gArmBoxR = ARMBOX_DONE;
            return;
        } else if (gArmBoxR == ARMBOX_DONE) {
            sendSlave2Command("pne r off");
            gArmBoxR = ARMBOX_BACK;
            return;
        }else if (gArmBoxR == ARMBOX_BACK) {
            gArmBoxR = ARMBOX_IDLE;
        }
    } else if (side == 'l') {
        if (gArmBoxL == ARMBOX_GRAB) {
            motorYSetTarget(MOTOR_Y_LEVEL_5);
            gArmBoxL = ARMBOX_DONE;
            return;
        } else if (gArmBoxL == ARMBOX_DONE) {
            sendSlave2Command("pne l off");
            gArmBoxL = ARMBOX_BACK;
            return;
        } else if (gArmBoxL == ARMBOX_BACK) {
            gArmBoxL = ARMBOX_IDLE;
        }
    }
}

// =====================================================================
//  CONTROL
// =====================================================================

void armBoxReset() {
    sendSlave2Command("pneall");
    gArmBoxR = ARMBOX_IDLE;
    gArmBoxL = ARMBOX_IDLE;
    gMotorXRunSign = 1;
    gMotorKRunSign = 1;
}

void armBoxFBToggle(char side) {
    if (side == 'r') {
        const int8_t sign = pickMotorRunSign(slave2LimitDepan(), slave2LimitBelakang(), gMotorXRunSign);
        sendSlave2Command("motor x %ld", (long)sign * MOTOR_SPEED);
    } else if (side == 'l') {
        const int8_t sign = pickMotorRunSign(
            readLimitSwitch(LIMIT_ARMBOX_DEPAN),
            readLimitSwitch(LIMIT_ARMBOX_BELAKANG),
            gMotorKRunSign);
        sendSlave2Command("motor k %ld", (long)sign * MOTOR_SPEED);
    }
}