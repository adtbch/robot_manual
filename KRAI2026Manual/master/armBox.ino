/*
 * =====================================================================
 * FILE    : armBox.ino
 * PERAN   : Arm box motor control — forward/backward toggle + speed.
 *
 * STATE & STATE MACHINE ada di otomat.ino
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "motor.h"
#include "serial.h"
#include "limit_switch.h"

// =====================================================================
//  CONFIG
// =====================================================================

namespace {

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
//  CONTROL
// =====================================================================

void armBoxFBToggle(char side) {
    if (side == 'r') {
        const int8_t sign = pickMotorRunSign(slave2LimitDepan(), slave2LimitBelakang(), gMotorXRunSign);
        sendSlave2Command("motor x %ld", (long)sign * 255);
    } else if (side == 'l') {
        const int8_t sign = pickMotorRunSign(
            readLimitSwitch(LIMIT_ARMBOX_DEPAN),
            readLimitSwitch(LIMIT_ARMBOX_BELAKANG),
            gMotorKRunSign);
        sendSlave2Command("motor k %ld", (long)sign * 255);
    }
}

void armBoxFBbySpeed(char side, int8_t speed) {
    if (side == 'r') {
        const int8_t sign = pickMotorRunSign(slave2LimitDepan(), slave2LimitBelakang(), gMotorXRunSign);
        sendSlave2Command("motor x %ld", (long)sign * speed);
    } else if (side == 'l') {
        const int8_t sign = pickMotorRunSign(
            readLimitSwitch(LIMIT_ARMBOX_DEPAN),
            readLimitSwitch(LIMIT_ARMBOX_BELAKANG),
            gMotorKRunSign);
        sendSlave2Command("motor k %ld", (long)sign * speed);
    }
}
