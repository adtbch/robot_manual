/*
 * =====================================================================
 * FILE    : i2c_bus.ino
 * PERAN   : Implementasi mutex bus I2C untuk MPU6050 + OLED.
 * =====================================================================
 */

#include "i2c_bus.h"

namespace I2cBus {

namespace {

Owner currentOwner = Owner::NONE;
constexpr uint32_t kSettleUs = 80;

} // anonymous namespace

bool acquire(Owner owner) {
    if (currentOwner != Owner::NONE && currentOwner != owner) {
        return false;
    }
    currentOwner = owner;
    return true;
}

void release(Owner owner) {
    if (currentOwner != owner) {
        return;
    }
    delayMicroseconds(kSettleUs);
    currentOwner = Owner::NONE;
}

bool isBusy() {
    return currentOwner != Owner::NONE;
}

} // namespace I2cBus
