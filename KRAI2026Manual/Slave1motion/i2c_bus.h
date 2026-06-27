/*
 * =====================================================================
 * FILE    : i2c_bus.h
 * PERAN   : Koordinator akses bus I2C (MPU6050 + OLED share Wire).
 *           Satu owner per waktu — hindari transaksi berurutan yang NACK.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <Arduino.h>

namespace I2cBus {

enum class Owner : uint8_t {
    NONE = 0,
    MPU,
    OLED,
};

bool acquire(Owner owner);
void release(Owner owner);
bool isBusy();

} // namespace I2cBus

#endif // I2C_BUS_H
