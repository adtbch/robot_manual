/*
 * =====================================================================
 * FILE    : mpu.h
 * PERAN   : Konfigurasi modul MPU6050 IMU (I2C, interrupt).
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef MPU_H
#define MPU_H

#include "config.h"

// =====================================================================
//  PIN I2C
// =====================================================================
constexpr uint8_t I2C_SDA = 13;
constexpr uint8_t I2C_SCL = 14;

// =====================================================================
//  INTERRUPT PIN
// =====================================================================
constexpr uint8_t MPU_INTERRUPT_PIN = 46;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
bool setupMPU();
void updateYaw();
float getYaw();
void calibrateGyro();
void calibrateGyroHot();
void setYawReference(float targetYaw);
void snapYaw();

#endif // MPU_H
