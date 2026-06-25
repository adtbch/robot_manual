/*
 * =====================================================================
 * FILE    : Slave1motion.ino
 * BOARD   : ESP32-S3 (Slave1 Motion Board KRAI 2026)
 * PERAN   : Entry point — setup() + loop().
 *           Mecanum drive 4 motor, MPU6050 yaw, PID, serial relay.
 * =====================================================================
 */

#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "mpu.h"
#include "kinematik.h"
#include "pid.h"
#include "serial.h"
#include <Wire.h>

// =====================================================================
//  SETUP
// =====================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("=== KRAI 2026 Slave1 Motion Board ===");

    // Init serial relay
    setupSerial();
    Serial.println("Serial: READY");

    // Init I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setTimeOut(100);
    Wire.setClock(400000);

    // Init motors
    SetupMotors();
    Serial.println("Motors: READY");

    // Init PID
    pidControllerInit();
    initYawPid();
    Serial.println("PID: READY");

    // Init encoders
    setupEncoders();
    Serial.println("Encoders: READY");

    // Init MPU
    if (!setupMPU()) {
        Serial.println("MPU: not ready, yaw will stay 0");
    } else {
        Serial.println("MPU: READY");
    }
}

// =====================================================================
//  LOOP
// =====================================================================

void loop() {
    // Relay WSN-31 → Master
    serialRelayTick();

    // Convert encoder ke RPM (40ms interval)
    static Jeda jedaEncoder;
    if (jedaEncoder.check(RPM_INTERVAL_MS)) {
        convertEncoderToRPM();
    }

    // Update yaw
    updateYaw();
}
