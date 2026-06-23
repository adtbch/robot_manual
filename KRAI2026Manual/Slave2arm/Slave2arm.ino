/*
 * =====================================================================
 * FILE    : Slave2arm.ino
 * BOARD   : ESP32-S3 (Slave2 Arm Board KRAI 2026)
 * PERAN   : Entry point — setup() + loop().
 *           4 motor arm manipulator, encoder, limit switch, PID.
 * =====================================================================
 */

#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "limit_switch.h"
#include "proximity.h"
#include "serial.h"
#include "pneumatic.h"

// =====================================================================
//  SETUP
// =====================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("=== KRAI 2026 Slave2 Arm Board ===");

    // Init motors
    SetupMotors();
    Serial.println("Motors: READY");

    // Init encoders
    setupEncoders();
    Serial.println("Encoders: READY");

    // Init limit switches
    setupLimits();
    Serial.println("Limit switches: READY");

    // Init proximity
    setupProximity();
    Serial.println("Proximity: READY");

    // Init serial (UART1)
    setupSerial();
    Serial.println("Serial: READY");

    // Init pneumatic
    setupPneumatic();
    Serial.println("Pneumatic: READY");
}

// =====================================================================
//  LOOP
// =====================================================================

void loop() {
    // TODO: tambah modul (PID, serial)
}
