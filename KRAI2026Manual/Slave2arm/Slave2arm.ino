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
#include "webconfig.h"
#include "forest_config.h"

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

    // Homing — blocking, sementara master masih delay
    motorYHoming();
    Serial.println("Motor Y homed");
    motorXHoming();
    Serial.println("Motor X homed");

    // Init serial (UART1)
    setupSerial();
    Serial.println("UART1: READY");

    // Init serial command handler
    setupSerialCommand();

    // Init pneumatic
    setupPneumatic();
    Serial.println("Pneumatic: READY");

    // WiFi AP + HTTP test panel
    setupWebServer();
}

// =====================================================================
//  LOOP
// =====================================================================

void loop() {
    // UART TX ke master — harus dari core 1 sebelum baca respons
    masterUartProxyTick();

    // Serial command handler
    serialCommandTick();

    // Motor X/K — continuous run with limit switch
    motorRunTick();
    motorYLimitTick();
    motorYPositionTick();
}
