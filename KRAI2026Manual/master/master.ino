/*
 * =====================================================================
 * FILE    : master.ino
 * BOARD   : ESP32-S3 (Master Board KRAI 2026)
 * PERAN   : Entry point — setup() + loop().
 *
 * STEP 1: ESP-NOW receiver test
 *         - Terima ControlPacket dari s3controllerespnow
 *         - Cetak statistik ke Serial Monitor
 *         - Decode & tampilkan paket
 *
 * ARSITEKTUR:
 *   [PS4 DualShock 4] --USB OTG--> [s3controllerespnow]
 *         | ESP-NOW
 *         v
 *   [ESP32-S3 Master — SKETCH INI]
 *         | espNowControlReadPacket()
 *         v
 *         Serial Monitor (115200 baud)
 * =====================================================================
 */

#include "config.h"
#include "espnow.h"
#include "motor.h"
#include "encoder.h"
#include "limit_switch.h"
#include "servo.h"
#include "relay.h"
#include "proximity.h"
#include "serial.h"

// =====================================================================
//  SETUP
// =====================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("=== KRAI 2026 Master Board ===");
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());

    // Init ESP-NOW receiver
    bool espNowReady = espNowControlInit();
    Serial.printf("ESP-NOW control: %s\n", espNowReady ? "READY" : "ERROR");

    // Init motors
    SetupMotors();
    Serial.println("Motors: READY");

    // Init encoders
    setupEncoders();
    Serial.println("Encoders: READY");

    // Init limit switches
    setupLimits();
    Serial.println("Limit switches: READY");

    // Init servos
    setupServos();
    Serial.println("Servos: READY");

    // Init relay
    setupRelay();
    Serial.println("Relay: READY");

    // Init proximity
    setupProximity();
    Serial.println("Proximity: READY");

    // Init serial (UART1 + UART2)
    setupSerial();
    Serial.println("Serial: READY");

    // Init serial command handler
    setupSerialCommand();
}

// =====================================================================
//  LOOP
// =====================================================================

void loop() {
    // Serial command handler
    serialCommandTick();

    // ESP-NOW receiver
    // 1. Cetak statistik periodik
    espNowControlTick();
    // 2. Ambil paket terbaru & cetak
    ControlPacket gLastRxPacket = {};
    if (espNowControlReadPacket(gLastRxPacket)) {
        espNowControlPrintPacket(gLastRxPacket);
    }
}
