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
#include "odom.h"

// =====================================================================
//  BOX STATE — tracking motor Y untuk kirim "box done" ke slave2
// =====================================================================

namespace {
char gBoxPendingSide = 0;  // 'r', 'l', atau 0 (tidak ada)
} // anonymous namespace

void boxSetPending(char side) {
    gBoxPendingSide = side;
}

void boxCheckDone() {
    if (gBoxPendingSide != 0 && !motorYIsActive()) {
        slave2Serial.printf("box%c done\n", gBoxPendingSide);
        Serial.printf("Box %c: DONE dikirim ke slave2\n", gBoxPendingSide);
        gBoxPendingSide = 0;
    }
}

// =====================================================================
//  SETUP
// =====================================================================

void setup() {
    delay(8000);
    Serial.begin(115200);
    Serial.println("=== KRAI 2026 Master Board ===");

    // Init ESP-NOW receiver
    bool espNowReady = espNowControlInit();
    // Init motors
    SetupMotors();
    // Init encoders
    setupEncoders();
    // Init limit switches
    setupLimits();
    // Init servos
    setupServos();
    // Init relay
    setupRelay();
    // Init proximity
    setupProximity();
    // Init serial (UART1 + UART2)
    setupSerial();
    // Init serial command handler
    setupSerialCommand();

    setHomingAll();
    Serial.println("Setup zone1: limit Y/X + motor Y lv0 + motor X enc0");
}

// =====================================================================
//  LOOP
// =====================================================================

void loop() {
    // Serial command handler
    serialCommandTick();

    // Gripper non-blocking update
    gripperZone1();

    // ESP-NOW receiver
    espNowControlTick();
    static ControlPacket gLastRxPacket = {};
    espNowControlReadPacket(gLastRxPacket);
    gripperControlTick(gLastRxPacket);
    motionControlTick(gLastRxPacket);
    odomRecordTick(gLastRxPacket);

    // Encoder positioning motor X/Y
    motorXPositionTick();
    motorYPositionTick();

    // Box done — cek motor Y sampai, kirim ke slave2
    boxCheckDone();
}
