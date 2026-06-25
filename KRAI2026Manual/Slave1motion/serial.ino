/*
 * =====================================================================
 * FILE    : serial.ino
 * PERAN   : Inisialisasi UART1 + UART2, relay WSN-31 ke master.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#include "serial.h"

// =====================================================================
//  SETUP
// =====================================================================

void setupSerial() {
    // WSN-31 SET HIGH = mode normal
    pinMode(WSN_SET_PIN, OUTPUT);
    digitalWrite(WSN_SET_PIN, HIGH);

    // UART ke WSN-31 (receiver)
    Serial2.begin(SERIAL_WSN_BAUD, SERIAL_8N1, SERIAL_WSN_RX, SERIAL_WSN_TX);
    // UART ke Master (forwarder)
    Serial1.begin(SERIAL_MASTER_BAUD, SERIAL_8N1, SERIAL_MASTER_RX, SERIAL_MASTER_TX);

    Serial.printf("[WSN-31] Relay init — RX=%d TX=%d → Master TX=%d RX=%d\n",
                  SERIAL_WSN_RX, SERIAL_WSN_TX, SERIAL_MASTER_TX, SERIAL_MASTER_RX);
}

// =====================================================================
//  RELAY — forwarding WSN-31 → Master
// =====================================================================

void serialRelayTick() {
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        Serial1.write(b);
    }
}
