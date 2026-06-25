/*
 * =====================================================================
 * FILE    : serial.ino
 * PERAN   : Serial debug — setup + helper untuk print.
 *
 * BOARD   : ESP32 (Web Server)
 * =====================================================================
 */

#include "serial.h"

void setupSerial() {
    Serial.begin(SERIAL_DEBUG_BAUD);
    delay(100);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  KRAI 2026 — Web Server ESP32");
    Serial.println("========================================");
}
