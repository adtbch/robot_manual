/*
 * =====================================================================
 * FILE    : webserver.ino
 * PERAN   : Main entry point — setup() + loop().
 *
 * BOARD   : ESP32 (Web Server — dedicated, bukan robot)
 *
 * FLOW:
 *   setup():
 *     1. Serial debug
 *     2. WiFi AP mode
 *     3. ESP-NOW init (kirim config ke master)
 *     4. HTTP server init (serve index.html + handle API)
 *
 *   loop():
 *     1. webServerTick() — handle HTTP requests
 *
 * ARCHITECTURE:
 *   ┌─────────┐   WiFi   ┌─────────────┐   ESP-NOW   ┌─────────┐
 *   │ Device  │ ────────→│ Web Server  │ ───────────→│ Master  │
 *   │ (phone) │          │ (ESP32)     │             │ (S3)    │
 *   └─────────┘          └─────────────┘             └─────────┘
 *
 * CATATAN:
 *   - File ini harus pertama secara alfabetic (webserver.ino < web.h)
 *   - Semua modul .ino dikompilasi bersama-sama
 *   - index.h berisi HTML PROGMEM, di-include oleh web.ino
 * =====================================================================
 */

#include "config.h"
#include "serial.h"
#include "espnow.h"
#include "web.h"

// =====================================================================
//  SETUP
// =====================================================================

void setup() {
    // 1. Serial debug
    setupSerial();
    Serial.println("[BOOT] Starting...");

    // 2. ESP-NOW init (kirim config ke master)
    Serial.println("[BOOT] Inisialisasi ESP-NOW...");
    if (!espNowConfigInit()) {
        Serial.println("[BOOT] ESP-NOW init GAGAL — continuing without ESP-NOW");
    }

    // 3. WiFi AP mode + HTTP server
    Serial.println("[BOOT] Inisialisasi Web Server...");
    webServerInit();

    Serial.println("[BOOT] Siap! Hubungkan ke WiFi: " + String(WIFI_AP_SSID));
    Serial.println("[BOOT] Buka http://192.168.4.1 di browser");
    Serial.println();
}

// =====================================================================
//  LOOP
// =====================================================================

void loop() {
    webServerTick();
}
