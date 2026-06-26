/*
 * =====================================================================
 * FILE    : ota.h
 * PERAN   : Konfigurasi OTA (Over-The-Air) update via WiFi AP Mode.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef OTA_H
#define OTA_H

#include "config.h"

// =====================================================================
//  WIFI CONFIGURATION (AP Mode)
// =====================================================================
static constexpr const char* OTA_AP_SSID = "KRAI_2026_SLAVE1";
static constexpr const char* OTA_AP_PASS = "krai2026motion";  // Min 8 karakter
static constexpr const char* OTA_HOSTNAME = "krai-slave1-motion";

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupOTA();
void handleOTA();
bool isOtaUploading();  // Cek apakah sedang proses transfer firmware

#endif // OTA_H
