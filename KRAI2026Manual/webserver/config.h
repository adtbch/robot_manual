/*
 * =====================================================================
 * FILE    : config.h
 * PERAN   : Pusat konfigurasi Web Server ESP32.
 *           Shared types untuk ESP-NOW config protocol, WiFi AP,
 *           HTTP endpoints, dan Jeda timer.
 *
 * BOARD   : ESP32 (Web Server — dedicated, bukan robot)
 *
 * CATATAN:
 *   ESP-NOW config protocol beda dari controller joystick protocol.
 *   Magic = 0xC0DE (config), bukan 0xA5B4 (joystick).
 * =====================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =====================================================================
//  WIFI AP CONFIG
// =====================================================================

constexpr const char* WIFI_AP_SSID = "KRAI2026_Config";
constexpr const char* WIFI_AP_PASS = "krai2026";
constexpr uint8_t     WIFI_AP_CHANNEL = 1;
constexpr uint8_t     WIFI_AP_MAX_CONN = 4;

// =====================================================================
//  ESP-NOW — CONFIG PROTOCOL TO MASTER
//  Beda magic dari joystick (0xA5B4), supaya master bisa bedakan.
// =====================================================================

constexpr uint16_t ESPNOW_CONFIG_MAGIC = 0xC0DE;
constexpr uint8_t  ESPNOW_CONFIG_CHANNEL = 1;

// MAC Address Master ESP32-S3 — WAJIB DIISI
constexpr uint8_t MASTER_MAC[6] = {0x30, 0x76, 0xF5, 0xE5, 0xD8, 0xE4};

// ESP-NOW MTU = 250 bytes
// Header: magic(2) + index(1) + total(1) + type(1) = 5 bytes
// Payload: 245 bytes max
constexpr size_t ESPNOW_PAYLOAD_MAX = 245;
constexpr size_t ESPNOW_HEADER_SIZE = 5;

// =====================================================================
//  CONFIG PACKET TYPES
// =====================================================================

constexpr uint8_t CONFIG_TYPE_FULL     = 0x01;  // Full config JSON
constexpr uint8_t CONFIG_TYPE_SERIAL   = 0x02;  // Serial command relay
constexpr uint8_t CONFIG_TYPE_REQUEST  = 0x03;  // Request config from master
constexpr uint8_t CONFIG_TYPE_ACK      = 0x04;  // ACK response

// =====================================================================
//  HTTP ENDPOINTS
// =====================================================================

static constexpr const char* API_STATUS  = "/api/status";
static constexpr const char* API_CONFIG  = "/api/config";
static constexpr const char* API_SERIAL  = "/api/serial";
static constexpr const char* API_ESPNOW  = "/api/espnow";

// =====================================================================
//  NON-BLOCKING TIMER: Jeda
// =====================================================================

struct Jeda {
    uint32_t lastMs = 0;

    bool check(uint32_t intervalMs) {
        const uint32_t nowMs = millis();
        if (nowMs - lastMs < intervalMs) {
            return false;
        }
        lastMs = nowMs;
        return true;
    }

    void reset() {
        lastMs = millis();
    }
};

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================

void setupSerial();

// ESP-NOW
bool espNowConfigInit();
bool espNowConfigSendJson(const String& jsonStr);
bool espNowConfigSendRaw(const uint8_t* data, size_t len);
bool espNowConfigIsConnected();

// Web server
void webServerInit();
void webServerTick();

#endif // CONFIG_H
