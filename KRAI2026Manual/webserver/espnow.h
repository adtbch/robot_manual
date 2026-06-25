/*
 * =====================================================================
 * FILE    : espnow.h
 * PERAN   : Konfigurasi modul ESP-NOW untuk Web Server.
 *           Mengirim config ke Master via ESP-NOW.
 *
 * BOARD   : ESP32 (Web Server)
 *
 * CATATAN:
 *   Protocol berbeda dari joystick (magic 0xC0DE vs 0xA5B4).
 *   Master harus handle kedua magic.
 * =====================================================================
 */

#ifndef ESPNOW_H
#define ESPNOW_H

#include "config.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <esp_mac.h>

// =====================================================================
//  ESP-NOW PACKET HEADER — untuk config protocol
// =====================================================================

struct __attribute__((packed)) EspNowConfigHeader {
    uint16_t magic;       // = 0xC0DE
    uint8_t  index;       // Packet index (0-based)
    uint8_t  total;       // Total packets
    uint8_t  type;        // CONFIG_TYPE_*
};

#endif // ESPNOW_H
