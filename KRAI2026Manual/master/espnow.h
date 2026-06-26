/*
 * =====================================================================
 * FILE    : espnow.h
 * PERAN   : Konfigurasi khusus modul ESP-NOW.
 *           MAC whitelist, channel, magic, timing.
 *
 * CATATAN:
 *   ControlPacket struct ada di config.h (shared).
 *   File ini hanya berisi konfigurasi ESP-NOW receiver.
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
//  MAC ADDRESS — WAJIB DIISI SEBELUM UPLOAD
// =====================================================================
// MAC WiFi STA pengirim (s3controllerespnow).
// Aktifkan/dinonaktifkan via espNowEnableMacWhitelist. E0:72:A1:D5:DD:9C
constexpr bool espNowEnableMacWhitelist = true;
constexpr uint8_t espNowAllowedTransmitterStaMac[6] = {0xE0, 0x72, 0xA1, 0xD5, 0xDD, 0x9C};
constexpr uint8_t espNowAllowedTransmitterApMac[6]  = {0xE0, 0x72, 0xA1, 0xD5, 0xDD, 0x9C};

// =====================================================================
//  KONFIGURASI ESP-NOW
// =====================================================================

constexpr uint8_t  espNowChannel          = 1;
constexpr uint16_t ESPNOW_PACKET_MAGIC    = 0xA5B4;
constexpr unsigned long espNowLinkAliveMs = 500;
constexpr unsigned long espNowStatsIntervalMs = 1000;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
bool espNowControlInit();
void espNowControlTick();
bool espNowControlReadPacket(ControlPacket &outPacket);
bool espNowControlIsLinkAlive();

#endif // ESPNOW_H
