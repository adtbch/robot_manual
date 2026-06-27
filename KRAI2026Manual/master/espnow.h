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
// Aktifkan/dinonaktifkan via espNowEnableMacWhitelist. 94:A9:90:D2:32:90
constexpr bool espNowEnableMacWhitelist = true;
constexpr uint8_t espNowAllowedTransmitterStaMac[6] = {0x94, 0xA9, 0x90, 0xD2, 0x32, 0x90};
constexpr uint8_t espNowAllowedTransmitterApMac[6]  = {0x94, 0xA9, 0x90, 0xD2, 0x32, 0x90};

// =====================================================================
//  KONFIGURASI ESP-NOW
// =====================================================================

constexpr uint8_t  espNowChannel          = 1;
constexpr uint16_t ESPNOW_PACKET_MAGIC    = 0xA5B4;
constexpr unsigned long espNowLinkAliveMs = 2000;
constexpr unsigned long espNowStatsIntervalMs = 1000;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
bool espNowControlInit();
void espNowControlTick();
bool espNowControlReadPacket(ControlPacket &outPacket);
bool espNowControlIsLinkAlive();

#endif // ESPNOW_H
