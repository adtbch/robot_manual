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
// Daftar MAC controller yang diizinkan — tambah spare di sini.
constexpr bool espNowEnableMacWhitelist = true;
constexpr uint8_t espNowAllowedTransmitterMacs[][6] = {
    {0xDC, 0xDA, 0x0C, 0x70, 0x76, 0xB8}, // Controller 1 DC:DA:0C:70:76:B8
    {0xA4, 0xCB, 0x8F, 0x99, 0xDA, 0x40}, // Spare 1 — A4:CB:8F:99:DA:40
    {0x8C, 0xBF, 0xEA, 0x17, 0x4D, 0x10}, // Spare 2 - 8C:BF:EA:17:4D:10
};
constexpr uint8_t espNowAllowedTransmitterCount = sizeof(espNowAllowedTransmitterMacs) / 6;

// =====================================================================
//  KONFIGURASI ESP-NOW
// =====================================================================

// ESP-NOW WiFi channel — harus sama di controller & master
extern uint8_t espNowChannel;
constexpr uint16_t ESPNOW_PACKET_MAGIC    = 0xA5B4;
constexpr unsigned long espNowLinkAliveMs = 500;  // ponytail: 2000→500, connected forward sudah instan
constexpr unsigned long espNowStatsIntervalMs = 1000;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
bool espNowControlInit();
void espNowControlTick();
bool espNowControlReadPacket(ControlPacket &outPacket);
bool espNowControlIsLinkAlive();

#endif // ESPNOW_H
