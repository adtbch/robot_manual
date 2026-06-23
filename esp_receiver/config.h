/*
 * =====================================================================
 * FILE    : config.h
 * PERAN   : Pusat konfigurasi ESP-NOW receiver.
 *           ControlPacket struct (IDENTIK dengan s3controllerespnow),
 *           MAC whitelist, konstanta, validasi paket.
 *
 * BOARD   : ESP32 (penerima)
 *
 * CATATAN:
 *   ControlPacket HARUS sama persis dengan sisi pengirim.
 *   Konsep ESP-NOW diambil dari esp32s3_master (sudah matang).
 * =====================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <esp_mac.h>

// =====================================================================
//  MAC ADDRESS — WAJIB DIISI SEBELUM UPLOAD
// =====================================================================

// MAC WiFi STA pengirim (s3controllerespnow).
// Aktifkan/dinonaktifkan via espNowEnableMacWhitelist.
constexpr bool espNowEnableMacWhitelist = true; // E0:72:A1:D5:DD:9C
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
//  STRUCT PAKET KONTROL — IDENTIK dengan s3controllerespnow
//  __attribute__((packed)) = tanpa padding bytes
// =====================================================================

struct __attribute__((packed)) ControlPacket {
    uint16_t magic;       // = ESPNOW_PACKET_MAGIC (0xA5B4)

    uint8_t  checksum;    // XOR checksum data

    // --- Perintah gerak ---
    int16_t x;            // Lateral    (+= kanan, -= kiri)
    int16_t y;            // Maju/mundur (+= maju, -= mundur)
    int16_t w;            // Rotasi     (+= CCW,  -= CW)

    // --- Analog stik (-128..127) ---
    int8_t lx;
    int8_t ly;
    int8_t rx;
    int8_t ry;

    // --- Trigger (0..255) ---
    uint8_t l2Value;
    uint8_t r2Value;

    // --- Gyro (selalu 0 via USB HID) ---
    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;

    // --- Bitmask tombol ---
    // bit 0=Cross  1=Circle  2=Triangle  3=Square
    // bit 4=L1  5=R1  6=L2(d)  7=R2(d)
    // bit 8=L3  9=R3  10=Up  11=Down  12=Left  13=Right
    // bit 14=Share  15=Options  16=PS  17=Touchpad
    uint32_t buttons;

    // --- Metadata ---
    uint16_t seq;
    uint8_t  connected;
};

// =====================================================================
//  FUNGSI: HITUNG XOR CHECKSUM
// =====================================================================

static uint8_t hitung_checksum(const ControlPacket &p) {
    const uint8_t *raw = reinterpret_cast<const uint8_t *>(&p);
    const size_t start = offsetof(ControlPacket, x);
    const size_t end   = sizeof(ControlPacket);

    uint8_t cs = 0;
    for (size_t i = start; i < end; i++) {
        cs ^= raw[i];
    }
    return cs;
}

// =====================================================================
//  BITMASK TOMBOL (untuk decode)
// =====================================================================

#define BTN_CROSS    (1u << 0)
#define BTN_CIRCLE   (1u << 1)
#define BTN_TRIANGLE (1u << 2)
#define BTN_SQUARE   (1u << 3)
#define BTN_L1       (1u << 4)
#define BTN_R1       (1u << 5)
#define BTN_L2       (1u << 6)
#define BTN_R2       (1u << 7)
#define BTN_L3       (1u << 8)
#define BTN_R3       (1u << 9)
#define BTN_UP       (1u << 10)
#define BTN_DOWN     (1u << 11)
#define BTN_LEFT     (1u << 12)
#define BTN_RIGHT    (1u << 13)
#define BTN_SHARE    (1u << 14)
#define BTN_OPTIONS  (1u << 15)
#define BTN_PS       (1u << 16)
#define BTN_TOUCHPAD (1u << 17)

#endif // CONFIG_H
