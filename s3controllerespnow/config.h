/*
 * =====================================================================
 * FILE    : config.h
 * PERAN   : Pusat konfigurasi proyek s3controllerespnow.
 *           Struct ControlPacket, MAC address, konstanta, extern globals.
 *           TIDAK ADA logic / fungsi — hanya deklarasi.
 *
 * BOARD   : ESP32-S3 (native USB Host + ESP-NOW)
 *
 * ALIRAN DATA:
 *   [PS4 DualShock 4] --USB OTG--> [ESP32-S3]
 *         | parse HID → isi ControlPacket
 *         | ESP-NOW
 *         v
 *   [Receiver ESP32-S3]
 * =====================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// =====================================================================
//  MAC ADDRESS — WAJIB DIISI SEBELUM UPLOAD
// =====================================================================
// MAC ESP32-S3 penerima (target ESP-NOW).
// Cara cari: Serial Monitor receiver saat boot, baris "MAC STA". 94:A9:90:D2:32:90
constexpr uint8_t kEspNowTargetMac[6] = {0x94, 0xA9, 0x90, 0xD2, 0x32, 0x90};

// =====================================================================
//  KONFIGURASI KOMUNIKASI
// =====================================================================

// ESP-NOW WiFi channel (harus sama dengan receiver)
constexpr uint8_t kEspNowChannel = 1;

// Magic number validasi paket (harus sama di pengirim & penerima)
constexpr uint16_t kPacketMagic = 0xA5B4;

// =====================================================================
//  KONFIGURASI TIMING
// =====================================================================

// Interval kirim data (ms) — 50ms = 20 Hz
constexpr uint32_t kSendIntervalMs = 50;

// =====================================================================
//  STRUCT PAKET KONTROL
//  __attribute__((packed)) = larang compiler menambah padding bytes.
//  Layout memory harus IDENTIK di sisi penerima.
// =====================================================================
struct __attribute__((packed)) ControlPacket {
    uint16_t magic;       // = kPacketMagic (0xA5B4)

    uint8_t  checksum;    // XOR checksum data (lihat packet.ino)

    // --- Perintah gerak (siap pakai untuk motor) ---
    int16_t x;            // Gerakan lateral    (+= kanan, -= kiri)
    int16_t y;            // Gerakan maju/mundur (+= maju, -= mundur)
    int16_t w;            // Rotasi             (+= CCW,  -= CW)

    // --- Data analog mentah dari stik (-128..127) ---
    int8_t lx;            // Analog kiri  X
    int8_t ly;            // Analog kiri  Y
    int8_t rx;            // Analog kanan X
    int8_t ry;            // Analog kanan Y

    // --- Trigger analog (0..255) ---
    uint8_t l2Value;      // Trigger L2
    uint8_t r2Value;      // Trigger R2

    // --- IMU / Gyro (tidak tersedia via USB HID, selalu 0) ---
    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;

    // --- Bitmask semua tombol (1 bit per tombol) ---
    // bit 0  = Cross      bit 8  = L3
    // bit 1  = Circle      bit 9  = R3
    // bit 2  = Triangle    bit 10 = Up
    // bit 3  = Square      bit 11 = Down
    // bit 4  = L1          bit 12 = Left
    // bit 5  = R1          bit 13 = Right
    // bit 6  = L2 (digital) bit 14 = Share
    // bit 7  = R2 (digital) bit 15 = Options
    // bit 16 = PS Button   bit 17 = Touchpad
    uint32_t buttons;

    // --- Metadata paket ---
    uint16_t seq;         // Nomor urut (packet loss detection)
    uint8_t  connected;   // 1 = gamepad aktif, 0 = disconnect
};

// =====================================================================
//  VARIABEL GLOBAL BERSAMA
// =====================================================================

// Nomor urut paket (bertambah setiap paket dikirim)
extern uint16_t nomor_urut_paket;

// Status inisialisasi ESP-NOW
extern bool espnow_siap;

#endif // CONFIG_H
