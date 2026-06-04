/*
 * =====================================================================
 * FILE    : config.h
 * PERAN   : Pusat konfigurasi seluruh proyek esp32controller.
 *           Hanya berisi konstanta, pin, enum, dan deklarasi extern.
 *           TIDAK ADA logic / fungsi di sini.
 *
 * CARA PAKAI:
 *   Setiap file .ino wajib meng-include file ini.
 *   #include "config.h"
 * =====================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <esp_now.h>

// =====================================================================
//  MAC ADDRESS — WAJIB DIISI SEBELUM UPLOAD
// =====================================================================

// MAC Bluetooth PS4 DualShock 4.
// Cara cari: flash sketch GetBDAddress (dari contoh library PS4Controller),
// lalu baca output Serial Monitor.
// Format: "xx:xx:xx:xx:xx:xx" (huruf kecil, dipisah titik dua)
static const char kPs4BluetoothMac[] = "4c:11:ae:75:d7:32"; // PS4 DualShock 4

// MAC WiFi STA ESP32-S3 penerima (target ESP-NOW).ard
// Cara cari: lihat Serial Monitor ESP32-S3 saat boot, baris "MAC STA".
// Format: { 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX }44:1B:F6:D4:0C:0C
constexpr uint8_t kEspNowTargetMac[6] = {0x44, 0x1B, 0xF6, 0xD4, 0x0C, 0x0C}; // ESP32-S3 Master

// =====================================================================
//  KONFIGURASI PIN
// =====================================================================

// WSN-31 UART (Serial2)
// RX ESP32 (26) ← TXD WSN-31 (terima dari modul)
// TX ESP32 (27) → RXD WSN-31 (kirim ke modul)
constexpr int kWsnRxPin = 26;
constexpr int kWsnTxPin = 27;

// WSN-31 pin SET — tarik LOW untuk masuk mode konfigurasi AT command
constexpr int kWsnSetPin = 25;

// Tombol BOOT bawaan ESP32 (GPIO0, active LOW, sudah ada pull-up internal)
constexpr int kTombolBootPin = 0;

// LED onboard ESP32 (GPIO2 pada ESP32 Dev Module)
constexpr int kLedPs4Pin = 2;

// Interval kedipan LED saat PS4 terputus (ms)
constexpr uint32_t kBlinkIntervalMs = 250;

// =====================================================================
//  KONFIGURASI KOMUNIKASI
// =====================================================================

// WSN-31 UART
constexpr long kWsnBaudrate = 9600;

// ESP-NOW WiFi channel (harus sama dengan ESP32-S3 penerima)
constexpr uint8_t kEspNowChannel = 1;

// Magic number validasi paket (harus sama di pengirim & penerima)
constexpr uint16_t kPacketMagic = 0xA5B4;

// Header frame UART binary ke WSN-31
constexpr uint8_t kFrameStart0 = 0xAA;
constexpr uint8_t kFrameStart1 = 0x55;

// =====================================================================
//  KONFIGURASI TIMING
// =====================================================================

// Interval kirim data ke ESP32-S3 (ms) — 25ms = 40 Hz
constexpr uint32_t kSendIntervalMs = 25;

// Debounce tombol BOOT (ms)
constexpr uint32_t kDebounceMs = 50;

// Timeout PS4: jika tidak ada update selama ini, anggap disconnect (ms)
constexpr uint32_t kPs4TimeoutMs = 500;

// Interval cetak statistik ke Serial Monitor (ms)
constexpr uint32_t kStatsIntervalMs = 3000;

// =====================================================================
//  ENUM: JALUR TRANSMISI AKTIF
// =====================================================================
enum class JalurAktif : uint8_t {
    WSN31  = 0,  // Jalur A: radio WSN-31 via UART binary
    ESPNOW = 1,  // Jalur B: ESP-NOW WiFi langsung ke ESP32-S3
};

// =====================================================================
//  STRUCT PAKET KONTROL
//  __attribute__((packed)) = larang compiler menambah padding bytes.
//  Wajib agar sizeof() dan layout memory sama persis di semua board.
//  HARUS IDENTIK di sisi penerima (ESP32-S3).
// =====================================================================
struct __attribute__((packed)) ControlPacket {
    uint16_t magic;      // Harus = kPacketMagic (0xA5B4). Validasi di penerima.

    // --- Perintah gerak (sudah diskala, siap pakai untuk motor) ---
    int16_t x;           // Gerakan lateral    (+= kanan, -= kiri)
    int16_t y;           // Gerakan maju/mundur (+= maju, -= mundur)
    int16_t w;           // Rotasi             (+= CCW,  -= CW)

    // --- Data analog mentah dari stik PS4 (-128..127) ---
    int8_t lx;           // Analog kiri  X
    int8_t ly;           // Analog kiri  Y
    int8_t rx;           // Analog kanan X
    int8_t ry;           // Analog kanan Y

    // --- Trigger analog PS4 (0..255) ---
    uint8_t l2Value;     // Trigger L2
    uint8_t r2Value;     // Trigger R2

    // --- IMU / Gyro dari PS4 DualShock 4 ---
    int16_t gyrX;        // Gyro sumbu X
    int16_t gyrY;        // Gyro sumbu Y
    int16_t gyrZ;        // Gyro sumbu Z

    // --- Bitmask semua tombol (1 bit per tombol) ---
    uint32_t buttons;

    // --- Metadata paket ---
    uint16_t seq;        // Nomor urut (counter). Dipakai penerima untuk deteksi packet loss.
    uint8_t  connected;  // 1 = PS4 terhubung & aktif, 0 = disconnect / stop
};

// =====================================================================
//  VARIABEL GLOBAL BERSAMA (definisi ada di esp32controller.ino)
//  File lain cukup extern — tidak perlu mendefinisikan ulang.
// =====================================================================

// Jalur yang sedang aktif (default: WSN31, bisa di-toggle via tombol BOOT)
extern JalurAktif jalur_aktif;

// Nomor urut paket (bertambah setiap paket dikirim, untuk deteksi packet loss)
extern uint16_t nomor_urut_paket;

// Statistik pengiriman (diisi oleh masing-masing modul)
extern uint32_t stat_kirim_wsn;
extern uint32_t stat_kirim_espnow;
extern uint32_t stat_espnow_error;

// Status inisialisasi ESP-NOW (diisi oleh espnow_transmitter.ino)
extern bool espnow_siap;

// Waktu terakhir paket PS4 diterima (diisi oleh ps4_bluetooth.ino)
extern uint32_t waktu_ps4_terakhir;

#endif // CONFIG_H
