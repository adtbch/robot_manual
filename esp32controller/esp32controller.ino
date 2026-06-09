/*
 * =====================================================================
 * FILE    : esp32controller.ino
 * BOARD   : ESP32 (bukan S3)
 * PERAN   : Entry point utama — hanya berisi setup() dan loop().
 *           Semua logic ada di file .ino masing-masing modul.
 *
 * ARSITEKTUR SISTEM:
 *
 *   [PS4 DualShock 4]
 *         | Bluetooth Classic
 *         v
 *   [ESP32 Controller]  <-- tombol BOOT (GPIO0) = toggle jalur
 *         |
 *         +-- JALUR A (WSN31)  --> TX26 --> WSN-31 ~~radio~~ WSN-31 --> ESP32-S3
 *         |
 *         +-- JALUR B (ESPNOW) --> WiFi 2.4GHz ESP-NOW langsung --> ESP32-S3
 *
 * STRUKTUR FILE:
 *   esp32controller.ino    <- INI (entry point, setup & loop)
 *   config.h               <- konstanta, pin, enum, extern
 *   packet.ino             <- ControlPacket struct + helper
 *   ps4_bluetooth.ino      <- baca input PS4
 *   wsn_serial.ino         <- kirim via WSN-31 UART binary
 *   espnow_transmitter.ino <- kirim via ESP-NOW WiFi
 *   tombol_toggle.ino      <- toggle jalur via tombol BOOT
 *   debug_stats.ino        <- output Serial Monitor
 *
 * URUTAN SETUP (penting — jangan diubah):
 *   1. debug_init()        Serial harus aktif duluan untuk logging error
 *   2. wsn_serial_init()   UART ke WSN-31
 *   3. ps4_init()          Bluetooth PS4 — init duluan
 *   4. espnow_init()       WiFi/ESP-NOW — setelah Bluetooth selesai
 *   5. tombol_init()       GPIO tombol BOOT
 *   6. led_ps4_init()      LED status PS4
 *   7. debug_cetak_info_boot()
 * =====================================================================
 */

#include "config.h"

// =====================================================================
//  DEFINISI VARIABEL GLOBAL (dideklarasikan extern di config.h)
//  Hanya didefinisikan satu kali di sini — file lain pakai extern.
// =====================================================================

JalurAktif jalur_aktif       = JalurAktif::ESPNOW; // default: Jalur B (ESP-NOW)
uint16_t   nomor_urut_paket  = 0;

uint32_t stat_kirim_wsn      = 0;
uint32_t stat_kirim_espnow   = 0;
uint32_t stat_espnow_error   = 0;

bool     espnow_siap         = false;
uint32_t waktu_ps4_terakhir  = 0;

// =====================================================================
//  SETUP — dijalankan sekali saat ESP32 menyala / reset
// =====================================================================
void setup() {
    // 1. Serial harus pertama agar semua log modul bisa tercetak
    debug_init();

    // 2. UART ke WSN-31
    wsn_serial_init();

    // 3. Bluetooth PS4 — init duluan agar WiFi tidak mengganggu
    ps4_init();

    // 4. ESP-NOW (WiFi) — setelah Bluetooth selesai
    espnow_siap = espnow_init();

    // 5. Tombol BOOT
    tombol_init();

    // 6. LED status PS4
    led_ps4_init();

    // 7. Cetak ringkasan konfigurasi ke Serial Monitor
    debug_cetak_info_boot(espnow_siap);

    // Catat waktu awal agar timeout tidak langsung terpicu
    waktu_ps4_terakhir = millis();
}

// =====================================================================
//  LOOP — dijalankan terus-menerus (Super-Loop non-blocking)
// =====================================================================
void loop() {
    const uint32_t sekarang = millis();

    // --- 1. Cek tombol BOOT → toggle jalur jika ditekan ---
    tombol_update(sekarang);

    // --- 2. Tentukan status koneksi PS4 ---
    const bool ps4_aktif = ps4_is_aktif(sekarang);

    // --- 3. Update LED status PS4 ---
    led_ps4_update(ps4_aktif);

    // --- 4. Kirim data sesuai interval ---
    static uint32_t waktu_kirim_terakhir = 0;
    if (sekarang - waktu_kirim_terakhir >= kSendIntervalMs) {
        waktu_kirim_terakhir = sekarang;

        // Buat paket sesuai status PS4
        ControlPacket paket = {};
        if (ps4_aktif) {
            ps4_baca_paket(paket);
        } else {
            buat_paket_stop(paket);
        }

        // Kirim ke jalur yang sedang aktif
        if (jalur_aktif == JalurAktif::WSN31) {
            kirim_via_wsn31(paket);
        } else {
            kirim_via_espnow(paket);
        }
    }

    // --- 5. Cetak statistik periodik ke Serial Monitor ---
    debug_cetak_statistik(sekarang, ps4_aktif);
}
