/*
 * =====================================================================
 * FILE    : tombol_toggle.ino
 * PERAN   : Membaca tombol BOOT (GPIO0) bawaan ESP32 dan
 *           melakukan toggle jalur aktif antara WSN-31 dan ESP-NOW.
 *
 * ISI FILE INI:
 *   - tombol_init()    Setup pin GPIO0 (dipanggil di setup())
 *   - tombol_update()  Cek state tombol tiap iterasi loop (non-blocking)
 *
 * MEKANISME:
 *   Tombol BOOT ESP32 = GPIO0, active LOW (ditekan = LOW, lepas = HIGH).
 *   Pull-up internal sudah ada di ESP32, tidak perlu resistor eksternal.
 *
 *   Setiap kali tombol ditekan (tepi turun HIGH→LOW):
 *     - Jalur WSN31  → ganti ke ESPNOW
 *     - Jalur ESPNOW → ganti ke WSN31
 *   Status jalur baru dicetak ke Serial Monitor.
 *
 * DEBOUNCE:
 *   Non-blocking menggunakan millis(). Tepi berikutnya diabaikan
 *   selama kDebounceMs setelah tepi pertama terdeteksi.
 * =====================================================================
 */

#include "config.h"

// =====================================================================
//  VARIABEL LOKAL (hanya dipakai di file ini)
// =====================================================================

static bool     state_tombol_sebelumnya = HIGH;  // active LOW, default tidak ditekan
static uint32_t waktu_tepi_terakhir     = 0;     // untuk debounce

// =====================================================================
//  FUNGSI: INISIALISASI PIN TOMBOL BOOT
// =====================================================================

/**
 * Konfigurasi GPIO0 sebagai input dengan pull-up internal.
 * Dipanggil satu kali di setup().
 */
void tombol_init() {
    pinMode(kTombolBootPin, INPUT_PULLUP);
    Serial.printf("[TOMBOL] BOOT button init — GPIO%d (tekan = toggle jalur)\n", kTombolBootPin);
}

// =====================================================================
//  FUNGSI INTERNAL: LAKUKAN TOGGLE JALUR
// =====================================================================

/**
 * Ganti jalur aktif dan cetak info ke Serial Monitor.
 * Hanya dipanggil dari dalam tombol_update().
 */
static void lakukan_toggle_jalur() {
    if (jalur_aktif == JalurAktif::WSN31) {
        jalur_aktif = JalurAktif::ESPNOW;
        Serial.println("[TOMBOL] >>> Pindah ke JALUR B — ESP-NOW (WiFi langsung) <<<");
    } else {
        jalur_aktif = JalurAktif::WSN31;
        Serial.println("[TOMBOL] >>> Pindah ke JALUR A — WSN-31 (radio UART) <<<");
    }
}

// =====================================================================
//  FUNGSI: UPDATE TOMBOL (non-blocking, dipanggil setiap loop)
// =====================================================================

/**
 * Baca state tombol BOOT dan deteksi tepi turun (HIGH→LOW = ditekan).
 * Terapkan debounce berbasis millis() agar tidak memantul.
 *
 * @param sekarang  nilai millis() dari loop utama
 */
void tombol_update(uint32_t sekarang) {
    const bool state_sekarang = digitalRead(kTombolBootPin);

    // Deteksi tepi turun: sebelumnya HIGH, sekarang LOW = tombol baru ditekan
    if (state_tombol_sebelumnya == HIGH && state_sekarang == LOW) {
        // Pastikan sudah lewat waktu debounce sejak tepi terakhir
        if (sekarang - waktu_tepi_terakhir >= kDebounceMs) {
            waktu_tepi_terakhir = sekarang;
            lakukan_toggle_jalur();
        }
    }

    state_tombol_sebelumnya = state_sekarang;
}
