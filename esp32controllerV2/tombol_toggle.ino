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

static bool     state_tombol_sebelumnya = 1;     // active LOW, 1 = tidak ditekan
static uint32_t waktu_tepi_terakhir     = 0;     // untuk debounce

// =====================================================================
//  FUNGSI: INISIALISASI PIN TOMBOL BOOT (native ESP-IDF GPIO)
// =====================================================================

/**
 * Konfigurasi GPIO0 sebagai input dengan pull-up internal.
 * Menggunakan gpio_config() native — bukan pinMode() Arduino.
 * Dipanggil satu kali di setup().
 */
void tombol_init() {
    const gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << kTombolBootPin),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    ESP_LOGI("button", "BOOT button init — GPIO%d (tekan = toggle jalur)", kTombolBootPin);
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
        ESP_LOGW("button", ">>> Pindah ke JALUR B — ESP-NOW (WiFi langsung) <<<");
    } else {
        jalur_aktif = JalurAktif::WSN31;
        ESP_LOGW("button", ">>> Pindah ke JALUR A — WSN-31 (radio UART) <<<");
    }
}

// =====================================================================
//  FUNGSI: UPDATE TOMBOL (non-blocking, native ESP-IDF GPIO)
// =====================================================================

/**
 * Baca state tombol BOOT via gpio_get_level() dan deteksi tepi turun.
 * Gunakan esp_timer_get_time() untuk debounce — bukan millis() Arduino.
 *
 * @param sekarang  nilai ms dari esp_timer_get_time()/1000
 */
void tombol_update(uint32_t sekarang) {
    const bool state_sekarang = (bool)gpio_get_level((gpio_num_t)kTombolBootPin);

    // Deteksi tepi turun: sebelumnya HIGH, sekarang LOW = tombol baru ditekan
    if (state_tombol_sebelumnya == 1 && state_sekarang == 0) {
        if (sekarang - waktu_tepi_terakhir >= kDebounceMs) {
            waktu_tepi_terakhir = sekarang;
            lakukan_toggle_jalur();
        }
    }

    state_tombol_sebelumnya = state_sekarang;
}
