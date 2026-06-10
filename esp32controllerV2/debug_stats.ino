/*
 * =====================================================================
 * FILE    : debug_stats.ino
 * PERAN   : Semua output informatif ke Serial Monitor terpusat di sini.
 *           Memudahkan debugging dan monitoring kondisi sistem.
 *
 * ISI FILE INI:
 *   - debug_init()              Inisialisasi Serial (dipanggil pertama di setup())
 *   - debug_cetak_info_boot()   Ringkasan konfigurasi saat pertama menyala
 *   - debug_cetak_statistik()   Statistik periodik (tiap kStatsIntervalMs)
 *
 * OUTPUT SERIAL YANG DIHASILKAN:
 *   Boot  : info MAC, pin, channel, jalur default
 *   Tiap 3s: jalur aktif, status PS4, count kirim WSN/ESPNOW, error count
 * =====================================================================
 */

#include "config.h"
#include <WiFi.h>



// =====================================================================
//  VARIABEL LOKAL
// =====================================================================

static uint32_t waktu_stats_terakhir = 0;

// =====================================================================
//  FUNGSI: INISIALISASI SERIAL + ESP-IDF LOG
// =====================================================================

/**
 * Mulai Serial (untuk PS4Controller compatibility) dan konfigurasi
 * ESP-IDF logging level. Dipanggil PERTAMA di setup().
 */
void debug_init() {
    Serial.begin(115200);
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("system", ESP_LOG_INFO);
    vTaskDelay(pdMS_TO_TICKS(300));
}

// =====================================================================
//  FUNGSI: CETAK INFO SAAT BOOT
// =====================================================================

/**
 * Cetak ringkasan konfigurasi sistem via ESP_LOGI.
 * Dipanggil satu kali di setup() setelah semua modul diinisialisasi.
 *
 * @param espnow_ok  hasil return dari espnow_init()
 */
void debug_cetak_info_boot(bool espnow_ok) {
    ESP_LOGI("system", "========================================");
    ESP_LOGI("system", "   ESP32 CONTROLLER — DUAL CHANNEL      ");
    ESP_LOGI("system", "========================================");
    ESP_LOGI("system", "Board WiFi STA MAC : %s", WiFi.macAddress().c_str());
    ESP_LOGI("system", "PS4 target MAC     : %s", kPs4BluetoothMac);
    ESP_LOGI("system", "ESP-NOW target MAC : %02X:%02X:%02X:%02X:%02X:%02X",
        kEspNowTargetMac[0], kEspNowTargetMac[1], kEspNowTargetMac[2],
        kEspNowTargetMac[3], kEspNowTargetMac[4], kEspNowTargetMac[5]);
    ESP_LOGI("system", "WSN-31 UART        : TX=%d RX=%d SET=%d @%ld bps",
        kWsnTxPin, kWsnRxPin, kWsnSetPin, kWsnBaudrate);
    ESP_LOGI("system", "ESP-NOW channel    : %d", kEspNowChannel);
    ESP_LOGI("system", "ESP-NOW init       : %s", espnow_ok ? "OK" : "GAGAL");
    ESP_LOGI("system", "Interval kirim     : WSN31=%lu ms ESP-NOW=%lu ms",
        (unsigned long)kSendIntervalMsWsn31, (unsigned long)kSendIntervalMsEspnow);
    ESP_LOGI("system", "----------------------------------------");
    ESP_LOGI("system", "Tekan BOOT GPIO0   : toggle jalur B <-> A");
    ESP_LOGI("system", "========================================");
}

// =====================================================================
//  FUNGSI: CETAK STATISTIK PERIODIK
// =====================================================================

/**
 * Cetak statistik sistem via ESP_LOGI setiap kStatsIntervalMs.
 *
 * @param sekarang   nilai ms dari esp_timer_get_time()/1000
 * @param ps4_aktif  status koneksi PS4 saat ini
 */
void debug_cetak_statistik(uint32_t sekarang, bool ps4_aktif) {
    if (sekarang - waktu_stats_terakhir < kStatsIntervalMs) {
        return;
    }
    waktu_stats_terakhir = sekarang;

    const char* nama_jalur = (jalur_aktif == JalurAktif::WSN31) ? "A-WSN31" : "B-ESPNOW";
    const char* status_ps4 = ps4_aktif ? "OK" : "PUTUS";

    ESP_LOGI("STAT", "jalur=%-8s ps4=%-5s wsn_tx=%lu espnow_tx=%lu espnow_err=%lu seq=%u",
             nama_jalur, status_ps4,
             (unsigned long)stat_kirim_wsn,
             (unsigned long)stat_kirim_espnow,
             (unsigned long)stat_espnow_error,
             nomor_urut_paket);
}
