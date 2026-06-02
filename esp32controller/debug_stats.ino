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

// =====================================================================
//  VARIABEL LOKAL
// =====================================================================

static uint32_t waktu_stats_terakhir = 0;

// =====================================================================
//  FUNGSI: INISIALISASI SERIAL
// =====================================================================

/**
 * Mulai Serial Monitor pada baudrate standar.
 * Harus dipanggil PERTAMA di setup() sebelum fungsi lain.
 */
void debug_init() {
    Serial.begin(115200);
    delay(300); // tunggu Serial stabil sebelum ada output
}

// =====================================================================
//  FUNGSI: CETAK INFO SAAT BOOT
// =====================================================================

/**
 * Cetak ringkasan konfigurasi sistem ke Serial Monitor.
 * Dipanggil satu kali di setup() setelah semua modul diinisialisasi.
 *
 * @param espnow_ok  hasil return dari espnow_init()
 */
void debug_cetak_info_boot(bool espnow_ok) {
    Serial.println("=========================================");
    Serial.println("     ESP32 CONTROLLER — DUAL CHANNEL     ");
    Serial.println("=========================================");
    Serial.printf("PS4 Bluetooth MAC target : %s\n", kPs4BluetoothMac);
    Serial.printf("ESP-NOW target MAC       : %02X:%02X:%02X:%02X:%02X:%02X\n",
        kEspNowTargetMac[0], kEspNowTargetMac[1], kEspNowTargetMac[2],
        kEspNowTargetMac[3], kEspNowTargetMac[4], kEspNowTargetMac[5]);
    Serial.printf("WSN-31 UART              : TX=%d RX=%d @%ld bps\n",
        kWsnTxPin, kWsnRxPin, kWsnBaudrate);
    Serial.printf("ESP-NOW channel          : %d\n", kEspNowChannel);
    Serial.printf("ESP-NOW init             : %s\n", espnow_ok ? "OK" : "GAGAL");
    Serial.printf("Interval kirim           : %lu ms (%lu Hz)\n",
        kSendIntervalMs, 1000UL / kSendIntervalMs);
    Serial.println("-----------------------------------------");
    Serial.println("Jalur default            : A — WSN-31 (radio UART)");
    Serial.println("Tekan tombol BOOT (GPIO0): toggle jalur A <-> B");
    Serial.println("=========================================");
}

// =====================================================================
//  FUNGSI: CETAK STATISTIK PERIODIK
// =====================================================================

/**
 * Cetak statistik sistem ke Serial Monitor setiap kStatsIntervalMs.
 * Non-blocking — gunakan millis() untuk throttle.
 *
 * Informasi yang dicetak:
 *   - Jalur aktif saat ini (A=WSN31 / B=ESPNOW)
 *   - Status koneksi PS4 (OK / PUTUS)
 *   - Total paket terkirim via WSN-31
 *   - Total paket terkirim via ESP-NOW
 *   - Total error pengiriman ESP-NOW
 *   - Nomor urut paket terakhir
 *
 * @param sekarang   nilai millis() dari loop utama
 * @param ps4_aktif  status koneksi PS4 saat ini
 */
void debug_cetak_statistik(uint32_t sekarang, bool ps4_aktif) {
    if (sekarang - waktu_stats_terakhir < kStatsIntervalMs) {
        return; // belum waktunya
    }
    waktu_stats_terakhir = sekarang;

    const char *nama_jalur  = (jalur_aktif == JalurAktif::WSN31) ? "A-WSN31" : "B-ESPNOW";
    const char *status_ps4  = ps4_aktif ? "OK" : "PUTUS";

    Serial.printf(
        "[STAT] jalur=%-8s ps4=%-5s wsn_tx=%lu espnow_tx=%lu espnow_err=%lu seq=%u\n",
        nama_jalur,
        status_ps4,
        (unsigned long)stat_kirim_wsn,
        (unsigned long)stat_kirim_espnow,
        (unsigned long)stat_espnow_error,
        nomor_urut_paket
    );
}
