/*
 * =====================================================================
 * FILE    : espnow_transmitter.ino
 * PERAN   : Komunikasi langsung ke ESP32-S3 via protokol ESP-NOW (WiFi).
 *           Ini adalah JALUR B (backup) yang aktif saat WSN-31 bermasalah.
 *
 * ISI FILE INI:
 *   - espnow_init()        Inisialisasi WiFi + ESP-NOW (dipanggil di setup())
 *   - kirim_via_espnow()   Kirim ControlPacket langsung ke ESP32-S3
 *   - callback send        Catat error jika pengiriman gagal
 *
 * CATATAN:
 *   ESP-NOW berjalan di atas stack WiFi ESP32, sedangkan PS4Controller
 *   menggunakan Bluetooth Classic (BR/EDR) — keduanya hardware terpisah
 *   dan bisa berjalan bersamaan tanpa konflik.
 *
 *   espnow_init() HARUS dipanggil SEBELUM ps4_init() di setup()
 *   agar inisialisasi WiFi selesai lebih dulu.
 *
 * ALIRAN DATA (Jalur B):
 *   ESP32 Controller → WiFi 2.4GHz (ESP-NOW) → ESP32-S3
 * =====================================================================
 */

#include "config.h"



// =====================================================================
//  CALLBACK: DIPANGGIL OTOMATIS SETELAH SETIAP esp_now_send()
//  Berjalan di task WiFi internal ESP-IDF, bukan di loop().
// =====================================================================

/**
 * Callback pengiriman ESP-NOW.
 * Mencatat error ke stat_espnow_error jika paket gagal terkirim.
 *
 * @param mac    MAC address tujuan
 * @param status ESP_NOW_SEND_SUCCESS atau ESP_NOW_SEND_FAIL
 */
static void saat_espnow_terkirim(const uint8_t *mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        stat_espnow_error++;
    }
}

// =====================================================================
//  FUNGSI: INISIALISASI ESP-NOW SEBAGAI TRANSMITTER
// =====================================================================

/**
 * Inisialisasi WiFi mode STA, set channel, init ESP-NOW,
 * daftarkan ESP32-S3 sebagai peer tujuan.
 *
 * Dipanggil satu kali di setup() — SEBELUM ps4_init().
 *
 * @return true  jika inisialisasi berhasil
 * @return false jika ada error (akan tercetak di Serial)
 */
bool espnow_init() {
    // Set WiFi mode STA agar ESP-NOW dapat berjalan
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, false);
    WiFi.setSleep(true); // Wajib aktif saat WiFi + Bluetooth bersamaan

    // Set channel agar cocok dengan ESP32-S3 penerima
    esp_wifi_set_promiscuous(true);
    const esp_err_t ch_err = esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (ch_err != ESP_OK) {
        ESP_LOGE("espnow", "Gagal set channel: err=%d", (int)ch_err);
    }

    // Inisialisasi ESP-NOW
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE("espnow", "esp_now_init GAGAL");
        return false;
    }

    // Daftarkan callback pengiriman
    esp_now_register_send_cb(saat_espnow_terkirim);

    // Daftarkan ESP32-S3 sebagai peer tujuan
    if (!esp_now_is_peer_exist(kEspNowTargetMac)) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, kEspNowTargetMac, 6);
        peer.channel = kEspNowChannel;
        peer.encrypt = false;
        peer.ifidx   = WIFI_IF_STA;

        const esp_err_t peer_err = esp_now_add_peer(&peer);
        if (peer_err != ESP_OK && peer_err != ESP_ERR_ESPNOW_EXIST) {
            ESP_LOGE("espnow", "Gagal daftarkan peer: err=%d", (int)peer_err);
            return false;
        }
    }

    ESP_LOGI("espnow", "Init OK — target %02X:%02X:%02X:%02X:%02X:%02X ch=%d",
        kEspNowTargetMac[0], kEspNowTargetMac[1], kEspNowTargetMac[2],
        kEspNowTargetMac[3], kEspNowTargetMac[4], kEspNowTargetMac[5],
        kEspNowChannel);

    return true;
}

// =====================================================================
//  FUNGSI: KIRIM ControlPacket VIA ESP-NOW
// =====================================================================

/**
 * Kirim ControlPacket langsung ke ESP32-S3 via ESP-NOW.
 * Tidak melakukan apa-apa jika espnow_siap == false.
 *
 * @param paket ControlPacket yang akan dikirim
 */
void kirim_via_espnow(const ControlPacket &paket) {
    if (!espnow_siap) {
        return;
    }

    const esp_err_t err = esp_now_send(
        kEspNowTargetMac,
        reinterpret_cast<const uint8_t *>(&paket),
        sizeof(ControlPacket)
    );

    if (err == ESP_OK) {
        stat_kirim_espnow++;
    } else {
        stat_espnow_error++;
        ESP_LOGE("espnow", "Kirim error: %d", (int)err);
    }
}
