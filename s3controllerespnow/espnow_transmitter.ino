/*
 * =====================================================================
 * FILE    : espnow_transmitter.ino
 * PERAN   : Komunikasi ESP-NOW ke receiver (ESP32-S3 penerima).
 *
 * ISI FILE INI:
 *   - espnow_init()        Inisialisasi WiFi STA + ESP-NOW + peer
 *   - kirim_via_espnow()   Kirim ControlPacket ke receiver
 *   - callback send        Catat error jika pengiriman gagal
 *
 * CATATAN:
 *   ESP32-S3 bisa menjalankan WiFi (ESP-NOW) dan USB Host secara
 *   bersamaan karena menggunakan hardware peripheral yang terpisah.
 *
 *   espnow_init() harus dipanggil SEBELUM usb.begin() di setup().
 * =====================================================================
 */

#include "config.h"

// =====================================================================
//  CALLBACK: DIPANGGIL OTOMATIS SETELAH SETIAP esp_now_send()
// =====================================================================

/**
 * Callback pengiriman ESP-NOW.
 * Mencatat error ke Serial jika paket gagal terkirim.
 *
 * @param mac_addr MAC address tujuan
 * @param status   ESP_NOW_SEND_SUCCESS atau ESP_NOW_SEND_FAIL
 */
static void saat_espnow_terkirim(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        // Serial.println("[ESP-NOW] Send FAILED");
    }
}

// =====================================================================
//  FUNGSI: INISIALISASI ESP-NOW SEBAGAI TRANSMITTER
// =====================================================================

/**
 * Inisialisasi WiFi mode STA, set channel, init ESP-NOW,
 * daftarkan receiver sebagai peer tujuan.
 *
 * Dipanggil satu kali di setup() — SEBELUM usb.begin().
 *
 * @return true  jika inisialisasi berhasil
 * @return false jika ada error
 */
bool espnow_init() {
    // WiFi mode STA untuk ESP-NOW
    WiFi.mode(WIFI_STA);
    Serial.println("Mac Address: " + WiFi.macAddress());
    WiFi.disconnect(false, false);

    // Lock TX power ke max 20 dBm (ESP32-S3 max)
    esp_wifi_set_max_tx_power(80);  // 80 * 0.25 = 20 dBm
    Serial.println("[ESP-NOW] TX power: 20 dBm (max)");

    // Long range mode — pakai 802.11n HT20, data rate rendah, jarak lebih jauh
    WiFi.enableLongRange(true);
    Serial.println("[ESP-NOW] Long range: ON");

    // Matikan WiFi sleep — minimum latency untuk ESP-NOW
    WiFi.setSleep(false);
    Serial.println("[ESP-NOW] WiFi sleep: OFF");

    // Set channel agar cocok dengan receiver
    esp_wifi_set_promiscuous(true);
    const esp_err_t ch_err = esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (ch_err != ESP_OK) {
        Serial.printf("[ESP-NOW] Gagal set channel: err=%d\n", (int)ch_err);
        return false;
    }

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] esp_now_init GAGAL");
        return false;
    }

    // Register callback pengiriman
    esp_now_register_send_cb(saat_espnow_terkirim);

    // Daftarkan semua receiver sebagai peer
    for (uint8_t i = 0; i < kEspNowTargetCount; i++) {
        if (!esp_now_is_peer_exist(kEspNowTargetMacs[i])) {
            esp_now_peer_info_t peer = {};
            memcpy(peer.peer_addr, kEspNowTargetMacs[i], 6);
            peer.channel = kEspNowChannel;
            peer.encrypt = false;
            peer.ifidx   = WIFI_IF_STA;

            const esp_err_t peer_err = esp_now_add_peer(&peer);
            if (peer_err != ESP_OK && peer_err != ESP_ERR_ESPNOW_EXIST) {
                Serial.printf("[ESP-NOW] Peer %d gagal: err=%d\n", i, (int)peer_err);
                return false;
            }
        }
    }

    Serial.println("[ESP-NOW] Init OK — target(s):");
    for (uint8_t i = 0; i < kEspNowTargetCount; i++) {
        Serial.printf("  [%d] %02X:%02X:%02X:%02X:%02X:%02X\n", i,
            kEspNowTargetMacs[i][0], kEspNowTargetMacs[i][1],
            kEspNowTargetMacs[i][2], kEspNowTargetMacs[i][3],
            kEspNowTargetMacs[i][4], kEspNowTargetMacs[i][5]);
    }
    Serial.printf("  ch=%d\n", kEspNowChannel);

    return true;
}

// =====================================================================
//  FUNGSI: KIRIM ControlPacket VIA ESP-NOW
// =====================================================================

/**
 * Kirim ControlPacket ke receiver via ESP-NOW.
 * Tidak melakukan apa-apa jika espnow_siap == false.
 *
 * @param paket ControlPacket yang akan dikirim
 */
void kirim_via_espnow(const ControlPacket &paket) {
    if (!espnow_siap) return;

    // nullptr = kirim ke semua peer — master utama + spare
    const esp_err_t err = esp_now_send(
        nullptr,
        reinterpret_cast<const uint8_t *>(&paket),
        sizeof(ControlPacket)
    );

    if (err != ESP_OK) {
        Serial.printf("[ESP-NOW] Kirim error: %d\n", (int)err);
    }
}
