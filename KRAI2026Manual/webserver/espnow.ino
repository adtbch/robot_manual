/*
 * =====================================================================
 * FILE    : espnow.ino
 * PERAN   : ESP-NOW sender — mengirim config ke Master ESP32-S3.
 *
 * ISI FILE INI:
 *   - espNowConfigInit()       Inisialisasi WiFi STA + ESP-NOW + peer
 *   - espNowConfigSendJson()   Kirim JSON config ke master (fragmented)
 *   - espNowConfigSendRaw()    Kirim raw bytes ke master
 *   - espNowConfigIsConnected() Cek apakah master terdaftar sebagai peer
 *
 * ALIRAN DATA:
 *   [Web UI] → POST /api/config
 *   [Web Server ESP32] → espNowConfigSendJson()
 *   [Master ESP32-S3] → parse & apply config
 *
 * PROTOCOL:
 *   Setiap packet: [Header: 5 bytes] + [Payload: up to 245 bytes]
 *   Header: magic(2) + index(1) + total(1) + type(1)
 *   Magic = 0xC0DE (config), beda dari joystick (0xA5B4)
 *
 * CATATAN:
 *   - WiFi STA mode untuk ESP-NOW (bukan AP mode)
 *   - AP mode di-handle oleh web server (separate task/implementation)
 *   - ESP-NOW dan WiFi AP bisa jalan bareng di ESP32 (dual-mode)
 * =====================================================================
 */

#include "espnow.h"

namespace {

// =====================================================================
//  STATE
// =====================================================================

bool gEspNowReady = false;
bool gMasterConnected = false;
uint32_t gLastSendMs = 0;
uint32_t gSendCount = 0;
uint32_t gSendFailCount = 0;

// =====================================================================
//  HELPERS
// =====================================================================

void printMac(const char* label, const uint8_t* mac) {
    Serial.printf("%s%02X:%02X:%02X:%02X:%02X:%02X\n",
                  label,
                  mac[0], mac[1], mac[2],
                  mac[3], mac[4], mac[5]);
}

bool ensurePeer(const uint8_t* peerMac, const char* label) {
    if (esp_now_is_peer_exist(peerMac)) {
        return true;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerMac, 6);
    peerInfo.channel = ESPNOW_CONFIG_CHANNEL;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    const esp_err_t err = esp_now_add_peer(&peerInfo);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        Serial.printf("[ESPNOW] Gagal daftarkan peer %s: err=%d\n", label, (int)err);
        return false;
    }
    return true;
}

// =====================================================================
//  CALLBACK — dikirim paket
// =====================================================================

void onEspNowSendComplete(const uint8_t* macAddr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        gMasterConnected = true;
    } else {
        gMasterConnected = false;
        gSendFailCount++;
    }
}

}  // namespace

// =====================================================================
//  PUBLIC API
// =====================================================================

bool espNowConfigInit() {
    // WiFi STA mode untuk ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, false);

    // Set channel
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ESPNOW_CONFIG_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    // Print MAC
    uint8_t staMac[6];
    if (esp_read_mac(staMac, ESP_MAC_WIFI_STA) == ESP_OK) {
        printMac("[ESPNOW] Web Server MAC: ", staMac);
    }
    printMac("[ESPNOW] Target Master MAC: ", MASTER_MAC);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNOW] esp_now_init GAGAL");
        return false;
    }

    // Register send callback
    esp_now_register_send_cb(onEspNowSendComplete);

    // Add master as peer
    if (!ensurePeer(MASTER_MAC, "master")) {
        Serial.println("[ESPNOW] Gagal daftarkan master sebagai peer");
        return false;
    }

    gEspNowReady = true;
    Serial.println("[ESPNOW] Init OK — siap kirim config ke master");
    return true;
}

bool espNowConfigSendJson(const String& jsonStr) {
    if (!gEspNowReady) {
        Serial.println("[ESPNOW] Belum siap");
        return false;
    }

    const size_t totalLen = jsonStr.length();
    const uint8_t totalPackets = (totalLen + ESPNOW_PAYLOAD_MAX - 1) / ESPNOW_PAYLOAD_MAX;

    Serial.printf("[ESPNOW] Mengirim config: %u bytes, %d packets\n",
                  (unsigned)totalLen, totalPackets);

    for (uint8_t i = 0; i < totalPackets; i++) {
        const size_t offset = i * ESPNOW_PAYLOAD_MAX;
        const size_t chunkLen = min(ESPNOW_PAYLOAD_MAX, totalLen - offset);

        // Build packet: header + payload
        uint8_t packet[ESPNOW_HEADER_SIZE + ESPNOW_PAYLOAD_MAX];
        EspNowConfigHeader* hdr = reinterpret_cast<EspNowConfigHeader*>(packet);
        hdr->magic = ESPNOW_CONFIG_MAGIC;
        hdr->index = i;
        hdr->total = totalPackets;
        hdr->type = CONFIG_TYPE_FULL;

        memcpy(packet + ESPNOW_HEADER_SIZE, jsonStr.c_str() + offset, chunkLen);

        const size_t packetLen = ESPNOW_HEADER_SIZE + chunkLen;

        // Send
        esp_err_t result = esp_now_send(MASTER_MAC, packet, packetLen);

        if (result != ESP_OK) {
            Serial.printf("[ESPNOW] Gagal kirim packet %d: err=%d\n", i, (int)result);
            gSendFailCount++;
            return false;
        }

        // Delay antar packet untuk reliability
        if (i < totalPackets - 1) {
            delay(20);
        }
    }

    gSendCount++;
    gLastSendMs = millis();
    Serial.printf("[ESPNOW] Config terkirim OK (%d packets)\n", totalPackets);
    return true;
}

bool espNowConfigSendRaw(const uint8_t* data, size_t len) {
    if (!gEspNowReady) {
        return false;
    }

    esp_err_t result = esp_now_send(MASTER_MAC, data, len);
    if (result != ESP_OK) {
        gSendFailCount++;
        return false;
    }

    gSendCount++;
    gLastSendMs = millis();
    return true;
}

bool espNowConfigIsConnected() {
    return gEspNowReady && gMasterConnected;
}
