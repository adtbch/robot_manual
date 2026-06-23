/*
 * =====================================================================
 * FILE    : espnow_receiver.ino
 * PERAN   : Menerima ControlPacket via ESP-NOW dari s3controllerespnow.
 *
 * KONSEP  : Diambil dari esp32s3_master/espnow_control.ino (sudah matang).
 *           - portMUX untuk critical section (ISR-safe)
 *           - MAC whitelist untuk keamanan
 *           - Sequence number validation (anti duplikat)
 *           - Statistik lengkap
 *           - Link alive detection
 *
 * ISI FILE INI:
 *   - espnow_receiver_init()    Inisialisasi WiFi + ESP-NOW
 *   - espnow_receiver_tick()    Cetak statistik periodik
 *   - espnow_receiver_read()    Ambil paket terbaru
 *   - espnow_receiver_is_link_alive()  Cek status link
 * =====================================================================
 */

#include "config.h"

namespace {
portMUX_TYPE espNowPacketMux = portMUX_INITIALIZER_UNLOCKED;

// =====================================================================
//  STATISTIK — identik dengan esp32s3_master
// =====================================================================

struct EspNowRxStats {
    volatile uint32_t any = 0;
    volatile uint32_t accepted = 0;
    volatile uint32_t rejectedMac = 0;
    volatile uint32_t rejectedLength = 0;
    volatile uint32_t rejectedMagic = 0;
    volatile uint32_t rejectedChecksum = 0;
    volatile uint32_t rejectedSequence = 0;
};

// =====================================================================
//  STATE RECEIVER — identik dengan esp32s3_master
// =====================================================================

struct EspNowReceiverState {
    ControlPacket latestPacket = {};
    bool packetAvailable = false;
    bool isReady = false;
    bool sequenceInitialized = false;
    uint16_t lastSequence = 0;
    uint32_t lastPacketRxMs = 0;
    uint32_t lastStatsPrintMs = 0;
    uint8_t lastConnected = 0;       // Untuk deteksi reconnect (0→1)
    uint8_t staMac[6] = {0};
    EspNowRxStats stats;
};

EspNowReceiverState gEspNow;

// =====================================================================
//  HELPERS — identik dengan esp32s3_master
// =====================================================================

bool macEquals(const uint8_t *left, const uint8_t *right) {
    return memcmp(left, right, 6) == 0;
}

void printMac(const char *label, const uint8_t *mac) {
    Serial.printf("%s%02X:%02X:%02X:%02X:%02X:%02X\n",
                  label,
                  mac[0], mac[1], mac[2],
                  mac[3], mac[4], mac[5]);
}

bool isAllowedSender(const uint8_t *macAddr) {
    if (!espNowEnableMacWhitelist) {
        return true;
    }
    return macEquals(macAddr, espNowAllowedTransmitterStaMac) ||
           macEquals(macAddr, espNowAllowedTransmitterApMac);
}

bool isNewSequence(uint16_t sequence) {
    if (!gEspNow.sequenceInitialized) {
        return true;
    }
    const uint16_t delta = (uint16_t)(sequence - gEspNow.lastSequence);
    return delta > 0 && delta < 32768;
}

// =====================================================================
//  CRITICAL SECTION — identik dengan esp32s3_master
// =====================================================================

void updatePacketFromIsr(const ControlPacket &packet) {
    portENTER_CRITICAL(&espNowPacketMux);
    gEspNow.latestPacket = packet;
    gEspNow.packetAvailable = true;
    portEXIT_CRITICAL(&espNowPacketMux);
}

bool fetchPacket(ControlPacket &outPacket) {
    bool hasPacket = false;

    portENTER_CRITICAL(&espNowPacketMux);
    if (gEspNow.packetAvailable) {
        const uint32_t nowMs = millis();
        const uint32_t gapMs = nowMs - gEspNow.lastPacketRxMs;
        const bool gapTooLong = gapMs > espNowLinkAliveMs;
        gEspNow.lastPacketRxMs = nowMs;

        // --- DETEKSI RECONNECT ---
        // Saat controller reconnect: connected berubah dari 0 → 1.
        // Reset sequence tracker agar paket pertama langsung diterima.
        if (gEspNow.latestPacket.connected && !gEspNow.lastConnected) {
            gEspNow.sequenceInitialized = false;
            Serial.println("[FETCH] Reconnect terdeteksi (connected 0→1), sequence direset");
        }
        gEspNow.lastConnected = gEspNow.latestPacket.connected;

        if (gapTooLong) {
            // Gap terlalu lama — terima paket apapun (reset sequence)
            gEspNow.lastSequence = gEspNow.latestPacket.seq;
            gEspNow.sequenceInitialized = true;
            outPacket = gEspNow.latestPacket;
            hasPacket = true;
        } else if (isNewSequence(gEspNow.latestPacket.seq)) {
            outPacket = gEspNow.latestPacket;
            gEspNow.lastSequence = gEspNow.latestPacket.seq;
            gEspNow.sequenceInitialized = true;
            hasPacket = true;
        } else {
            gEspNow.stats.rejectedSequence++;
        }
        gEspNow.packetAvailable = false;
    }
    portEXIT_CRITICAL(&espNowPacketMux);

    return hasPacket;
}

// =====================================================================
//  PEER & WIFI — identik dengan esp32s3_master
// =====================================================================

bool ensurePeer(const uint8_t *peerMac, const char *label) {
    if (esp_now_is_peer_exist(peerMac)) {
        return true;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerMac, 6);
    peerInfo.channel = espNowChannel;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    const esp_err_t err = esp_now_add_peer(&peerInfo);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        Serial.printf("esp_now_add_peer %s err=%d\n", label, (int)err);
        return false;
    }
    return true;
}

bool configureWifiChannel() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, false);

    esp_wifi_set_promiscuous(true);
    const esp_err_t channelErr = esp_wifi_set_channel(espNowChannel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (channelErr != ESP_OK) {
        Serial.printf("esp_wifi_set_channel err=%d\n", (int)channelErr);
        return false;
    }
    return true;
}

// =====================================================================
//  STATISTIK — identik dengan esp32s3_master
// =====================================================================

void printStatsIfDue() {
    const uint32_t nowMs = millis();
    if (nowMs - gEspNow.lastStatsPrintMs < espNowStatsIntervalMs) {
        return;
    }

    gEspNow.lastStatsPrintMs = nowMs;
    const bool linkAlive = (nowMs - gEspNow.lastPacketRxMs) <= espNowLinkAliveMs;

    Serial.printf(
        "[STAT] jalur=ESPNOW ps4=%-5s seq=%u btn=0x%08lX lx=%4d ly=%4d rx=%4d ry=%4d l2=%3u r2=%3u | "
        "any=%lu ok=%lu rejMac=%lu rejLen=%lu rejMagic=%lu rejCk=%lu rejSeq=%lu link=%s\n",
        gEspNow.latestPacket.connected ? "YES" : "NO",
        gEspNow.latestPacket.seq,
        (unsigned long)gEspNow.latestPacket.buttons,
        gEspNow.latestPacket.lx, gEspNow.latestPacket.ly,
        gEspNow.latestPacket.rx, gEspNow.latestPacket.ry,
        gEspNow.latestPacket.l2Value, gEspNow.latestPacket.r2Value,
        (unsigned long)gEspNow.stats.any,
        (unsigned long)gEspNow.stats.accepted,
        (unsigned long)gEspNow.stats.rejectedMac,
        (unsigned long)gEspNow.stats.rejectedLength,
        (unsigned long)gEspNow.stats.rejectedMagic,
        (unsigned long)gEspNow.stats.rejectedChecksum,
        (unsigned long)gEspNow.stats.rejectedSequence,
        linkAlive ? "OK" : "TIMEOUT"
    );
}

// =====================================================================
//  CALLBACK: ESP-NOW RECEIVE — +checksum validation
// =====================================================================

void onEspNowReceive(const uint8_t *macAddr, const uint8_t *data, int dataLen) {
    gEspNow.stats.any++;

    if (macAddr == nullptr || data == nullptr) {
        gEspNow.stats.rejectedLength++;
        return;
    }

    // --- MAC whitelist ---
    if (!isAllowedSender(macAddr)) {
        gEspNow.stats.rejectedMac++;
        return;
    }

    // --- Length check ---
    if (dataLen != (int)sizeof(ControlPacket)) {
        gEspNow.stats.rejectedLength++;
        return;
    }

    ControlPacket incoming = {};
    memcpy(&incoming, data, sizeof(ControlPacket));

    // --- Magic check ---
    if (incoming.magic != ESPNOW_PACKET_MAGIC) {
        gEspNow.stats.rejectedMagic++;
        return;
    }

    // --- Checksum check (tambahan dari esp32s3_master) ---
    if (hitung_checksum(incoming) != incoming.checksum) {
        gEspNow.stats.rejectedChecksum++;
        return;
    }

    // --- Simpan via critical section ---
    updatePacketFromIsr(incoming);
    gEspNow.stats.accepted++;
}

}  // namespace

// =====================================================================
//  API PUBLIK
// =====================================================================

bool espNowReceiverInit() {
    memset(&gEspNow, 0, sizeof(gEspNow));

    // Tampilkan MAC
    if (esp_read_mac(gEspNow.staMac, ESP_MAC_WIFI_STA) == ESP_OK) {
        printMac("Receiver STA MAC: ", gEspNow.staMac);
    }
    printMac("Allowed TX STA : ", espNowAllowedTransmitterStaMac);
    printMac("Allowed TX AP  : ", espNowAllowedTransmitterApMac);

    // Warning jika MAC sama
    if (macEquals(gEspNow.staMac, espNowAllowedTransmitterStaMac) ||
        macEquals(gEspNow.staMac, espNowAllowedTransmitterApMac)) {
        Serial.println("WARN: Allowed transmitter MAC sama dengan receiver MAC, cek konfigurasi.");
    }

    // WiFi + channel
    configureWifiChannel();

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] init GAGAL");
        return false;
    }

    // Register callback
    esp_now_register_recv_cb(onEspNowReceive);

    // Register peer (jika whitelist aktif)
    bool peerOk = true;
    if (espNowEnableMacWhitelist) {
        peerOk = ensurePeer(espNowAllowedTransmitterStaMac, "sta");
        peerOk = ensurePeer(espNowAllowedTransmitterApMac, "ap") && peerOk;
    }

    gEspNow.isReady = peerOk;
    gEspNow.lastPacketRxMs = millis();
    gEspNow.lastStatsPrintMs = millis();

    Serial.printf("[ESP-NOW] Receiver siap — channel=%u magic=0x%04X ready=%s\n",
        espNowChannel, ESPNOW_PACKET_MAGIC, gEspNow.isReady ? "true" : "false");

    return gEspNow.isReady;
}

void espNowReceiverTick() {
    if (!gEspNow.isReady) {
        return;
    }
    printStatsIfDue();
}

bool espNowReceiverReadPacket(ControlPacket &outPacket) {
    if (!gEspNow.isReady) {
        return false;
    }
    return fetchPacket(outPacket);
}

bool espNowReceiverIsLinkAlive() {
    if (!gEspNow.isReady) {
        return false;
    }
    return (millis() - gEspNow.lastPacketRxMs) <= espNowLinkAliveMs;
}
