/*
 * =====================================================================
 * FILE    : espnow.ino
 * PERAN   : Komunikasi ESP-NOW receiver — menerima ControlPacket dari
 *           s3controllerespnow (ESP32-S3 USB Host + PS4 gamepad).
 *
 * ISI FILE INI:
 *   - espNowControlInit()          Inisialisasi WiFi STA + ESP-NOW + peer
 *   - espNowControlTick()          Cetak statistik periodik
 *   - espNowControlReadPacket()    Ambil paket terbaru (non-blocking)
 *   - espNowControlPrintPacket()   Cetak detail paket ke Serial
 *
 * KONSEP:
 *   Diambil dari esp32s3_master/espnow_control.ino (sudah matang).
 *   - portMUX untuk critical section (ISR-safe)
 *   - MAC whitelist untuk keamanan
 *   - Sequence number validation (anti duplikat)
 *   - Checksum validation (XOR, sama dengan s3controllerespnow)
 *   - Statistik lengkap
 *   - Reconnect detection (connected 0→1 reset sequence)
 *
 * ALIRAN DATA:
 *   [s3controllerespnow (ESP32-S3)]
 *         | ESP-NOW
 *         v
 *   [ESP32-S3 Master — SKETCH INI]
 *         | espNowControlTick()           → cetak statistik
 *         | espNowControlReadPacket()     → ambil paket
 *         v
 *         mecanum_control / gripper / armbox
 *
 * CATATAN:
 *   ControlPacket struct HARUS IDENTIK dengan s3controllerespnow/config.h
 *   Magic = 0xA5B4, Checksum = XOR byte dari field x sampai connected.
 *
 * ORDERING:
 *   - Public API functions: declared in config.h, order doesn't matter
 *   - Anonymous namespace functions: MUST be ordered by dependency
 *     (helper before caller, no forward declaration in anonymous ns)
 * =====================================================================
 */

#include "espnow.h"

namespace {

// =====================================================================
//  1. STATE
// =====================================================================

portMUX_TYPE espNowPacketMux = portMUX_INITIALIZER_UNLOCKED;

struct EspNowRxStats {
    volatile uint32_t any = 0;
    volatile uint32_t accepted = 0;
    volatile uint32_t rejectedMac = 0;
    volatile uint32_t rejectedLength = 0;
    volatile uint32_t rejectedMagic = 0;
    volatile uint32_t rejectedChecksum = 0;
    volatile uint32_t rejectedSequence = 0;
};

struct EspNowReceiverState {
    ControlPacket latestPacket = {};
    bool packetAvailable = false;
    bool isReady = false;
    bool sequenceInitialized = false;
    uint16_t lastSequence = 0;
    uint32_t lastPacketRxMs = 0;
    uint8_t lastConnected = 0;
    uint8_t staMac[6] = {0};
    EspNowRxStats stats;
};

EspNowReceiverState gEspNow;
Jeda jedaStats;

// =====================================================================
//  2. SIMPLE HELPERS (no internal deps)
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

static uint8_t hitungChecksum(const ControlPacket &p) {
    const uint8_t *raw = reinterpret_cast<const uint8_t *>(&p);
    const size_t start = offsetof(ControlPacket, x);
    const size_t end   = sizeof(ControlPacket);

    uint8_t cs = 0;
    for (size_t i = start; i < end; i++) {
        cs ^= raw[i];
    }
    return cs;
}

// =====================================================================
//  3. HELPERS (use simple helpers)
// =====================================================================

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

void updatePacketFromIsr(const ControlPacket &packet) {
    portENTER_CRITICAL(&espNowPacketMux);
    gEspNow.latestPacket = packet;
    gEspNow.packetAvailable = true;
    portEXIT_CRITICAL(&espNowPacketMux);
}

bool configureWifiChannel() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, false);

    esp_wifi_set_promiscuous(true);
    const esp_err_t channelErr = esp_wifi_set_channel(espNowChannel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (channelErr != ESP_OK) {
        Serial.printf("[ESPNOW] Gagal set channel: err=%d\n", (int)channelErr);
        return false;
    }
    return true;
}

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
        Serial.printf("[ESPNOW] Gagal daftarkan peer %s: err=%d\n", label, (int)err);
        return false;
    }
    return true;
}

// =====================================================================
//  4. CORE LOGIC (use helpers)
// =====================================================================

bool fetchPacket(ControlPacket &outPacket) {
    bool hasPacket = false;

    portENTER_CRITICAL(&espNowPacketMux);
    if (gEspNow.packetAvailable) {
        const uint32_t nowMs = millis();
        const uint32_t gapMs = nowMs - gEspNow.lastPacketRxMs;
        const bool gapTooLong = gapMs > espNowLinkAliveMs;
        gEspNow.lastPacketRxMs = nowMs;

        if (gEspNow.latestPacket.connected && !gEspNow.lastConnected) {
            gEspNow.sequenceInitialized = false;
            Serial.println("[ESPNOW] Reconnect terdeteksi (connected 0→1), sequence direset");
        }
        gEspNow.lastConnected = gEspNow.latestPacket.connected;

        if (gapTooLong) {
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

void printStatsIfDue() {
    if (!jedaStats.check(espNowStatsIntervalMs)) {
        return;
    }

    const uint32_t nowMs = millis();
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
//  5. CALLBACK (use all helpers above)
// =====================================================================

void onEspNowReceive(const esp_now_recv_info *info, const uint8_t *data, int dataLen) {
    gEspNow.stats.any++;

    if (info == nullptr || data == nullptr) {
        gEspNow.stats.rejectedLength++;
        return;
    }

    const uint8_t *macAddr = info->src_addr;

    if (!isAllowedSender(macAddr)) {
        gEspNow.stats.rejectedMac++;
        return;
    }

    if (dataLen != (int)sizeof(ControlPacket)) {
        gEspNow.stats.rejectedLength++;
        return;
    }

    ControlPacket incoming = {};
    memcpy(&incoming, data, sizeof(ControlPacket));

    if (incoming.magic != ESPNOW_PACKET_MAGIC) {
        gEspNow.stats.rejectedMagic++;
        return;
    }

    if (hitungChecksum(incoming) != incoming.checksum) {
        gEspNow.stats.rejectedChecksum++;
        return;
    }

    updatePacketFromIsr(incoming);
    gEspNow.stats.accepted++;
}

}  // namespace

// =====================================================================
//  PUBLIC API (declared in config.h, order doesn't matter)
// =====================================================================

bool espNowControlInit() {
    memset(&gEspNow, 0, sizeof(gEspNow));

    if (esp_read_mac(gEspNow.staMac, ESP_MAC_WIFI_STA) == ESP_OK) {
        printMac("Receiver STA MAC: ", gEspNow.staMac);
    }
    printMac("Allowed TX STA : ", espNowAllowedTransmitterStaMac);
    printMac("Allowed TX AP  : ", espNowAllowedTransmitterApMac);

    if (macEquals(gEspNow.staMac, espNowAllowedTransmitterStaMac) ||
        macEquals(gEspNow.staMac, espNowAllowedTransmitterApMac)) {
        Serial.println("[ESPNOW] WARN: Allowed transmitter MAC sama dengan receiver MAC, cek konfigurasi.");
    }

    if (!configureWifiChannel()) {
        return false;
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNOW] esp_now_init GAGAL");
        return false;
    }

    esp_now_register_recv_cb(onEspNowReceive);

    bool peerOk = true;
    if (espNowEnableMacWhitelist) {
        peerOk = ensurePeer(espNowAllowedTransmitterStaMac, "sta");
        peerOk = ensurePeer(espNowAllowedTransmitterApMac, "ap") && peerOk;
    }

    gEspNow.isReady = peerOk;
    gEspNow.lastPacketRxMs = millis();
    jedaStats.reset();

    Serial.printf("[ESPNOW] Init OK — channel=%u magic=0x%04X ready=%s\n",
        espNowChannel, ESPNOW_PACKET_MAGIC, gEspNow.isReady ? "true" : "false");

    return gEspNow.isReady;
}

void espNowControlTick() {
    if (!gEspNow.isReady) {
        return;
    }
    printStatsIfDue();
}

bool espNowControlReadPacket(ControlPacket &outPacket) {
    if (!gEspNow.isReady) {
        return false;
    }
    return fetchPacket(outPacket);
}

bool espNowControlIsLinkAlive() {
    if (!gEspNow.isReady) {
        return false;
    }
    return (millis() - gEspNow.lastPacketRxMs) <= espNowLinkAliveMs;
}
