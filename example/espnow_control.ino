#include "robot_config.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>

namespace {
portMUX_TYPE espNowPacketMux = portMUX_INITIALIZER_UNLOCKED;

struct EspNowRxStats {
  volatile uint32_t any = 0;
  volatile uint32_t accepted = 0;
  volatile uint32_t rejectedMac = 0;
  volatile uint32_t rejectedLength = 0;
  volatile uint32_t rejectedMagic = 0;
  volatile uint32_t rejectedSequence = 0;
};

struct EspNowReceiverState {
  EspNowControlPacket latestPacket = {};
  bool packetAvailable = false;

  bool isReady = false;
  bool sequenceInitialized = false;
  uint16_t lastSequence = 0;

  uint32_t lastPacketRxMs = 0;
  uint32_t lastStatsPrintMs = 0;

  uint8_t staMac[6] = {0};
  EspNowRxStats stats;
};

EspNowReceiverState gEspNow;

bool macEquals(const uint8_t *left, const uint8_t *right) {
  return memcmp(left, right, 6) == 0;
}

void printMac(const char *label, const uint8_t *mac) {
  Serial.printf("%s%02X:%02X:%02X:%02X:%02X:%02X\n",
                label,
                mac[0],
                mac[1],
                mac[2],
                mac[3],
                mac[4],
                mac[5]);
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

void updatePacketFromIsr(const EspNowControlPacket &packet) {
  portENTER_CRITICAL(&espNowPacketMux);
  gEspNow.latestPacket = packet;
  gEspNow.packetAvailable = true;
  portEXIT_CRITICAL(&espNowPacketMux);
}

bool fetchPacket(EspNowControlPacket &outPacket) {
  bool hasPacket = false;

  portENTER_CRITICAL(&espNowPacketMux);
  if (gEspNow.packetAvailable) {
    if (isNewSequence(gEspNow.latestPacket.seq)) {
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

void printStatsIfDue() {
  const uint32_t nowMs = millis();
  if (nowMs - gEspNow.lastStatsPrintMs < espNowStatsIntervalMs) {
    return;
  }

  gEspNow.lastStatsPrintMs = nowMs;
  const bool linkAlive = (nowMs - gEspNow.lastPacketRxMs) <= espNowLinkAliveMs;

  Serial.printf("ESPNOW RX any=%lu ok=%lu rejMac=%lu rejLen=%lu rejMagic=%lu rejSeq=%lu link=%s\n",
                (unsigned long)gEspNow.stats.any,
                (unsigned long)gEspNow.stats.accepted,
                (unsigned long)gEspNow.stats.rejectedMac,
                (unsigned long)gEspNow.stats.rejectedLength,
                (unsigned long)gEspNow.stats.rejectedMagic,
                (unsigned long)gEspNow.stats.rejectedSequence,
                linkAlive ? "OK" : "TIMEOUT");
}

void onEspNowReceive(const uint8_t *macAddr, const uint8_t *data, int dataLen) {
  gEspNow.stats.any++;

  if (macAddr == nullptr || data == nullptr) {
    gEspNow.stats.rejectedLength++;
    return;
  }

  if (!isAllowedSender(macAddr)) {
    gEspNow.stats.rejectedMac++;
    return;
  }

  if (dataLen != (int)sizeof(EspNowControlPacket)) {
    gEspNow.stats.rejectedLength++;
    return;
  }

  EspNowControlPacket incoming = {};
  memcpy(&incoming, data, sizeof(EspNowControlPacket));

  if (incoming.magic != ESPNOW_PACKET_MAGIC) {
    gEspNow.stats.rejectedMagic++;
    return;
  }

  updatePacketFromIsr(incoming);
  gEspNow.stats.accepted++;
}
}  // namespace

bool espNowControlInit() {
  memset(&gEspNow, 0, sizeof(gEspNow));

  if (esp_read_mac(gEspNow.staMac, ESP_MAC_WIFI_STA) == ESP_OK) {
    printMac("Receiver STA MAC: ", gEspNow.staMac);
  }
  printMac("Allowed TX STA : ", espNowAllowedTransmitterStaMac);
  printMac("Allowed TX AP  : ", espNowAllowedTransmitterApMac);

  if (macEquals(gEspNow.staMac, espNowAllowedTransmitterStaMac) ||
      macEquals(gEspNow.staMac, espNowAllowedTransmitterApMac)) {
    Serial.println("WARN: Allowed transmitter MAC sama dengan receiver MAC, cek konfigurasi.");
  }

  configureWifiChannel();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init gagal");
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
  gEspNow.lastStatsPrintMs = millis();

  Serial.printf("ESP-NOW ready=%s channel=%u\n", gEspNow.isReady ? "true" : "false", espNowChannel);
  return gEspNow.isReady;
}

void espNowControlTick() {
  if (!gEspNow.isReady) {
    return;
  }

  printStatsIfDue();
}

bool espNowControlReadPacket(EspNowControlPacket &outPacket) {
  if (!gEspNow.isReady) {
    return false;
  }

  const bool hasPacket = fetchPacket(outPacket);
  if (hasPacket) {
    gEspNow.lastPacketRxMs = millis();
  }
  return hasPacket;
}

bool espNowControlIsLinkAlive() {
  if (!gEspNow.isReady) {
    return false;
  }
  return (millis() - gEspNow.lastPacketRxMs) <= espNowLinkAliveMs;
}
