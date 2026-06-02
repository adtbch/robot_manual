#include "espnow_transport.h"
#include "transmitter_state.h"

#include <string.h>
#include <WiFi.h>
#include <esp_wifi.h>

namespace {
const uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

bool isBroadcastMac(const uint8_t *mac) {
	return mac != nullptr && memcmp(mac, kBroadcastMac, sizeof(kBroadcastMac)) == 0;
}

bool ensureTxPeer(const uint8_t *peerMac, const char *name) {
	if (esp_now_is_peer_exist(peerMac)) {
		return true;
	}

	esp_now_peer_info_t peerInfo = {};
	memcpy(peerInfo.peer_addr, peerMac, 6);
	peerInfo.channel = kEspNowChannel;
	peerInfo.encrypt = false;
	peerInfo.ifidx = WIFI_IF_STA;

	const esp_err_t err = esp_now_add_peer(&peerInfo);
	if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
		Serial.printf("esp_now_add_peer %s err=%d\n", name, static_cast<int>(err));
		return false;
	}
	return true;
}
}  // namespace

void onEspNowSent(const uint8_t *macAddr, esp_now_send_status_t status) {
	sendBusy = false;
	if (status != ESP_NOW_SEND_SUCCESS) {
		if (espNowFailBurst < 65535u) {
			espNowFailBurst++;
		}
		if (espNowFailBurst == 1 || (espNowFailBurst % 20u) == 0u) {
			Serial.printf("ESP-NOW send fail (ch=%u burst=%u dst=%s)\n",
				WiFi.channel(),
				espNowFailBurst,
				isBroadcastMac(macAddr) ? "BC" : "UC");
		}
		if (espNowFailBurst >= 40u) {
			espNowReady = false;
		}
	} else {
		if (espNowFailBurst > 0u) {
			espNowFailBurst--;
		}
	}
}

bool initEspNow() {
	WiFi.mode(WIFI_STA);
	WiFi.disconnect(false, false);
	// Wajib untuk coexistence BT Classic + WiFi pada core 2.0.17.
	WiFi.setSleep(true);

	esp_wifi_set_promiscuous(true);
	const esp_err_t chErr = esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);
	esp_wifi_set_promiscuous(false);
	if (chErr != ESP_OK) {
		Serial.printf("esp_wifi_set_channel err=%d\n", static_cast<int>(chErr));
	}

	const esp_err_t initErr = esp_now_init();
	if (initErr != ESP_OK) {
		Serial.printf("esp_now_init err=%d\n", static_cast<int>(initErr));
		return false;
	}

	esp_now_register_send_cb(onEspNowSent);

	bool receiverPeerOk = true;
	if (!kEspNowUseBroadcastOnly) {
		receiverPeerOk = ensureTxPeer(kReceiverMac, "receiver");
	}
	const bool broadcastPeerOk = ensureTxPeer(kBroadcastMac, "broadcast");

	return receiverPeerOk && broadcastPeerOk;
}

bool trySendPacket(ControlPacket &packet) {
	if (!espNowReady || sendBusy) {
		return false;
	}

	const uint8_t *targetMac = kReceiverMac;
	if (kEspNowUseBroadcastOnly) {
		targetMac = kBroadcastMac;
	} else if (kEspNowEnableBroadcastFallback && espNowFailBurst >= kEspNowBroadcastThreshold) {
		targetMac = kBroadcastMac;
	}

	packet.seq = ++txSeq;
	if (esp_now_send(targetMac, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet)) == ESP_OK) {
		sendBusy = true;
		lastSendMs = millis();
		lastSentPacket = packet;
		return true;
	}
	return false;
}
