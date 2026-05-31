#include <Arduino.h>
#include "config.h"

#include <string.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>

namespace {
void printMacLine(const char *label, const uint8_t *mac) {
	Serial.printf("%s%02X:%02X:%02X:%02X:%02X:%02X\n",
		label,
		mac[0],
		mac[1],
		mac[2],
		mac[3],
		mac[4],
		mac[5]);
}

void printReceiverMacSummary() {
	uint8_t staMac[6] = {0};
	uint8_t apMac[6] = {0};
	uint8_t btMac[6] = {0};

	const bool staOk = esp_read_mac(staMac, ESP_MAC_WIFI_STA) == ESP_OK;
	const bool apOk = esp_read_mac(apMac, ESP_MAC_WIFI_SOFTAP) == ESP_OK;
	const bool btOk = esp_read_mac(btMac, ESP_MAC_BT) == ESP_OK;

	Serial.println("=== RX MAC CHECK ===");
	if (staOk) {
		printMacLine("WiFi STA MAC runtime: ", staMac);
	}
	if (apOk) {
		printMacLine("WiFi AP  MAC runtime: ", apMac);
	}
	if (btOk) {
		printMacLine("Bluetooth MAC runtime: ", btMac);
	}
	printMacLine("Allowed TX STA MAC  : ", kAllowedTransmitterStaMac);
	printMacLine("Allowed TX AP  MAC  : ", kAllowedTransmitterApMac);
	Serial.printf("Packet magic        : 0x%04X\n", kPacketMagic);
}

void printLineIfNonZero(const char *label, const int32_t value, uint8_t &line) {
	if (value == 0 || line >= 8) {
		return;
	}
	oled.setCursor(0, line * 8);
	oled.print(label);
	oled.print(':');
	oled.print(value);
	line++;
}

void printHexLineIfNonZero(const char *label, const uint32_t value, uint8_t &line) {
	if (value == 0 || line >= 8) {
		return;
	}
	oled.setCursor(0, line * 8);
	oled.print(label);
	oled.print(':');
	oled.print("0x");
	oled.print(value, HEX);
	line++;
}

void updateOled(const uint32_t now) {
	if (!oledReady) {
		return;
	}

	if (now - lastOledRefreshMs < kOledRefreshIntervalMs) {
		return;
	}
	lastOledRefreshMs = now;

	oled.clearDisplay();
	oled.setTextSize(1);
	oled.setTextColor(SSD1306_WHITE);
	oled.setCursor(0, 0);
	oled.print("ESPNOW:");
	if (!espNowInitReady) {
		oled.print("INIT ERR");
	} else if (oledPacketValid && (now - lastPacketRxMs <= kEspNowLinkAliveMs)) {
		oled.print("OK");
	} else {
		oled.print("WAIT");
	}

	if (oledPacketValid) {
		uint8_t line = 1;
		printLineIfNonZero("x", oledPacket.x, line);
		printLineIfNonZero("y", oledPacket.y, line);
		printLineIfNonZero("w", oledPacket.w, line);
		printLineIfNonZero("lx", oledPacket.lx, line);
		printLineIfNonZero("ly", oledPacket.ly, line);
		printLineIfNonZero("rx", oledPacket.rx, line);
		printLineIfNonZero("ry", oledPacket.ry, line);
		printLineIfNonZero("l2", oledPacket.l2Value, line);
		printLineIfNonZero("r2", oledPacket.r2Value, line);
		printLineIfNonZero("gx", oledPacket.gyrX, line);
		printLineIfNonZero("gy", oledPacket.gyrY, line);
		printLineIfNonZero("gz", oledPacket.gyrZ, line);
		printHexLineIfNonZero("btn", oledPacket.buttons, line);
	}

	oled.display();
}

void onEspNowReceive(const uint8_t *macAddr, const uint8_t *data, int len) {
	rxAnyCount++;
	if (macAddr == nullptr) {
		rxRejectedMacCount++;
		return;
	}

	memcpy(lastRxMac, macAddr, sizeof(lastRxMac));
	lastRxMacValid = true;

	// Lapis 1: Strict MAC whitelist — hanya dari transmitter yang terdaftar.
	const bool fromAllowed =
		memcmp(macAddr, kAllowedTransmitterStaMac, 6) == 0 ||
		memcmp(macAddr, kAllowedTransmitterApMac, 6) == 0;
	if (!fromAllowed) {
		rxRejectedMacCount++;
		return;
	}

	// Lapis 2: Validasi ukuran paket.
	if (len != static_cast<int>(sizeof(ControlPacket))) {
		rxRejectedLenCount++;
		return;
	}

	// Lapis 3: Validasi magic number.
	ControlPacket incoming = {};
	memcpy(&incoming, data, sizeof(ControlPacket));
	if (incoming.magic != kPacketMagic) {
		rxRejectedMagicCount++;
		return;
	}

	rxAcceptedCount++;

	portENTER_CRITICAL(&packetMux);
	latestPacket = incoming;
	packetAvailable = true;
	portEXIT_CRITICAL(&packetMux);
}

bool ensureReceiverPeer(const uint8_t *peerMac, const char *name) {
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

bool initEspNowReceiver() {
	WiFi.mode(WIFI_STA);
	WiFi.disconnect(false, false);
	WiFi.setSleep(false);
	esp_wifi_set_promiscuous(true);
	const esp_err_t chErr = esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);
	esp_wifi_set_promiscuous(false);
	if (chErr != ESP_OK) {
		Serial.printf("esp_wifi_set_channel err=%d\n", static_cast<int>(chErr));
	}
	if (esp_now_init() != ESP_OK) {
		return false;
	}
	esp_now_register_recv_cb(onEspNowReceive);

	const bool staPeerOk = ensureReceiverPeer(kAllowedTransmitterStaMac, "tx_sta");
	const bool apPeerOk = ensureReceiverPeer(kAllowedTransmitterApMac, "tx_ap");
	return staPeerOk || apPeerOk;
}

bool fetchPacket(ControlPacket &out) {
	bool hasPacket = false;
	portENTER_CRITICAL(&packetMux);
	if (packetAvailable) {
		const uint16_t seq = latestPacket.seq;
		const uint16_t delta = static_cast<uint16_t>(seq - lastSeq);
		if (!seqInitialized || (delta > 0 && delta < 32768)) {
			out = latestPacket;
			lastSeq = seq;
			seqInitialized = true;
			hasPacket = true;
		}
		packetAvailable = false;
	}
	portEXIT_CRITICAL(&packetMux);
	return hasPacket;
}
}  // namespace

void setup() {
	Serial.begin(115200);
	printReceiverMacSummary();

	// Gaya sederhana: setiap pin PWM di-attach langsung (Arduino LEDC API).
	const bool pwmReady = initAllMotorsPwm();
	espNowInitReady = initEspNowReceiver();

	Wire.begin(kOledSdaPin, kOledSclPin);
	Wire.setClock(400000);
	oledReady = oled.begin(SSD1306_SWITCHCAPVCC, kOledI2cAddress);
	if (oledReady) {
		oled.clearDisplay();
		oled.setTextSize(1);
		oled.setTextColor(SSD1306_WHITE);
		oled.setCursor(0, 0);
		oled.println("OLED ready");
		oled.println("ESPNOW: WAIT");
		oled.display();
	}

	stopAllMotors();

	lastCommandTimeMs = millis();
	if (!pwmReady) {
		Serial.println("LEDC init error: cek pin/channel config");
	}
	if (!espNowInitReady) {
		Serial.println("ESP-NOW init error");
	}
	if (!oledReady) {
		Serial.println("OLED init error");
	}
	Serial.print("Receiver WiFi MAC: ");
	Serial.println(WiFi.macAddress());
	Serial.printf("Receiver WiFi CH: %u\n", WiFi.channel());
	Serial.println("Receiver ready via ESP-NOW");
}

void loop() {
	const uint32_t now = millis();

	ControlPacket packet = {};
	if (fetchPacket(packet)) {
		float x = clampCommand(packet.x);
		float y = clampCommand(packet.y);
		float w = clampCommand(packet.w);

		if ((packet.buttons & kBtnLeftMask) != 0u) {
			x = 0.0f;
			y = -kPwmMax;
			w = 0.0f;
		}

		if (packet.connected) {
			moveXYW(x, y, w);
		} else {
			stopAllMotors();
		}
		lastCommandTimeMs = now;
		lastPacketRxMs = now;
		oledPacket = packet;
		oledPacketValid = true;
	}

	if (now - lastCommandTimeMs > kCommandTimeoutMs) {
		stopAllMotors();
	}

	if (now - lastRxStatsMs >= 1000) {
		lastRxStatsMs = now;
		Serial.printf("RX stat any=%lu ok=%lu rejMac=%lu rejLen=%lu rejMagic=%lu link=%s\n",
			static_cast<unsigned long>(rxAnyCount),
			static_cast<unsigned long>(rxAcceptedCount),
			static_cast<unsigned long>(rxRejectedMacCount),
			static_cast<unsigned long>(rxRejectedLenCount),
			static_cast<unsigned long>(rxRejectedMagicCount),
			(oledPacketValid && (now - lastPacketRxMs <= kEspNowLinkAliveMs)) ? "OK" : "WAIT");
		if (lastRxMacValid) {
			Serial.printf("RX last src: %02X:%02X:%02X:%02X:%02X:%02X\n",
				lastRxMac[0],
				lastRxMac[1],
				lastRxMac[2],
				lastRxMac[3],
				lastRxMac[4],
				lastRxMac[5]);
		}
	}

	updateOled(now);
}