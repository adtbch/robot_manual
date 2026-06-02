#include "device_info.h"
#include "config.h"

#include <string.h>
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
}  // namespace

bool  isReceiverMacConfiguredAsTxMac() {
	return memcmp(kReceiverMac, kThisTransmitterStaMac, sizeof(kReceiverMac)) == 0;
}

void printEspBluetoothMac() {
	uint8_t btMac[6] = {0};
	if (esp_read_mac(btMac, ESP_MAC_BT) != ESP_OK) {
		Serial.println("Gagal baca Bluetooth MAC ESP");
		return;
	}

	Serial.printf("ESP Bluetooth MAC aktual: %02X:%02X:%02X:%02X:%02X:%02X\n",
		btMac[0],
		btMac[1],
		btMac[2],
		btMac[3],
		btMac[4],
		btMac[5]);
}

void printEspMacSummary() {
	uint8_t staMac[6] = {0};
	uint8_t apMac[6] = {0};
	uint8_t btMac[6] = {0};

	const bool staOk = esp_read_mac(staMac, ESP_MAC_WIFI_STA) == ESP_OK;
	const bool apOk = esp_read_mac(apMac, ESP_MAC_WIFI_SOFTAP) == ESP_OK;
	const bool btOk = esp_read_mac(btMac, ESP_MAC_BT) == ESP_OK;

	Serial.println("=== TX MAC CHECK ===");
	if (staOk) {
		printMacLine("WiFi STA MAC runtime: ", staMac);
	}
	if (apOk) {
		printMacLine("WiFi AP  MAC runtime: ", apMac);
	}
	if (btOk) {
		printMacLine("Bluetooth MAC runtime: ", btMac);
	}
	printMacLine("Config TX STA MAC   : ", kThisTransmitterStaMac);
	printMacLine("Config TX AP  MAC   : ", kThisTransmitterApMac);
	printMacLine("Config RX target MAC: ", kReceiverMac);
}
