#include "config.h"
#include "device_info.h"
#include "espnow_transport.h"
#include "ps4_input.h"
#include "transmitter_state.h"

#include <PS4Controller.h>
#include <esp_system.h>

namespace {
void printResetReason() {
	const esp_reset_reason_t reason = esp_reset_reason();
	Serial.print("Reset reason: ");
	Serial.println(static_cast<int>(reason));
	if (reason == ESP_RST_BROWNOUT) {
		Serial.println("Warning: brownout terdeteksi (cek supply USB/5V)");
	}
}
}  // namespace

void setup() {
	Serial.begin(115200);
	printResetReason();

	const bool ps4BeginOk = PS4.begin(kPs4HostAddress);
	if (!ps4BeginOk) {
		Serial.println("PS4.begin gagal (cek host address / library)");
	}
	Serial.print("PS4 host address: ");
	Serial.println(kPs4HostAddress);
	printEspMacSummary();
	receiverMacValid = !isReceiverMacConfiguredAsTxMac();
	if (!receiverMacValid) {
		Serial.println("ERROR: kReceiverMac masih sama dengan MAC transmitter");
		Serial.println("ESP-NOW dinonaktifkan sampai MAC receiver diperbaiki");
	}
	Serial.println("Pair stik: tahan Share + PS");
	Serial.println("MODE: PS4 buttons+joystick+gyro -> ESP-NOW low latency");
	Serial.printf("ESP-NOW send mode: %s\n", kEspNowUseBroadcastOnly ? "BROADCAST" : "UNICAST/FALLBACK");

}

void loop() {
	const uint32_t now = millis();
	const bool connected = PS4.isConnected();
	callbackConnected = connected;
	if (connected != lastConnected) {
		Serial.println(connected ? "PS4 connected" : "PS4 disconnected");
		lastConnected = connected;
		if (connected) {
			ps4ConnectedAtMs = now;
			lastEspNowInitAttemptMs = 0;
			espNowInitTried = false;
			resetYaw();  // mulai akumulasi yaw dari 0.
		} else {
			buildPacketFromPs4(txPacket, false);
			if (receiverMacValid) {
				trySendPacket(txPacket);
			}
			ledStateInitialized = false;
			sendBusy = false;
		}
	}

	if (!connected) {
		if (now - lastStatusMs >= 1000) {
			lastStatusMs = now;
			Serial.printf("state conn=%u cb=%u espnow=%u\n",
				connected ? 1 : 0,
				callbackConnected ? 1 : 0,
				espNowReady ? 1 : 0);
		}
		yield();
		return;
	}

	const uint32_t connectedDurationMs = now - ps4ConnectedAtMs;
	if (!espNowReady && receiverMacValid && connectedDurationMs >= 2000 &&
		(lastEspNowInitAttemptMs == 0 || (now - lastEspNowInitAttemptMs >= kEspNowInitRetryMs))) {
		lastEspNowInitAttemptMs = now;
		espNowInitTried = true;
		espNowReady = initEspNow();
		if (espNowReady) {
			Serial.println("ESP-NOW ready");
		} else {
			Serial.println("ESP-NOW init gagal (retry)");
		}
	}

	if (connectedDurationMs < kPs4SettleMs) {
		yield();
		return;
	}

	if (connectedDurationMs >= kPs4SettleMs) {
		updateBatteryLed(false);
	}
	buildPacketFromPs4(txPacket, connected);
	const bool changed = packetChanged(txPacket, lastSentPacket);
	const bool dueKeepAlive = (now - lastSendMs >= kSendKeepAliveMs);
	const bool dueChanged = changed && (now - lastSendMs >= kSendMinIntervalMs);
	if (receiverMacValid && (dueChanged || dueKeepAlive)) {
		trySendPacket(txPacket);
	}

	if (now - lastStatusMs >= 1000) {
		lastStatusMs = now;
		Serial.printf("state conn=%u espnow=%u fail=%u x=%d y=%d w=%d batt=%d%%\n",
			connected ? 1 : 0,
			espNowReady ? 1 : 0,
			espNowFailBurst,
			txPacket.x,
			txPacket.y,
			txPacket.w,
			batteryPercent(PS4.Battery()));
		Serial.printf("  rawAcc x=%d y=%d z=%d  rawGyr x=%d y=%d z=%d\n",
			PS4.AccX(), PS4.AccY(), PS4.AccZ(),
			PS4.GyrX(), PS4.GyrY(), PS4.GyrZ());
		Serial.printf("  mahony roll=%d pitch=%d yaw=%d\n",
			txPacket.gyrX, txPacket.gyrY, txPacket.gyrZ);
	}

	yield();
}