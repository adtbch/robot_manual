/*
 * =====================================================================
 * FILE    : ota.ino
 * PERAN   : Driver WiFi AP + ArduinoOTA.
 *           Mengamankan motor ketika proses upload dimulai.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#include "ota.h"
#include "motor.h"
#include <WiFi.h>
#include <ArduinoOTA.h>

namespace {

bool otaUploading = false;

} // anonymous namespace

void setupOTA() {
    Serial.println("Setting up WiFi AP...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(OTA_AP_SSID, OTA_AP_PASS);
    
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    ArduinoOTA.setHostname(OTA_HOSTNAME);

    ArduinoOTA.onStart([]() {
        otaUploading = true;
        motorStopAll();  // MATIKAN MOTOR demi keamanan saat flash di-write
        Serial.println("[OTA] Upload starting... MOTORS STOPPED.");
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\n[OTA] Upload complete. Rebooting...");
        otaUploading = false;
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static uint32_t lastPrint = 0;
        uint32_t now = millis();
        if (now - lastPrint >= 500) {  // Limit print biar tidak terlalu cepat
            lastPrint = now;
            Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
        }
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        otaUploading = false;
    });

    ArduinoOTA.begin();
    Serial.println("OTA: READY");
}

void handleOTA() {
    ArduinoOTA.handle();
}

bool isOtaUploading() {
    return otaUploading;
}
