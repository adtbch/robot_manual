/*
 * Minimal PS4 connectivity test
 * Tidak ada WiFi, ESP-NOW, WSN-31, atau komponen lain.
 * PS4.begin() memakai MAC master dari Sixaxis Pair Tool.
 */

#include <PS4Controller.h>

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n===========================");
    Serial.println("  PS4 TEST — WITH MAC ARGUMENT");
    Serial.println("===========================");

    PS4.begin("4c:03:4f:e2:ab:0c");  // MAC Current Master dari Sixaxis Pair Tool

    Serial.println("PS4.begin() OK");
    Serial.println("Tekan Share + PS pada stik untuk pairing");
    Serial.println("===========================\n");
}

void loop() {
    if (PS4.isConnected()) {
        static bool prevConnected = false;
        if (!prevConnected) {
            prevConnected = true;
            Serial.println(">>> PS4 CONNECTED! <<<");
        }

        static uint32_t lastPrint = 0;
        if (millis() - lastPrint >= 100) {
            lastPrint = millis();
            Serial.printf("LStick: X=%4d Y=%4d  RStick: X=%4d Y=%4d  Batt=%d%%\n",
                PS4.LStickX(), PS4.LStickY(),
                PS4.RStickX(), PS4.RStickY(),
                PS4.Battery());
        }
    } else {
        static uint32_t lastBlink = 0;
        if (millis() - lastBlink >= 1000) {
            lastBlink = millis();
            Serial.println("Menunggu koneksi stik...");
        }
    }
}
