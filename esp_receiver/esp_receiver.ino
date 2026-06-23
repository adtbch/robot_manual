/*
 * =====================================================================
 * FILE    : esp_receiver.ino
 * BOARD   : ESP32 (penerima)
 * PERAN   : Menerima ControlPacket dari s3controllerespnow via ESP-NOW.
 *           Cetak statistik & decode paket ke Serial Monitor.
 *
 * ARSITEKTUR:
 *   [s3controllerespnow (ESP32-S3)]
 *         | USB OTG → PS4 DualShock 4
 *         | ESP-NOW
 *         v
 *   [ESP32 Receiver — SKETCH INI]
 *         | espNowReceiverTick()   → cetak statistik
 *         | espNowReceiverReadPacket()  → decode paket
 *         v
 *         Serial Monitor (115200 baud)
 *
 * KONSEP:
 *   ESP-NOW receiver diambil dari esp32s3_master (sudah matang).
 *   - MAC whitelist untuk keamanan
 *   - portMUX critical section (ISR-safe)
 *   - Sequence validation (anti duplikat)
 *   - Link alive detection
 *   - Tanpa delay(), tanpa ESP.restart()
 * =====================================================================
 */

#include "config.h"

// =====================================================================
//  SETUP
// =====================================================================

void setup() {
    Serial.begin(115200);

    Serial.println("=== ESP-NOW Receiver ===");
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());

    // Init ESP-NOW receiver (tidak restart jika gagal — tetap jalan)
    bool espNowReady = espNowReceiverInit();
    Serial.printf("ESP-NOW control: %s\n", espNowReady ? "READY" : "ERROR");

    Serial.println("Menunggu paket dari s3controllerespnow...");
    Serial.println("--------------------------------------------------");
}

// =====================================================================
//  LOOP — identik dengan esp32s3_master pattern
// =====================================================================

void loop() {
    // 1. Cetak statistik periodik
    espNowReceiverTick();

    // 2. Ambil paket terbaru
    ControlPacket gLastRxPacket = {};
    if (espNowReceiverReadPacket(gLastRxPacket)) {
        // Decode & tampilkan detail paket
        Serial.printf(
            "[PKT] seq=%u conn=%d | "
            "sticks: LX=%4d LY=%4d RX=%4d RY=%4d | "
            "trig: L2=%3u R2=%3u | "
            "btn: 0x%08lX",
            gLastRxPacket.seq,
            gLastRxPacket.connected,
            gLastRxPacket.lx, gLastRxPacket.ly,
            gLastRxPacket.rx, gLastRxPacket.ry,
            gLastRxPacket.l2Value, gLastRxPacket.r2Value,
            (unsigned long)gLastRxPacket.buttons
        );

        // Decode tombol yang ditekan
        if (gLastRxPacket.buttons) {
            Serial.print(" [");
            if (gLastRxPacket.buttons & BTN_UP)       Serial.print("UP ");
            if (gLastRxPacket.buttons & BTN_DOWN)     Serial.print("DN ");
            if (gLastRxPacket.buttons & BTN_LEFT)     Serial.print("LT ");
            if (gLastRxPacket.buttons & BTN_RIGHT)    Serial.print("RT ");
            if (gLastRxPacket.buttons & BTN_CROSS)    Serial.print("X ");
            if (gLastRxPacket.buttons & BTN_CIRCLE)   Serial.print("O ");
            if (gLastRxPacket.buttons & BTN_TRIANGLE) Serial.print("T ");
            if (gLastRxPacket.buttons & BTN_SQUARE)   Serial.print("S ");
            if (gLastRxPacket.buttons & BTN_L1)       Serial.print("L1 ");
            if (gLastRxPacket.buttons & BTN_R1)       Serial.print("R1 ");
            if (gLastRxPacket.buttons & BTN_L2)       Serial.print("L2d ");
            if (gLastRxPacket.buttons & BTN_R2)       Serial.print("R2d ");
            if (gLastRxPacket.buttons & BTN_L3)       Serial.print("L3 ");
            if (gLastRxPacket.buttons & BTN_R3)       Serial.print("R3 ");
            if (gLastRxPacket.buttons & BTN_SHARE)    Serial.print("SH ");
            if (gLastRxPacket.buttons & BTN_OPTIONS)  Serial.print("OP ");
            if (gLastRxPacket.buttons & BTN_PS)       Serial.print("PS ");
            if (gLastRxPacket.buttons & BTN_TOUCHPAD) Serial.print("TP ");
            Serial.print("]");
        }

        Serial.println();
    }
}
