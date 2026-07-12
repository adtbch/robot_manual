/*
 * =====================================================================
 * FILE    : S3.ino
 * BOARD   : ESP32-S3
 * PERAN   : Entry point — setup() + loop().
 *
 * ARSITEKTUR:
 *   [PS4 DualShock 4]
 *         | USB OTG (native USB Host — EspUsbHost library)
 *         v
 *   [ESP32-S3]
 *         | loop():
 *         |   - usb.task()           proses event USB
 *         |   - isi_paket_dari_gamepad()  GamepadState → ControlPacket
 *         |   - kirim_via_espnow()   WiFi ESP-NOW → receiver
 *         v
 *   [Receiver ESP32-S3]
 *
 * KOMPONEN:
 *   - config.h                 : ControlPacket, MAC, konstanta
 *   - gamepad.h                : PS4 HID parser + GamepadState
 *   - usb_host.h               : GamepadHost (EspUsbHost subclass)
 *   - packet.ino               : checksum, stop packet, fill dari gamepad
 *   - espnow_transmitter.ino   : ESP-NOW init + kirim
 *
 * CATATAN:
 *   - External 5V VBUS power WAJIB untuk USB Host
 *   - WiFi (ESP-NOW) dan USB Host bisa jalan bersama di ESP32-S3
 * =====================================================================
 */

#include "usb_host.h"
#include "config.h"
#include <esp_mac.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

// =====================================================================
//  DEFINISI VARIABEL GLOBAL (extern di config.h)
// =====================================================================

uint8_t  kEspNowChannel    = 12;  // fixed channel 12
uint16_t nomor_urut_paket = 0;
bool     espnow_siap      = false;
uint8_t  gControllerMode  = 1;  // default: otomatis

// =====================================================================
//  OBJEK GLOBAL
// =====================================================================

GamepadState gp;
GamepadHost  usb;
Adafruit_NeoPixel strip(1, kLedPin, NEO_GRB + NEO_KHZ800);

// =====================================================================
//  SETUP
// =====================================================================

void setup() {
    Serial.begin(115200);

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.println("=== ESP32-S3 USB Gamepad + ESP-NOW ===");

    // Boot button — input pullup, active LOW
    pinMode(kBootPin, INPUT_PULLUP);

    // Load last mode from NVS
    Preferences prefs;
    prefs.begin("ctrl", true);
    gControllerMode = prefs.getUChar("mode", 0);
    prefs.end();

    // LED strip init
    strip.begin();
    strip.setBrightness(50);
    updateLedMode();

    // 1. ESP-NOW (WiFi) — harus init SEBELUM USB Host
    espnow_siap = espnow_init();

    // 3. USB Host
    usb.begin();

    Serial.println("Setup selesai. Menunggu koneksi gamepad...");
    Serial.println("Pastikan external 5V VBUS terhubung ke pin 5V ESP32-S3.");
}

// =====================================================================
//  LOOP
// =====================================================================

void loop() {
    // Boot button double-press (non-blocking)
    bootButtonTick();

    // Proses event USB Host (enumerate, data, disconnect)
    usb.task();

    // Disconnect → kirim stop packet lalu restart
    if (usb.needsRestart) {
        ControlPacket stopPaket = {};
        buat_paket_stop(stopPaket);
        kirim_via_espnow(stopPaket);
        Serial.println("Disconnected. Waiting...");
        ESP.restart();
    }

    // Kirim paket pada interval yang teratur
    static uint32_t waktu_kirim_terakhir = 0;
    const uint32_t sekarang = millis();

    if (sekarang - waktu_kirim_terakhir >= kSendIntervalMs) {
        waktu_kirim_terakhir = sekarang;

        ControlPacket paket = {};

        if (gp.connected) {
            // Gamepad aktif → isi dari USB HID data
            isi_paket_dari_gamepad(paket, gp);

            // Debug output (opsional, bisa dihapus untuk production)
            static uint32_t last_print = 0;
            if (sekarang - last_print >= 1000) {
                last_print = sekarang;
                // Serial.printf("[GP] LX:%4d LY:%4d RX:%4d RY:%4d L2:%3d R2:%3d BTN:0x%08lX SEQ:%u\n",
                //     gp.lx, gp.ly, gp.rx, gp.ry, gp.l2a, gp.r2a,
                //     (unsigned long)paket.buttons, paket.seq);
            }
        } else {
            // Gamepad tidak terkirim → paket stop
            buat_paket_stop(paket);
        }

        // Kirim via ESP-NOW
        kirim_via_espnow(paket);
    }

    // // Minimal delay agar task lain bisa jalan
    // delay(1);
}

// =====================================================================
//  BOOT BUTTON — double-press toggle mode
// =====================================================================

void updateLedMode() {
    if (gControllerMode == 1) {
        strip.setPixelColor(0, strip.Color(255, 255, 255));  // putih = auto
    } else {
        strip.setPixelColor(0, strip.Color(165, 42, 42));    // coklat = manual
    }
    strip.show();
}

void bootButtonTick() {
    static uint32_t lastPressMs   = 0;
    static uint32_t lastDebounceMs = 0;
    static bool     lastStable     = HIGH;

    const bool raw = digitalRead(kBootPin);

    // debounce — skip jika < 50ms dari perubahan terakhir
    if (raw != lastStable) {
        lastDebounceMs = millis();
        lastStable     = raw;
        return;
    }
    if ((millis() - lastDebounceMs) < kDebounceMs) return;

    // deteksi falling edge (tekan)
    static bool prevStable = HIGH;
    if (raw == LOW && prevStable == HIGH) {
        const uint32_t now = millis();
        if ((now - lastPressMs) < kDoublePressMs) {
            // double-press terdeteksi → toggle
            gControllerMode = (gControllerMode == 1) ? 0 : 1;
            Preferences prefs;
            prefs.begin("ctrl", false);
            prefs.putUChar("mode", gControllerMode);
            prefs.end();
            updateLedMode();
            Serial.printf("[Boot] mode → %s\n", gControllerMode ? "AUTO" : "MANUAL");
            lastPressMs = 0;  // reset supaya triple-press tidak trigger 2x
        } else {
            lastPressMs = now;
        }
    }
    prevStable = raw;
}
