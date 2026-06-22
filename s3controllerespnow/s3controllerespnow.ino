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

// =====================================================================
//  DEFINISI VARIABEL GLOBAL (extern di config.h)
// =====================================================================

uint16_t nomor_urut_paket = 0;
bool     espnow_siap      = false;

// =====================================================================
//  OBJEK GLOBAL
// =====================================================================

GamepadState gp;
GamepadHost  usb;

// =====================================================================
//  SETUP
// =====================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("MAC Address: " + WiFi.macAddress());
    Serial.println("=== ESP32-S3 USB Gamepad + ESP-NOW ===");

    // 1. ESP-NOW (WiFi) — harus init SEBELUM USB Host
    espnow_siap = espnow_init();

    // 2. USB Host
    usb.begin();

    Serial.println("Setup selesai. Menunggu koneksi gamepad...");
    Serial.println("Pastikan external 5V VBUS terhubung ke pin 5V ESP32-S3.");
}

// =====================================================================
//  LOOP
// =====================================================================

void loop() {
    // Proses event USB Host (enumerate, data, disconnect)
    usb.task();

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
                Serial.printf("[GP] LX:%4d LY:%4d RX:%4d RY:%4d L2:%3d R2:%3d BTN:0x%08lX SEQ:%u\n",
                    gp.lx, gp.ly, gp.rx, gp.ry, gp.l2a, gp.r2a,
                    (unsigned long)paket.buttons, paket.seq);
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
