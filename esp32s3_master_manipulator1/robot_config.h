#pragma once

#include <Arduino.h>

// ============================================================
// ESP-NOW Configuration untuk Receiver (Master Manipulator 1)
// ============================================================

// Magic number untuk validasi paket
#define ESPNOW_PACKET_MAGIC  0xA5B4

// Enable MAC whitelist untuk keamanan
const bool espNowEnableMacWhitelist = true;

// MAC Address controller yang diizinkan (GANTI dengan MAC controller Anda)
// Cara mendapatkan MAC: Upload sketch ke controller, buka Serial Monitor
const uint8_t espNowAllowedTransmitterStaMac[6] = {0x58, 0xBF, 0x25, 0x8B, 0xDB, 0x18};
const uint8_t espNowAllowedTransmitterApMac[6] = {0x58, 0xBF, 0x25, 0x8B, 0xDB, 0x19};

// Channel WiFi untuk ESP-NOW (1-13)
const uint8_t espNowChannel = 1;

// Timeout link (ms) - jika tidak ada paket dalam waktu ini, link dianggap mati
const unsigned long espNowLinkAliveMs = 180;

// Interval print statistik (ms)
const unsigned long espNowStatsIntervalMs = 1000;

// ============================================================
// Struktur Data Paket Kontrol dari Controller
// ============================================================
// Struktur ini HARUS sama dengan yang ada di controller/transmitter

typedef struct {
	uint16_t magic;       // Magic number untuk validasi (0xA5B4)
	int16_t x;            // Joystick kiri X (-127 to 127)
	int16_t y;            // Joystick kiri Y (-127 to 127)
	int16_t w;            // Joystick kanan X untuk rotasi (-127 to 127)
	int8_t lx;            // Joystick kiri X (analog)
	int8_t ly;            // Joystick kiri Y (analog)
	int8_t rx;            // Joystick kanan X (analog)
	int8_t ry;            // Joystick kanan Y (analog)
	uint8_t l2Value;      // Trigger L2 (0-255)
	uint8_t r2Value;      // Trigger R2 (0-255)
	int16_t gyrX;         // Gyroscope X
	int16_t gyrY;         // Gyroscope Y
	int16_t gyrZ;         // Gyroscope Z
	uint32_t buttons;     // Button states (bitmask)
	uint16_t seq;         // Sequence number untuk deteksi packet loss
	uint8_t connected;    // Status koneksi controller (0=disconnect, 1=connected)
} EspNowControlPacket;

// ============================================================
// Button Bitmask Definitions
// ============================================================
// Gunakan ini untuk cek button yang ditekan
// Contoh: if (packet.buttons & BTN_CROSS) { ... }

#define BTN_CROSS      (1 << 0)   // Button X / Cross
#define BTN_CIRCLE     (1 << 1)   // Button O / Circle
#define BTN_SQUARE     (1 << 2)   // Button Square
#define BTN_TRIANGLE   (1 << 3)   // Button Triangle
#define BTN_L1         (1 << 4)   // Button L1
#define BTN_R1         (1 << 5)   // Button R1
#define BTN_L2         (1 << 6)   // Button L2
#define BTN_R2         (1 << 7)   // Button R2
#define BTN_SHARE      (1 << 8)   // Button Share
#define BTN_OPTIONS    (1 << 9)   // Button Options
#define BTN_L3         (1 << 10)  // Button L3 (joystick kiri tekan)
#define BTN_R3         (1 << 11)  // Button R3 (joystick kanan tekan)
#define BTN_PS         (1 << 12)  // Button PS
#define BTN_TOUCHPAD   (1 << 13)  // Touchpad tekan
#define BTN_UP         (1 << 14)  // D-Pad Up
#define BTN_DOWN       (1 << 15)  // D-Pad Down
#define BTN_LEFT       (1 << 16)  // D-Pad Left
#define BTN_RIGHT      (1 << 17)  // D-Pad Right

// ============================================================
// Function Declarations untuk ESP-NOW Control
// ============================================================

// Inisialisasi ESP-NOW receiver
// Return: true jika berhasil, false jika gagal
bool espNowControlInit();

// Tick function untuk update statistik (panggil di loop)
void espNowControlTick();

// Baca paket terbaru dari controller
// Parameter: outPacket - struct untuk menyimpan data paket
// Return: true jika ada paket baru, false jika tidak ada
bool espNowControlReadPacket(EspNowControlPacket &outPacket);

// Cek apakah link dengan controller masih hidup
// Return: true jika masih ada komunikasi, false jika timeout
bool espNowControlIsLinkAlive();

// ============================================================
// Helper Functions untuk Button Check
// ============================================================

// Cek apakah button tertentu ditekan
inline bool isButtonPressed(uint32_t buttons, uint32_t buttonMask) {
	return (buttons & buttonMask) != 0;
}

// Contoh penggunaan:
// if (isButtonPressed(packet.buttons, BTN_CROSS)) {
//     Serial.println("Button X ditekan!");
// }
