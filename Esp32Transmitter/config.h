#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// MAC milik board transmitter ini (referensi):
constexpr uint8_t kThisTransmitterStaMac[6] = {0x58, 0xBF, 0x25, 0x8B, 0xDB, 0x18};
constexpr uint8_t kThisTransmitterApMac[6] = {0x58, 0xBF, 0x25, 0x8B, 0xDB, 0x19};

// Isi dengan MAC WiFi STA ESP receiver (tujuan ESP-NOW), bukan MAC transmitter.
constexpr uint8_t kReceiverMac[6] = {0x80, 0xB5, 0x4E, 0xC1, 0xC1, 0x98};

// MAC Bluetooth lokal ESP transmitter untuk pairing stik PS4.
// Saat pertama pair: tekan Share + PS pada stik.
constexpr char kPs4HostAddress[] = "4c:11:ae:75:d7:32";

constexpr uint16_t kSendMinIntervalMs = 5;
constexpr uint16_t kSendKeepAliveMs = 20;
constexpr uint8_t kEspNowChannel = 1;
constexpr uint8_t kBatteryLowPercent = 20;
constexpr int kPwmMax = 1023;
constexpr int kStickDeadband = 12;
constexpr uint32_t kPs4SettleMs = 500;
constexpr uint32_t kEspNowInitRetryMs = 1500;
constexpr bool kEspNowUseBroadcastOnly = true;
constexpr bool kEspNowEnableBroadcastFallback = true;
constexpr uint16_t kEspNowBroadcastThreshold = 3;
constexpr bool kEnablePs4ControllerOutput = false;

// Jika true: gyrX/Y/Z di packet berisi roll/pitch/yaw (derajat × 10) hasil Mahony filter.
// Jika false: gyrX/Y/Z berisi data raw PS4 gyroscope.
constexpr bool kUseProcessedYaw = true;

// Skala gyro PS4: ±2000 dps pada 16-bit signed.
constexpr float kPs4GyrScale = 2000.0f / 32767.0f;

// Skala accelerometer PS4: ±4g pada 16-bit signed → 1g = 8192 unit.
// Jika hasil terlihat terbalik, coba 16384.0f (±2g range).
constexpr float kPs4AccScale = 1.0f / 8192.0f;

// Mahony complementary filter gains.
// Kp: kecepatan koreksi accel terhadap gyro (lebih besar = lebih responsif, lebih rentan noise accel).
// Ki: estimasi bias gyro jangka panjang (yaw drift correction).
constexpr float kMahonyKp = 2.0f;
constexpr float kMahonyKi = 0.0f; // Set 0.0 untuk 6-DOF agar tidak mengakumulasi bias palsu (drift).

// Magic number untuk validasi paket (harus sama di TX dan RX).
constexpr uint16_t kPacketMagic = 0xA5B4;

enum Ps4ButtonBits : uint32_t {
	kBtnUp = 1u << 0,
	kBtnRight = 1u << 1,
	kBtnDown = 1u << 2,
	kBtnLeft = 1u << 3,
	kBtnSquare = 1u << 4,
	kBtnCross = 1u << 5,
	kBtnCircle = 1u << 6,
	kBtnTriangle = 1u << 7,
	kBtnUpRight = 1u << 8,
	kBtnDownRight = 1u << 9,
	kBtnUpLeft = 1u << 10,
	kBtnDownLeft = 1u << 11,
	kBtnL1 = 1u << 12,
	kBtnR1 = 1u << 13,
	kBtnL2 = 1u << 14,
	kBtnR2 = 1u << 15,
	kBtnShare = 1u << 16,
	kBtnOptions = 1u << 17,
	kBtnL3 = 1u << 18,
	kBtnR3 = 1u << 19,
	kBtnPs = 1u << 20,
	kBtnTouchpad = 1u << 21,
};

struct ControlPacket {
	uint16_t magic;     // harus == kPacketMagic, validasi di receiver.
	int16_t x;
	int16_t y;
	int16_t w;
	int8_t lx;
	int8_t ly;
	int8_t rx;
	int8_t ry;
	uint8_t l2Value;
	uint8_t r2Value;
	int16_t gyrX;
	int16_t gyrY;
	int16_t gyrZ;
	uint32_t buttons;
	uint16_t seq;
	uint8_t connected;
};

#endif
