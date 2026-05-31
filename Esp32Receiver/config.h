#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define encoderExternal0A 38
#define encoderExternal0B 21
#define encoderExternal1A 40
#define encoderExternal1B 39
#define encoderExternal2A 42
#define encoderExternal2B 41
#define encoderExternal3A 1
#define encoderExternal3B 2

#define motorFrontRight0 18
#define motorFrontRight1 17
#define motorFrontLeft0 15
#define motorFrontLeft1 16
#define motorRearRight0 4
#define motorRearRight1 5
#define motorRearLeft0 7
#define motorRearLeft1 6

struct Motor {
	uint8_t pinForward;
	uint8_t pinReverse;
	uint8_t channelForward;
	uint8_t channelReverse;
	bool invert;
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

constexpr int kPwmFreqHz = 25000;
constexpr int kPwmResolutionBits = 10;
constexpr int kPwmMax = 1023;
constexpr uint32_t kCommandTimeoutMs = 180;
constexpr uint8_t kEspNowChannel = 1;
constexpr int kOledSdaPin = 9;
constexpr int kOledSclPin = 8;
constexpr uint8_t kOledI2cAddress = 0x3C;
constexpr int kOledWidth = 128;
constexpr int kOledHeight = 64;
constexpr uint32_t kOledRefreshIntervalMs = 200;
constexpr uint32_t kEspNowLinkAliveMs = 1000;
constexpr uint32_t kBtnLeftMask = 1u << 3;

// Hanya terima paket ESP-NOW dari transmitter ini (strict MAC filter).
constexpr uint8_t kAllowedTransmitterStaMac[6] = {0x58, 0xBF, 0x25, 0x8B, 0xDB, 0x18};
constexpr uint8_t kAllowedTransmitterApMac[6] = {0x58, 0xBF, 0x25, 0x8B, 0xDB, 0x19};

// Magic number harus sama persis dengan transmitter.
constexpr uint16_t kPacketMagic = 0xA5B4;

constexpr float kWheelRadiusM = 0.05f;
constexpr float kWheelbaseLengthM = 0.19f;
constexpr float kTrackWidthM = 0.15f;

constexpr bool kInvertFrontLeft = false;
constexpr bool kInvertFrontRight = true;
constexpr bool kInvertRearLeft = false;
constexpr bool kInvertRearRight = true;


Motor frontLeft = {motorFrontLeft0, motorFrontLeft1, 0, 1, kInvertFrontLeft};
Motor frontRight = {motorFrontRight0, motorFrontRight1, 2, 3, kInvertFrontRight};
Motor rearLeft = {motorRearLeft0, motorRearLeft1, 4, 5, kInvertRearLeft};
Motor rearRight = {motorRearRight0, motorRearRight1, 6, 7, kInvertRearRight};

uint32_t lastCommandTimeMs = 0;

portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;
ControlPacket latestPacket = {};
bool packetAvailable = false;
uint16_t lastSeq = 0;
bool seqInitialized = false;

Adafruit_SSD1306 oled(kOledWidth, kOledHeight, &Wire, -1);
bool oledReady = false;
bool espNowInitReady = false;
uint32_t lastPacketRxMs = 0;
uint32_t lastOledRefreshMs = 0;
ControlPacket oledPacket = {};
bool oledPacketValid = false;
volatile uint32_t rxAnyCount = 0;
volatile uint32_t rxAcceptedCount = 0;
volatile uint32_t rxRejectedMacCount = 0;
volatile uint32_t rxRejectedLenCount = 0;
volatile uint32_t rxRejectedMagicCount = 0;
uint32_t lastRxStatsMs = 0;
uint8_t lastRxMac[6] = {0};
bool lastRxMacValid = false;

bool initAllMotorsPwm();
void driveMotorNormalized(const Motor &motor, float normalizedSpeed);
void stopAllMotors();

float clampCommand(float value);
void moveXYW(float xPwm, float yPwm, float wPwm);
void moveX(float xPwm);
void moveXY(float xPwm, float yPwm);
void applyMecanumCommand(float vx, float vy, float wz);
bool readVelocityCommand(float &vx, float &vy, float &wz);

#endif