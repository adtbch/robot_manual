/*
 * =====================================================================
 * FILE    : config.h
 * PERAN   : Pusat konfigurasi KRAI 2026 Master Board.
 *           Shared types yang dipakai oleh SEMUA modul:
 *           ControlPacket, Jeda, BTN bitmask, pin (nanti).
 *
 * BOARD   : ESP32-S3 (Master)
 *
 * CATATAN:
 *   ControlPacket HARUS sama persis dengan s3controllerespnow/config.h.
 *   Konfigurasi per-modul taruh di .h masing-masing (espnow.h, motor.h, dst).
 * =====================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =====================================================================
//  STRUCT PAKET KONTROL — IDENTIK dengan s3controllerespnow
//  __attribute__((packed)) = tanpa padding bytes
// =====================================================================

struct __attribute__((packed)) ControlPacket {
    uint16_t magic;       // = ESPNOW_PACKET_MAGIC (0xA5B4)

    uint8_t  checksum;    // XOR checksum data

    // --- Perintah gerak ---
    int16_t x;            // Lateral    (+= kanan, -= kiri)
    int16_t y;            // Maju/mundur (+= maju, -= mundur)
    int16_t w;            // Rotasi     (+= CCW,  -= CW)

    // --- Analog stik (-128..127) ---
    int8_t lx;
    int8_t ly;
    int8_t rx;
    int8_t ry;

    // --- Trigger (0..255) ---
    uint8_t l2Value;
    uint8_t r2Value;

    // --- Gyro (selalu 0 via USB HID) ---
    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;

    // --- Bitmask tombol ---
    // bit 0=Cross  1=Circle  2=Triangle  3=Square
    // bit 4=L1  5=R1  6=L2(d)  7=R2(d)
    // bit 8=L3  9=R3  10=Up  11=Down  12=Left  13=Right
    // bit 14=Share  15=Options  16=PS  17=Touchpad
    uint32_t buttons;

    // --- Metadata ---
    uint16_t seq;
    uint8_t  connected;
    uint8_t  mode;        // 0 = manual, 1 = otomatis (dari controller boot)
};

// =====================================================================
//  BITMASK TOMBOL (untuk decode)
// =====================================================================

#define BTN_CROSS    (1u << 0)
#define BTN_CIRCLE   (1u << 1)
#define BTN_TRIANGLE (1u << 2)
#define BTN_SQUARE   (1u << 3)
#define BTN_L1       (1u << 4)
#define BTN_R1       (1u << 5)
#define BTN_L2       (1u << 6)
#define BTN_R2       (1u << 7)
#define BTN_L3       (1u << 8)
#define BTN_R3       (1u << 9)
#define BTN_UP       (1u << 10)
#define BTN_DOWN     (1u << 11)
#define BTN_LEFT     (1u << 12)
#define BTN_RIGHT    (1u << 13)
#define BTN_SHARE    (1u << 14)
#define BTN_OPTIONS  (1u << 15)
#define BTN_PS       (1u << 16)
#define BTN_TOUCHPAD (1u << 17)

// =====================================================================
//  NON-BLOCKING TIMER: Jeda
//  Buat 1 instance per timer yang dibutuhkan.
//
//  Contoh:
//    static Jeda jedaPrint;
//    static Jeda jedaRead;
//    if (jedaPrint.check(1000)) { Serial.println("hi"); }
//    if (jedaRead.check(50))    { bacaSensor(); }
// =====================================================================

struct Jeda {
    uint32_t lastMs = 0;

    bool check(uint32_t intervalMs) {
        const uint32_t nowMs = millis();
        if (nowMs - lastMs < intervalMs) {
            return false;
        }
        lastMs = nowMs;
        return true;
    }

    void reset() {
        lastMs = millis();
    }
};

// =====================================================================
//  SHARED STATE — GripperState (akses dari gripper.ino & gripper_control.ino)
// =====================================================================

enum GripperState { IDLE, CLOSING, UP, STRAIGHTEN, READY_TO_STAB };
extern GripperState gGripperState;

// =====================================================================
//  SHARED STATE — InputMode (akses dari motion_control.ino & gripper_control.ino)
// =====================================================================

enum InputMode { MODE_ANALOG, MODE_DPAD };
extern InputMode gInputMode;

// =====================================================================
//  SHARED STATE — gYawTarget (akses dari motion_control.ino & modul lain)
// =====================================================================

extern int16_t gYawTarget;

// =====================================================================
//  SHARED STATE — gModeInvert (toggle L1+R1+L2+R2; motion lx/ly; gripper lx di driveMotorX saja)
// =====================================================================

extern bool gModeInvert;

// =====================================================================
//  SHARED STATE — AllianceColor (toggle via BOOT button, NVS-stored)
// =====================================================================

enum class AllianceColor : uint8_t { BLUE, RED };
extern AllianceColor gAllianceColor;

inline uint8_t allianceIdx(AllianceColor c) {
    return static_cast<uint8_t>(c);
}
inline uint8_t allianceIdx() {
    return allianceIdx(gAllianceColor);
}
inline const char* allianceLabel(AllianceColor c) {
    return (c == AllianceColor::BLUE) ? "BLUE" : "RED";
}

// =====================================================================
//  SHARED STATE — goto target (motion teleop + forest waypoint)
// =====================================================================

// =====================================================================
//  SERIAL BINARY PROTOCOL TO SLAVE1
// =====================================================================
struct __attribute__((packed)) SerialMotionCmd {
    uint8_t  header1; // 0xAA
    uint8_t  header2; // 0xBB
    uint8_t  type;    // 1 = goto, 2 = kn
    int16_t  x;       // x_cm / vx
    int16_t  y;       // y_cm / vy
    int16_t  yaw;     // yaw_deg
    int16_t  speed;   // speedRpm / dummy
    uint8_t  checksum;
};

extern float   gTargetX_cm;
extern float   gTargetY_cm;
extern int16_t gTargetSpeedRpm;
extern bool    gMotionWaypointMode;  // true = forest set target, stick diblock


// =====================================================================
//  SHARED STATE — odometri dari slave1 (UART1 response)
// =====================================================================

extern float gOdomX_m;
extern float gOdomY_m;
extern float gOdomW_deg;   // heading odometri (derajat)
extern bool  gOdomValid;

extern bool    modeKinematics;   // false=goto, true=kn
extern uint8_t zoneState;

extern ControlPacket gLastRxPacket;

#endif // CONFIG_H
