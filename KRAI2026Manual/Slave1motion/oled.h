/*
 * =====================================================================
 * FILE    : oled.h
 * PERAN   : Konfigurasi OLED display (SSD1306 128x64).
 *           Multi-mode: yaw, debug (enc + rpm + odometry).
 *           Boot button untuk ganti mode.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef OLED_H
#define OLED_H

#include "config.h"

// =====================================================================
//  DISPLAY
// =====================================================================
constexpr int OLED_WIDTH  = 128;
constexpr int OLED_HEIGHT = 64;
constexpr uint8_t OLED_I2C_ADDR = 0x3C;
constexpr int8_t OLED_RESET = -1;

// =====================================================================
//  MODE
// =====================================================================
enum OledMode : uint8_t {
    OLED_MODE_YAW = 0,
    OLED_MODE_DEBUG,
    OLED_MODE_COUNT
};

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
bool setupOLED();
void updateOLED();
void oledShowStatus(const char* line1, const char* line2);  // Tampilkan text ke OLED (untuk setup/debug)

#endif // OLED_H
