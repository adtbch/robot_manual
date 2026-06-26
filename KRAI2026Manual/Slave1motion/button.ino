/*
 * =====================================================================
 * FILE    : button.ino
 * PERAN   : Logic button debouncing.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#include "button.h"

namespace {

bool lastButtonState = HIGH;
uint32_t pressStartMs = 0;
bool isPressing = false;
bool longPressLocked = false;

bool flagShortPress = false;
bool flagLongPress = false;

constexpr uint32_t DEBOUNCE_DELAY_MS = 50;
constexpr uint32_t LONG_PRESS_MS = 3000;

} // anonymous namespace

void setupButton() {
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
}

void updateButton() {
    bool reading = digitalRead(BOOT_BUTTON_PIN);
    uint32_t now = millis();

    if (reading == LOW && lastButtonState == HIGH) {
        // Baru ditekan
        pressStartMs = now;
        isPressing = true;
        longPressLocked = false;
    }
    else if (reading == HIGH && lastButtonState == LOW) {
        // Dilepas
        if (isPressing) {
            if (longPressLocked) {
                flagLongPress = true;
            } else {
                if (now - pressStartMs > DEBOUNCE_DELAY_MS) {
                    flagShortPress = true;
                }
            }
            isPressing = false;
            longPressLocked = false;
        }
    }
    else if (reading == LOW && isPressing) {
        // Sedang ditahan — lock kalau sudah ≥ 3 detik
        if (!longPressLocked && (now - pressStartMs >= LONG_PRESS_MS)) {
            longPressLocked = true;
            Serial.println("[BUTTON] 3 detik. Lepas untuk AUTOTUNE!");
        }
    }

    lastButtonState = reading;
}

bool isButtonShortPressed() {
    if (flagShortPress) { flagShortPress = false; return true; }
    return false;
}

bool isButtonLongPressed() {
    if (flagLongPress) { flagLongPress = false; return true; }
    return false;
}

// Dipakai OLED — true selama tombol masih ditahan dan sudah ≥ 3 detik
bool isButtonLongHolding() {
    return isPressing && longPressLocked;
}
