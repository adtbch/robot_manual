/*
 * =====================================================================
 * FILE    : config.h
 * PERAN   : Pusat konfigurasi KRAI 2026 Slave2 Arm Board.
 *           Shared types yang dipakai oleh SEMUA modul.
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =====================================================================
//  NON-BLOCKING TIMER: Jeda
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

#endif // CONFIG_H
