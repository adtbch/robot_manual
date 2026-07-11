/*
 * =====================================================================
 * FILE    : serial_command.ino
 * PERAN   : Kirim data/status sensor KE master via UART1.
 *           Dipisah dari serial.ino yang hanya parsing perintah masuk.
 *
 * KONSEP:
 *   Slave2arm baca semua sensor, kirim status ke master.
 *   Master baca dan decide mau diapakan.
 *   Slave2arm TIDAK punya logika box/gerakan — semua keputusan di master.
 *
 * UART1   : masterSerial (RX=36, TX=35) → master
 *
 * PROTOCOL:
 *   Format: <jenis> <id> <nilai>\n
 *   Contoh:
 *     prox r 1          — proximity R detected
 *     prox l 0          — proximity L clear
 *     limit depan 1     — limit switch depan triggered
 *     enc x 542         — encoder X = 542
 *     pne r 1           — pneumatic R = ON
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#include "serial.h"
#include "proximity.h"
#include "limit_switch.h"
#include "pneumatic.h"
#include "encoder.h"
#include "motor.h"

void sendProximityStatus(char side, bool detected) {
    masterSerial.printf("prox %c %d\n", side, detected ? 1 : 0);
}

void sendLimitStatus(const char* name, bool triggered) {
    masterSerial.printf("limit %s %d\n", name, triggered ? 1 : 0);
}

void sendEncoderStatus(char id, long count) {
    masterSerial.printf("enc %c %ld\n", id, count);
}

// =====================================================================
//  SENSOR TICK — auto-kirim perubahan proximity & limit ke master
//
//  - value berubah → kirim langsung, reset counter
//  - value sama → counter++, kirim tiap 50ms
// =====================================================================

namespace {

struct SensorTrack {
    bool lastValue;
    uint8_t sameCount;
    uint32_t lastSendMs;
};

SensorTrack gProxR = {false, 0, 0};
SensorTrack gProxL = {false, 0, 0};
SensorTrack gLimitT = {false, 0, 0};

constexpr uint32_t SENSOR_INTERVAL_MS = 50;
constexpr uint8_t  SENSOR_SAME_MAX    = 50;

void trackAndSend(SensorTrack &track, bool currentValue, void (*sendFn)()) {
    // Cek perubahan — reset counter, kirim langsung
    if (currentValue != track.lastValue) {
        track.lastValue = currentValue;
        track.sameCount = 0;
        track.lastSendMs = millis();
        sendFn();
        return;
    }

    // Value sama, sudah 50x → stop kirim
    if (track.sameCount >= SENSOR_SAME_MAX) return;

    // Value sama, belum 50x → kirim tiap 50ms
    if (millis() - track.lastSendMs < SENSOR_INTERVAL_MS) return;
    track.sameCount++;
    track.lastSendMs = millis();
    sendFn();
}

} // anonymous namespace

void sensorTick() {
    trackAndSend(gProxR, readProximity('r'), []() {
        sendProximityStatus('r', readProximity('r'));
    });

    trackAndSend(gProxL, readProximity('l'), []() {
        sendProximityStatus('l', readProximity('l'));
    });

    trackAndSend(gLimitT, readLimitSwitch(LIMIT_ARMBOX_TURUN), []() {
        sendLimitStatus("t", readLimitSwitch(LIMIT_ARMBOX_TURUN));
    });
}
