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

void sendPneumaticStatus(char side, bool state) {
    masterSerial.printf("pne %c %d\n", side, state ? 1 : 0);
}

// =====================================================================
//  FULL STATUS — dump semua sensor sekaligus
// =====================================================================

void sendFullStatus() {
    sendProximityStatus('r', readProximity('r'));
    sendProximityStatus('l', readProximity('l'));

    sendLimitStatus("d", readLimitSwitch(LIMIT_ARMBOX_DEPAN));
    sendLimitStatus("b", readLimitSwitch(LIMIT_ARMBOX_BELAKANG));
    sendLimitStatus("t", readLimitSwitch(LIMIT_ARMBOX_TURUN));

    sendEncoderStatus('x', getEncoderCount('x'));
    sendEncoderStatus('y', getEncoderCount('y'));
    sendEncoderStatus('k', getEncoderCount('k'));

    sendPneumaticStatus('r', pneumaticState('r'));
    sendPneumaticStatus('l', pneumaticState('l'));
}
