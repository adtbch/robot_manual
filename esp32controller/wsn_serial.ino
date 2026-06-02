/*
 * =====================================================================
 * FILE    : wsn_serial.ino
 * PERAN   : Komunikasi dengan modul radio WSN-31 via UART binary.
 *           Ini adalah JALUR A (primary) pengiriman data ke ESP32-S3.
 *
 * ISI FILE INI:
 *   - wsn_serial_init()    Inisialisasi Serial2 ke WSN-31 (dipanggil di setup())
 *   - kirim_via_wsn31()    Kirim ControlPacket dalam frame binary
 *
 * WIRING:
 *   ESP32 TX26 → WSN-31 RXD  (ESP32 kirim ke modul)
 *   ESP32 RX27 ← WSN-31 TXD  (ESP32 terima dari modul)
 *   ESP32 25   → WSN-31 SET  (LOW = mode AT command, HIGH = mode normal)
 *   ESP32 GND  → WSN-31 GND  (WAJIB common ground)
 *
 * FORMAT FRAME BINARY:
 *   [0xAA] [0x55] [LEN_L] [LEN_H] [...payload...] [XOR_CHECKSUM]
 *    start   start  panjang payload                  integritas
 *
 * ALIRAN DATA (Jalur A):
 *   ESP32 Controller → UART → WSN-31 #1 ~~radio~~ WSN-31 #2 → ESP32-S3
 * =====================================================================
 */

#include "config.h"

// Alias agar kode lebih mudah dibaca (wsn_serial bukan Serial2)
#define wsn_serial Serial2

// =====================================================================
//  FUNGSI: INISIALISASI UART KE WSN-31
// =====================================================================

/**
 * Inisialisasi Serial2 sebagai jalur komunikasi ke modul WSN-31.
 * Dipanggil satu kali di setup() sebelum fungsi kirim dipakai.
 */
void wsn_serial_init() {
    // Pin SET HIGH = mode normal (transparan, data langsung diteruskan radio)
    pinMode(kWsnSetPin, OUTPUT);
    digitalWrite(kWsnSetPin, HIGH);

    wsn_serial.begin(kWsnBaudrate, SERIAL_8N1, kWsnRxPin, kWsnTxPin);
    Serial.printf("[WSN-31] UART init — TX=%d RX=%d SET=%d @%ld bps\n",
                  kWsnTxPin, kWsnRxPin, kWsnSetPin, kWsnBaudrate);
}

// =====================================================================
//  FUNGSI: KIRIM ControlPacket VIA WSN-31 (frame binary)
// =====================================================================

/**
 * Bungkus ControlPacket ke dalam frame binary lalu kirimkan ke wsn_serial.
 *
 * Struktur frame:
 *   Byte 0    : 0xAA  — start byte pertama
 *   Byte 1    : 0x55  — start byte kedua
 *   Byte 2    : LEN & 0xFF        — panjang payload (low byte)
 *   Byte 3    : (LEN >> 8) & 0xFF — panjang payload (high byte)
 *   Byte 4..N : isi ControlPacket (binary, packed)
 *   Byte N+1  : XOR checksum dari seluruh byte payload
 *
 * @param paket ControlPacket yang akan dikirim
 */
void kirim_via_wsn31(const ControlPacket &paket) {
    const uint8_t  *payload  = reinterpret_cast<const uint8_t *>(&paket);
    const uint16_t  panjang  = static_cast<uint16_t>(sizeof(ControlPacket));
    const uint8_t   checksum = hitung_checksum(payload, panjang);

    // Tulis header (4 byte)
    wsn_serial.write(kFrameStart0);
    wsn_serial.write(kFrameStart1);
    wsn_serial.write(static_cast<uint8_t>(panjang & 0xFF));
    wsn_serial.write(static_cast<uint8_t>((panjang >> 8) & 0xFF));

    // Tulis payload (sizeof(ControlPacket) byte)
    wsn_serial.write(payload, panjang);

    // Tulis checksum (1 byte)
    wsn_serial.write(checksum);

    stat_kirim_wsn++;
}
