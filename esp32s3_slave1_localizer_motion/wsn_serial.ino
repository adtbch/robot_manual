/*
 * =====================================================================
 * FILE    : wsn_serial.ino
 * PERAN   : Menerima paket binary dari modul radio WSN-31 via UART.
 *           Ini adalah JALUR A penerima data dari ESP32 Controller.
 *
 * FORMAT FRAME BINARY (dari esp32controller):
 *   [0xAA] [0x55] [LEN_L] [LEN_H] [...payload...] [XOR_CHECKSUM]
 *    start   start  panjang payload                  integritas
 *
 * PAYLOAD = ControlPacket (29 bytes, packed)
 *
 * WIRING (ESP32-S3):
 *   ESP32-S3 RX  (wsn_serial_rxPin=12) ← TXD WSN-31
 *   ESP32-S3 TX  (wsn_serial_txPin=11) → RXD WSN-31
 *   ESP32-S3 SET (kWsnSetPin=19)        → WSN-31 SET
 *   ESP32-S3 GND                        → WSN-31 GND
 *
 * CATATAN:
 *   Struct ControlPacket harus IDENTIK dengan yang di esp32controller.
 * =====================================================================
 */

#include "robot_config.h"

// Alias agar kode lebih mudah dibaca
#define wsn_serial Serial2

// =====================================================================
//  STATE MESIN STATE PARSE (frame binary receiver)
// =====================================================================

enum class WsnParseState : uint8_t {
    WAIT_START0,    // Menunggu byte pertama frame (0xAA)
    WAIT_START1,    // Menunggu byte kedua frame (0x55)
    READ_LEN_L,     // Baca panjang payload (low byte)
    READ_LEN_H,     // Baca panjang payload (high byte)
    READ_PAYLOAD,   // Baca byte payload satu per satu
    READ_CHECKSUM   // Baca byte checksum
};

static WsnParseState gWsnState = WsnParseState::WAIT_START0;
static uint8_t  gWsnPayloadBuffer[256];   // Buffer sementara untuk payload
static uint16_t gWsnPayloadIdx = 0;       // Index byte saat ini di payload
static uint16_t gWsnPayloadLen = 0;       // Panjang payload yang diharapkan
static uint8_t  gWsnXorAccum = 0;         // Akumulator XOR checksum

// Packet terakhir yang berhasil di-parse (volatile karena dibaca dari loop)
static volatile ControlPacket gWsnLatestPacket = {};
static volatile bool         gWsnPacketReady = false;
static uint16_t              gWsnLastSeq = 0;

// Statistik
static uint32_t gWsnRxOk = 0;
static uint32_t gWsnRxErrCrc = 0;
static uint32_t gWsnRxErrLen = 0;
static uint32_t gWsnRxErrMagic = 0;

// =====================================================================
//  FUNGSI: INISIALISASI UART KE WSN-31
// =====================================================================

/**
 * Inisialisasi Serial2 sebagai jalur komunikasi dari modul WSN-31.
 * Dipanggil satu kali di setup().
 */
void wsn_serial_init() {
    // Pin SET HIGH = mode normal (transparan, data langsung diteruskan radio)
    pinMode(kWsnSetPin, OUTPUT);
    digitalWrite(kWsnSetPin, HIGH);

    wsn_serial.begin(9600, SERIAL_8N1, wsn_serial_rxPin, wsn_serial_txPin);
    Serial.printf("[WSN-31] UART init — RX=%d TX=%d SET=%d @115200 bps\n",
                  wsn_serial_rxPin, wsn_serial_txPin, kWsnSetPin);
}

// =====================================================================
//  FUNGSI: HITUNG XOR CHECKSUM
// =====================================================================

static uint8_t wsn_hitung_checksum(const uint8_t *data, uint16_t panjang) {
    uint8_t cs = 0;
    for (uint16_t i = 0; i < panjang; i++) {
        cs ^= data[i];
    }
    return cs;
}

// =====================================================================
//  FUNGSI: PROSES 1 BYTE MASUKAN (state machine)
// =====================================================================

/**
 * Feed satu byte ke state machine parser frame binary.
 * Panggil terus-menerus dari loop() selama wsn_serial available.
 *
 * Frame: [0xAA][0x55][LEN_L][LEN_H][payload...][XOR]
 */
static void wsn_parse_byte(uint8_t b) {
    switch (gWsnState) {
        case WsnParseState::WAIT_START0:
            if (b == 0xAA) {
                gWsnState = WsnParseState::WAIT_START1;
            }
            break;

        case WsnParseState::WAIT_START1:
            gWsnState = (b == 0x55) ? WsnParseState::READ_LEN_L
                                     : WsnParseState::WAIT_START0;
            break;

        case WsnParseState::READ_LEN_L:
            gWsnPayloadLen = b;
            gWsnState = WsnParseState::READ_LEN_H;
            break;

        case WsnParseState::READ_LEN_H:
            gWsnPayloadLen |= (static_cast<uint16_t>(b) << 8);
            if (gWsnPayloadLen > sizeof(gWsnPayloadBuffer)) {
                // Payload terlalu besar, reset
                gWsnRxErrLen++;
                gWsnState = WsnParseState::WAIT_START0;
            } else {
                gWsnPayloadIdx = 0;
                gWsnXorAccum = 0;
                gWsnState = WsnParseState::READ_PAYLOAD;
            }
            break;

        case WsnParseState::READ_PAYLOAD:
            gWsnPayloadBuffer[gWsnPayloadIdx++] = b;
            gWsnXorAccum ^= b;
            if (gWsnPayloadIdx >= gWsnPayloadLen) {
                gWsnState = WsnParseState::READ_CHECKSUM;
            }
            break;

        case WsnParseState::READ_CHECKSUM: {
            if (b == gWsnXorAccum) {
                // CRC valid — cek magic
                ControlPacket candidate = {};
                if (gWsnPayloadLen == sizeof(ControlPacket)) {
                    memcpy(&candidate, gWsnPayloadBuffer, sizeof(ControlPacket));
                    if (candidate.magic == kPacketMagic) {
                        // Paket valid!
                        memcpy((void*)&gWsnLatestPacket, &candidate, sizeof(ControlPacket));
                        gWsnPacketReady = true;
                        gWsnRxOk++;
                    } else {
                        gWsnRxErrMagic++;
                    }
                } else {
                    gWsnRxErrLen++;
                }
            } else {
                gWsnRxErrCrc++;
            }
            gWsnState = WsnParseState::WAIT_START0;
            break;
        }
    }
}

// =====================================================================
//  FUNGSI: BACA PAKET TERBARU (dipanggil dari loop)
// =====================================================================

/**
 * Cek apakah ada paket baru dari WSN-31. Jika ya, copy ke outPacket.
 *
 * @param outPacket Tempat menyimpan paket yang diterima
 * @return true jika ada paket baru, false jika belum
 */
bool wsn_serial_readPacket(ControlPacket &outPacket) {
    if (!gWsnPacketReady) {
        return false;
    }

    // Copy dari volatile pakai memcpy
    noInterrupts();
    memcpy(&outPacket, (const void*)&gWsnLatestPacket, sizeof(ControlPacket));
    gWsnPacketReady = false;
    interrupts();

    return true;
}

// =====================================================================
//  FUNGSI: TIck — baca semua byte dari UART
// =====================================================================

/**
 * Dipanggil setiap iterasi loop(). Membaca semua byte yang tersedia
 * di buffer UART dan memasukkannya ke state machine parser.
 */
void wsn_serial_tick() {
    while (wsn_serial.available()) {
        uint8_t b = wsn_serial.read();
        wsn_parse_byte(b);
    }
}

// =====================================================================
//  FUNGSI: STATISTIK
// =====================================================================

uint32_t wsn_serial_getRxOk()        { return gWsnRxOk; }
uint32_t wsn_serial_getRxCrcErr()    { return gWsnRxErrCrc; }
uint32_t wsn_serial_getRxMagicErr()  { return gWsnRxErrMagic; }
