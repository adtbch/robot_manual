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



// =====================================================================
//  FUNGSI: INISIALISASI UART KE WSN-31 (native ESP-IDF driver)
// =====================================================================

/**
 * Inisialisasi UART2 sebagai jalur komunikasi ke modul WSN-31.
 * Menggunakan ESP-IDF uart driver — bukan Serial2 Arduino.
 * Dipanggil satu kali di setup() sebelum fungsi kirim dipakai.
 */
void wsn_serial_init() {
    // Pin SET HIGH = mode normal (data langsung diteruskan radio)
    gpio_config_t set_conf = {
        .pin_bit_mask = (1ULL << kWsnSetPin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&set_conf);
    gpio_set_level((gpio_num_t)kWsnSetPin, 1);

    // Konfigurasi UART native ESP-IDF
    const uart_config_t uart_cfg = {
        .baud_rate = (int)kWsnBaudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM_2, &uart_cfg);
    uart_set_pin(UART_NUM_2, kWsnTxPin, kWsnRxPin,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, 256, 0, 0, NULL, 0);

    ESP_LOGI("wsn31", "UART init — TX=%d RX=%d SET=%d @%ld bps",
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

    // Header (4 byte)
    const uint8_t header[] = {
        kFrameStart0,
        kFrameStart1,
        static_cast<uint8_t>(panjang & 0xFF),
        static_cast<uint8_t>((panjang >> 8) & 0xFF),
    };
    uart_write_bytes(UART_NUM_2, header, sizeof(header));

    // Payload
    uart_write_bytes(UART_NUM_2, payload, panjang);

    // Checksum
    uart_write_bytes(UART_NUM_2, &checksum, 1);

    stat_kirim_wsn++;
}
