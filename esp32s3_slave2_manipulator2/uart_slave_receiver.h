#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

// ==========================================
// PROTOKOL PENERIMA UART SERIAL (ESP SLAVE 2)
// Frame: 0xAA 0x55 [ID] [CMD] [DATA] [CHK]
// Baudrate: 921600 bps
// ==========================================

#define PROTO_HDR1              0xAA
#define PROTO_HDR2              0x55

#define SLAVE_ID_ARM2           0x02
#define CMD_GO_TO_ARM2          0x06

// Packed Struct Transmit Posisi Lengan 2 (12 bytes)
#pragma pack(push, 1)
typedef struct {
    uint8_t  header1;       // 0xAA
    uint8_t  header2;       // 0x55
    uint8_t  slave_id;      // 0x02
    uint8_t  cmd_type;      // 0x06
    int16_t  target_putar;  // ticks
    int16_t  target_z;      // ticks
    int16_t  target_y;      // ticks
    uint8_t  target_wrist;  // derajat (0-180)
    uint8_t  checksum;      // (slave_id+cmd+putar_hi+putar_lo+z_hi+z_lo+y_hi+y_lo+wrist) & 0xFF
} arm2_packet_t;
#pragma pack(pop)

// Target gerak Lengan 2 - diisi secara asinkron oleh parser UART
extern volatile int16_t  l2_target_putar;
extern volatile int16_t  l2_target_z;
extern volatile int16_t  l2_target_y;
extern volatile uint8_t  l2_target_wrist;
extern volatile uint32_t last_packet_ms;
extern volatile bool     new_target_ready;

// Public API
void uart_slave_init();
void uart_slave_parse_incoming_data();
uint8_t uart_slave_calculate_checksum(const uint8_t *data, uint8_t len);

#endif
