#include "uart_slave_receiver.h"
#include "config.h"
#include "pinout.h"

// ==========================================
// IMPLEMENTASI RECEIVER SERIAL UART (SLAVE 2)
// ==========================================

volatile int16_t  l2_target_putar  = 0;
volatile int16_t  l2_target_z      = 0;
volatile int16_t  l2_target_y      = 0;
volatile uint8_t  l2_target_wrist  = 0;
volatile uint32_t last_packet_ms   = 0;
volatile bool     new_target_ready = false;

static uint8_t  rx_buf[sizeof(arm2_packet_t)];
static int      rx_state = 0;

void uart_slave_init() {
    Serial1.setRxBufferSize(UART_RX_BUFFER_SIZE);
    Serial1.begin(UART_BAUD_MASTER, SERIAL_8N1, PIN_UART_MASTER_RX, PIN_UART_MASTER_TX);

    #if DEBUG_UART_RX
        Serial.println("[PROTO] UART Master Link initialized @921600bps");
    #endif
}

uint8_t uart_slave_calculate_checksum(const uint8_t *data, uint8_t len) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

void uart_slave_parse_incoming_data() {
    while (Serial1.available() > 0) {
        uint8_t byte_in = (uint8_t)Serial1.read();

        switch (rx_state) {
            case 0:
                if (byte_in == PROTO_HDR1) {
                    rx_state = 1;
                }
                break;

            case 1:
                if (byte_in == PROTO_HDR2) {
                    rx_buf[0] = PROTO_HDR1;
                    rx_buf[1] = PROTO_HDR2;
                    rx_state = 2;
                } else {
                    rx_state = 0;
                }
                break;

            default:
                rx_buf[rx_state] = byte_in;
                rx_state++;

                if (rx_state >= (int)sizeof(arm2_packet_t)) {
                    const arm2_packet_t *pkt = (const arm2_packet_t *)rx_buf;

                    uint8_t calc_chk = uart_slave_calculate_checksum(&rx_buf[2], sizeof(arm2_packet_t) - 3);

                    if (calc_chk == pkt->checksum && pkt->slave_id == SLAVE_ID_ARM2) {
                        l2_target_putar  = pkt->target_putar;
                        l2_target_z      = pkt->target_z;
                        l2_target_y      = pkt->target_y;
                        l2_target_wrist  = pkt->target_wrist;
                        last_packet_ms   = millis();
                        new_target_ready = true;

                        #if DEBUG_UART_RX
                            Serial.printf("[PROTO] Rx Lengan2 -> P:%d Z:%d Y:%d Wrist:%d\n",
                                l2_target_putar, l2_target_z, l2_target_y, l2_target_wrist);
                        #endif
                    } else {
                        #if DEBUG_UART_RX
                            Serial.printf("[PROTO] Rx Checksum/ID Error. Expected:0x%02X Got:0x%02X\n",
                                calc_chk, pkt->checksum);
                        #endif
                    }
                    rx_state = 0;
                }
                break;
        }
    }
}
