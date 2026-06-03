/*
 * =====================================================================
 * FILE    : motion_serial.ino
 * PERAN   : Menerima paket ControlPacket dari ESP32-S3 Slave via Serial1
 *           (UART1). Frame binary yang sama persis dengan WSN-31:
 *
 *           [0xAA] [0x55] [LEN_L] [LEN_H] [...payload...] [XOR_CHECKSUM]
 *
 *           Paket yang diterima diperlakukan SAMA PERSIS dengan yang
 *           diterima dari ESP-NOW — menggunakan struktur ControlPacket
 *           yang sama.
 *
 * WIRING:
 *   ESP32-S3 Slave TX → ESP32-S3 Master RX (motion_serial_rxPin=7)
 *   ESP32-S3 Slave RX ← ESP32-S3 Master TX (motion_serial_txPin=6)
 * =====================================================================
 */

#include "robot_config.h"

#define motion_serial Serial1

// =====================================================================
//  FRAME STATE MACHINE
// =====================================================================

enum MotionSerialState {
  MS_IDLE,        // Mencari magic byte 0xAA
  MS_MAGIC2,      // Mencari magic byte 0x55
  MS_LEN_L,       // Byte panjang low
  MS_LEN_H,       // Byte panjang high
  MS_PAYLOAD,     // Menerima payload
  MS_CHECKSUM     // Menerima checksum
};

static struct {
  MotionSerialState state;
  uint8_t  buf[64];        // Buffer payload
  uint16_t idx;            // Index payload saat ini
  uint16_t payloadLen;     // Panjang payload yang diharapkan
  uint8_t  checksum;       // Checksum yang dihitung

  // Packet output
  ControlPacket latestPacket;
  bool packetAvailable;

  // Stats
  uint32_t rxCount;
  uint32_t acceptedCount;
  uint32_t rejectedChecksum;
  uint32_t rejectedOverflow;

  uint32_t lastPacketRxMs;
  uint32_t lastStatsPrintMs;
} gMS = {};

// =====================================================================
//  INISIALISASI
// =====================================================================

void motion_serial_init() {
  motion_serial.begin(baudrate, SERIAL_8N1, motion_serial_rxPin, motion_serial_txPin);
  memset(&gMS, 0, sizeof(gMS));
  gMS.state = MS_IDLE;

  Serial.printf("motion_serial ready — RX=%d TX=%d @ %lu baud\n",
                motion_serial_rxPin, motion_serial_txPin, (unsigned long)baudrate);
}

// =====================================================================
//  CHECKSUM
// =====================================================================

static uint8_t ms_xor_checksum(const uint8_t *data, uint16_t len) {
  uint8_t cs = 0;
  for (uint16_t i = 0; i < len; i++) {
    cs ^= data[i];
  }
  return cs;
}

// =====================================================================
//  TICK — dipanggil setiap loop()
// =====================================================================

void motion_serial_tick() {
  while (motion_serial.available()) {
    const uint8_t b = motion_serial.read();

    switch (gMS.state) {

      case MS_IDLE:
        if (b == 0xAA) {
          gMS.state = MS_MAGIC2;
        }
        break;

      case MS_MAGIC2:
        gMS.state = (b == 0x55) ? MS_LEN_L : MS_IDLE;
        break;

      case MS_LEN_L:
        gMS.payloadLen = b;
        gMS.state = MS_LEN_H;
        break;

      case MS_LEN_H:
        gMS.payloadLen |= ((uint16_t)b << 8);
        if (gMS.payloadLen > sizeof(gMS.buf)) {
          gMS.rejectedOverflow++;
          gMS.state = MS_IDLE;
        } else {
          gMS.idx = 0;
          gMS.checksum = 0;
          gMS.state = MS_PAYLOAD;
        }
        break;

      case MS_PAYLOAD:
        gMS.buf[gMS.idx++] = b;
        gMS.checksum ^= b;
        if (gMS.idx >= gMS.payloadLen) {
          gMS.state = MS_CHECKSUM;
        }
        break;

      case MS_CHECKSUM:
        gMS.rxCount++;
        if (b == gMS.checksum) {
          memcpy(&gMS.latestPacket, gMS.buf, sizeof(ControlPacket));
          gMS.packetAvailable = true;
          gMS.acceptedCount++;
          gMS.lastPacketRxMs = millis();
        } else {
          gMS.rejectedChecksum++;
        }
        gMS.state = MS_IDLE;
        break;
    }
  }
}

// =====================================================================
//  READ PACKET — dipanggil dari loop()
// =====================================================================

bool motion_serialReadPacket(ControlPacket &outPacket) {
  if (!gMS.packetAvailable) {
    return false;
  }

  outPacket = gMS.latestPacket;
  gMS.packetAvailable = false;
  return true;
}

// =====================================================================
//  STATS — cetak periodik
// =====================================================================

void motion_serialPrintStats() {
  const uint32_t nowMs = millis();
  if (nowMs - gMS.lastStatsPrintMs < 1000) {
    return;
  }
  gMS.lastStatsPrintMs = nowMs;

  const bool linkAlive = (nowMs - gMS.lastPacketRxMs) <= espNowLinkAliveMs;

  Serial.printf("MOTION-SERIAL RX ok=%lu chk=%lu ofw=%lu link=%s\n",
                (unsigned long)gMS.acceptedCount,
                (unsigned long)gMS.rejectedChecksum,
                (unsigned long)gMS.rejectedOverflow,
                linkAlive ? "OK" : "TIMEOUT");
}
