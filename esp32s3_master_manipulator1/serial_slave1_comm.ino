// ============================================================
// SERIAL COMMUNICATION - Master to Slave1 (2-Way)
// File: serial_slave1_comm.ino
// ============================================================
// Komunikasi serial 2 arah antara Master dan Slave1
// Master mengirim: perintah motor (vx, vy, vtheta)
// Slave1 mengirim: status odometri, RPM motor, dll
// ============================================================

#include "robot_config.h"

namespace SerialSlave1 {

// ============================================================
// KONFIGURASI SERIAL
// ============================================================
#define SERIAL_SLAVE1_BAUD 921600
// PENTING: Sesuaikan pin ini dengan hardware ESP32-S3 Master Anda!
// Pin ini harus crossover dengan Slave1 (RX:9, TX:10)
// Contoh: Master TX:17 --> Slave1 RX:9
//         Master RX:16 <-- Slave1 TX:10
#define SERIAL_SLAVE1_RX_PIN 16  // Pin RX Master (ganti sesuai hardware)
#define SERIAL_SLAVE1_TX_PIN 17  // Pin TX Master (ganti sesuai hardware)

// ============================================================
// PROTOKOL PAKET
// ============================================================
// Header dan Footer untuk validasi paket
#define PACKET_HEADER_1 0xAA
#define PACKET_HEADER_2 0x55
#define PACKET_FOOTER   0xFF

// Command IDs
#define CMD_MOTOR_CONTROL  0x01  // Master -> Slave1: Kontrol motor
#define CMD_MOTOR_STOP     0x02  // Master -> Slave1: Stop semua motor
#define CMD_STATUS_REQUEST 0x03  // Master -> Slave1: Request status
#define CMD_STATUS_REPLY   0x04  // Slave1 -> Master: Reply status
#define CMD_ODOMETRY_DATA  0x05  // Slave1 -> Master: Data odometri

// ============================================================
// STRUKTUR DATA PAKET
// ============================================================

// Paket Master -> Slave1: Kontrol Motor
typedef struct __attribute__((packed)) {
  uint8_t header1;      // 0xAA
  uint8_t header2;      // 0x55
  uint8_t cmdId;        // CMD_MOTOR_CONTROL
  int16_t vx;           // Velocity X (-1000 to 1000)
  int16_t vy;           // Velocity Y (-1000 to 1000)
  int16_t vtheta;       // Velocity Theta/Rotasi (-1000 to 1000)
  uint8_t checksum;     // XOR checksum
  uint8_t footer;       // 0xFF
} MotorControlPacket;

// Paket Master -> Slave1: Stop Motor
typedef struct __attribute__((packed)) {
  uint8_t header1;      // 0xAA
  uint8_t header2;      // 0x55
  uint8_t cmdId;        // CMD_MOTOR_STOP
  uint8_t checksum;     // XOR checksum
  uint8_t footer;       // 0xFF
} MotorStopPacket;

// NOTE: StatusReplyPacket dan OdometryDataPacket sudah didefinisikan di robot_config.h
// Tidak perlu didefinisikan ulang di sini untuk menghindari duplicate typedef error

// ============================================================
// VARIABEL GLOBAL
// ============================================================
static bool gSerialReady = false;
static uint32_t gLastTxMs = 0;
static uint32_t gLastRxMs = 0;
static uint32_t gTxCount = 0;
static uint32_t gRxCount = 0;
static uint32_t gRxErrorCount = 0;

// Buffer untuk parsing
static uint8_t gRxBuffer[64];
static uint8_t gRxBufferIndex = 0;
static uint8_t gRxState = 0;

// Data terakhir yang diterima
static StatusReplyPacket gLastStatus = {};
static OdometryDataPacket gLastOdometry = {};
static bool gStatusValid = false;
static bool gOdometryValid = false;

// ============================================================
// HELPER FUNCTIONS
// ============================================================

// Hitung checksum XOR
uint8_t calculateChecksum(const uint8_t *data, uint8_t len) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

// Validasi checksum paket
bool validateChecksum(const uint8_t *packet, uint8_t len, uint8_t receivedChecksum) {
  uint8_t calculated = calculateChecksum(packet, len);
  return calculated == receivedChecksum;
}

// ============================================================
// INISIALISASI
// ============================================================
bool serialSlave1Init() {
  Serial1.begin(SERIAL_SLAVE1_BAUD, SERIAL_8N1, SERIAL_SLAVE1_RX_PIN, SERIAL_SLAVE1_TX_PIN);
  Serial1.setRxBufferSize(1024);  // Buffer besar untuk baudrate tinggi
  
  gSerialReady = true;
  gLastTxMs = millis();
  gLastRxMs = millis();
  
  Serial.println("=== SERIAL SLAVE1 INIT ===");
  Serial.printf("Baud: %d\n", SERIAL_SLAVE1_BAUD);
  Serial.printf("RX Pin: %d\n", SERIAL_SLAVE1_RX_PIN);
  Serial.printf("TX Pin: %d\n", SERIAL_SLAVE1_TX_PIN);
  Serial.println("==========================");
  
  return gSerialReady;
}

// ============================================================
// TRANSMIT FUNCTIONS (Master -> Slave1)
// ============================================================

// Kirim perintah kontrol motor
bool sendMotorControl(int16_t vx, int16_t vy, int16_t vtheta) {
  if (!gSerialReady) {
    return false;
  }
  
  MotorControlPacket packet = {};
  packet.header1 = PACKET_HEADER_1;
  packet.header2 = PACKET_HEADER_2;
  packet.cmdId = CMD_MOTOR_CONTROL;
  packet.vx = vx;
  packet.vy = vy;
  packet.vtheta = vtheta;
  packet.footer = PACKET_FOOTER;
  
  // Hitung checksum (skip header, footer, checksum field)
  uint8_t *data = (uint8_t *)&packet;
  packet.checksum = calculateChecksum(data + 2, sizeof(MotorControlPacket) - 4);
  
  // Kirim paket
  size_t written = Serial1.write((uint8_t *)&packet, sizeof(MotorControlPacket));
  
  if (written == sizeof(MotorControlPacket)) {
    gTxCount++;
    gLastTxMs = millis();
    return true;
  }
  
  return false;
}

// Kirim perintah stop motor
bool sendMotorStop() {
  if (!gSerialReady) {
    return false;
  }
  
  MotorStopPacket packet = {};
  packet.header1 = PACKET_HEADER_1;
  packet.header2 = PACKET_HEADER_2;
  packet.cmdId = CMD_MOTOR_STOP;
  packet.footer = PACKET_FOOTER;
  
  // Hitung checksum
  uint8_t *data = (uint8_t *)&packet;
  packet.checksum = calculateChecksum(data + 2, sizeof(MotorStopPacket) - 4);
  
  // Kirim paket
  size_t written = Serial1.write((uint8_t *)&packet, sizeof(MotorStopPacket));
  
  if (written == sizeof(MotorStopPacket)) {
    gTxCount++;
    gLastTxMs = millis();
    return true;
  }
  
  return false;
}

// Kirim request status
bool sendStatusRequest() {
  if (!gSerialReady) {
    return false;
  }
  
  MotorStopPacket packet = {};  // Reuse struktur yang sama
  packet.header1 = PACKET_HEADER_1;
  packet.header2 = PACKET_HEADER_2;
  packet.cmdId = CMD_STATUS_REQUEST;
  packet.footer = PACKET_FOOTER;
  
  // Hitung checksum
  uint8_t *data = (uint8_t *)&packet;
  packet.checksum = calculateChecksum(data + 2, sizeof(MotorStopPacket) - 4);
  
  // Kirim paket
  size_t written = Serial1.write((uint8_t *)&packet, sizeof(MotorStopPacket));
  
  if (written == sizeof(MotorStopPacket)) {
    gTxCount++;
    gLastTxMs = millis();
    return true;
  }
  
  return false;
}

// ============================================================
// RECEIVE FUNCTIONS (Slave1 -> Master)
// ============================================================

// Process received status reply
void processStatusReply(const uint8_t *data, uint8_t len) {
  if (len != sizeof(StatusReplyPacket)) {
    gRxErrorCount++;
    return;
  }
  
  StatusReplyPacket packet = {};
  memcpy(&packet, data, sizeof(StatusReplyPacket));
  
  // Validasi checksum
  if (!validateChecksum(data + 2, sizeof(StatusReplyPacket) - 4, packet.checksum)) {
    gRxErrorCount++;
    return;
  }
  
  // Simpan data
  gLastStatus = packet;
  gStatusValid = true;
  gRxCount++;
  gLastRxMs = millis();
}

// Process received odometry data
void processOdometryData(const uint8_t *data, uint8_t len) {
  if (len != sizeof(OdometryDataPacket)) {
    gRxErrorCount++;
    return;
  }
  
  OdometryDataPacket packet = {};
  memcpy(&packet, data, sizeof(OdometryDataPacket));
  
  // Validasi checksum
  if (!validateChecksum(data + 2, sizeof(OdometryDataPacket) - 4, packet.checksum)) {
    gRxErrorCount++;
    return;
  }
  
  // Simpan data
  gLastOdometry = packet;
  gOdometryValid = true;
  gRxCount++;
  gLastRxMs = millis();
}

// ============================================================
// PARSING STATE MACHINE (Non-Blocking)
// ============================================================
void serialSlave1Tick() {
  if (!gSerialReady) {
    return;
  }
  
  // Baca semua byte yang tersedia
  while (Serial1.available() > 0) {
    uint8_t byte = Serial1.read();
    
    switch (gRxState) {
      case 0:  // Tunggu header 1
        if (byte == PACKET_HEADER_1) {
          gRxBuffer[0] = byte;
          gRxBufferIndex = 1;
          gRxState = 1;
        }
        break;
        
      case 1:  // Tunggu header 2
        if (byte == PACKET_HEADER_2) {
          gRxBuffer[1] = byte;
          gRxBufferIndex = 2;
          gRxState = 2;
        } else {
          gRxState = 0;  // Reset
        }
        break;
        
      case 2:  // Baca command ID
        gRxBuffer[2] = byte;
        gRxBufferIndex = 3;
        gRxState = 3;
        break;
        
      case 3:  // Baca payload
        gRxBuffer[gRxBufferIndex++] = byte;
        
        // Cek apakah sudah lengkap berdasarkan command ID
        uint8_t cmdId = gRxBuffer[2];
        uint8_t expectedLen = 0;
        
        if (cmdId == CMD_STATUS_REPLY) {
          expectedLen = sizeof(StatusReplyPacket);
        } else if (cmdId == CMD_ODOMETRY_DATA) {
          expectedLen = sizeof(OdometryDataPacket);
        }
        
        if (gRxBufferIndex >= expectedLen) {
          // Paket lengkap, proses
          if (cmdId == CMD_STATUS_REPLY) {
            processStatusReply(gRxBuffer, expectedLen);
          } else if (cmdId == CMD_ODOMETRY_DATA) {
            processOdometryData(gRxBuffer, expectedLen);
          }
          
          // Reset state
          gRxState = 0;
          gRxBufferIndex = 0;
        }
        
        // Proteksi buffer overflow
        if (gRxBufferIndex >= sizeof(gRxBuffer)) {
          gRxState = 0;
          gRxBufferIndex = 0;
          gRxErrorCount++;
        }
        break;
    }
  }
}

// ============================================================
// PUBLIC API FUNCTIONS
// ============================================================

// Cek apakah ada status baru
bool getStatus(StatusReplyPacket &outStatus) {
  if (gStatusValid) {
    outStatus = gLastStatus;
    gStatusValid = false;  // Clear flag
    return true;
  }
  return false;
}

// Cek apakah ada odometry baru
bool getOdometry(OdometryDataPacket &outOdometry) {
  if (gOdometryValid) {
    outOdometry = gLastOdometry;
    gOdometryValid = false;  // Clear flag
    return true;
  }
  return false;
}

// Cek apakah link masih hidup
bool isLinkAlive() {
  const uint32_t timeout = 500;  // 500ms timeout
  return (millis() - gLastRxMs) < timeout;
}

// Print statistik
void printStats() {
  Serial.println("=== SERIAL SLAVE1 STATS ===");
  Serial.printf("TX Count: %lu\n", gTxCount);
  Serial.printf("RX Count: %lu\n", gRxCount);
  Serial.printf("RX Error: %lu\n", gRxErrorCount);
  Serial.printf("Last TX: %lu ms ago\n", millis() - gLastTxMs);
  Serial.printf("Last RX: %lu ms ago\n", millis() - gLastRxMs);
  Serial.printf("Link: %s\n", isLinkAlive() ? "ALIVE" : "TIMEOUT");
  Serial.println("===========================");
}

}  // namespace SerialSlave1
