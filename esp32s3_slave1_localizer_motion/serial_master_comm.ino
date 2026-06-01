// ============================================================
// SERIAL COMMUNICATION - Slave1 Receiver (2-Way)
// File: serial_master_comm.ino
// ============================================================
// Komunikasi serial 2 arah antara Slave1 dan Master
// Slave1 menerima: perintah motor (vx, vy, vtheta)
// Slave1 mengirim: status odometri, RPM motor, dll
// ============================================================

#include "robot_config.h"

namespace SerialMaster {

// ============================================================
// KONFIGURASI SERIAL
// ============================================================
#define SERIAL_MASTER_BAUD 921600
// Pin sudah didefinisikan di robot_config.h:
// serial_1_rxPin = 9
// serial_1_txPin = 10

// ============================================================
// PROTOKOL PAKET (SAMA DENGAN MASTER)
// ============================================================
#define PACKET_HEADER_1 0xAA
#define PACKET_HEADER_2 0x55
#define PACKET_FOOTER   0xFF

// Command IDs
#define CMD_MOTOR_CONTROL  0x01
#define CMD_MOTOR_STOP     0x02
#define CMD_STATUS_REQUEST 0x03
#define CMD_STATUS_REPLY   0x04
#define CMD_ODOMETRY_DATA  0x05

// ============================================================
// STRUKTUR DATA PAKET (SAMA DENGAN MASTER)
// ============================================================

typedef struct __attribute__((packed)) {
  uint8_t header1;
  uint8_t header2;
  uint8_t cmdId;
  int16_t vx;
  int16_t vy;
  int16_t vtheta;
  uint8_t checksum;
  uint8_t footer;
} MotorControlPacket;

typedef struct __attribute__((packed)) {
  uint8_t header1;
  uint8_t header2;
  uint8_t cmdId;
  uint8_t checksum;
  uint8_t footer;
} MotorStopPacket;

typedef struct __attribute__((packed)) {
  uint8_t header1;
  uint8_t header2;
  uint8_t cmdId;
  uint16_t rpmMotor1;
  uint16_t rpmMotor2;
  uint16_t rpmMotor3;
  uint16_t rpmMotor4;
  uint8_t status;
  uint8_t checksum;
  uint8_t footer;
} StatusReplyPacket;

typedef struct __attribute__((packed)) {
  uint8_t header1;
  uint8_t header2;
  uint8_t cmdId;
  float posX;
  float posY;
  float heading;
  uint8_t checksum;
  uint8_t footer;
} OdometryDataPacket;

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
static MotorControlPacket gLastMotorCmd = {};
static bool gMotorCmdValid = false;
static bool gStopRequested = false;
static bool gStatusRequested = false;

// ============================================================
// HELPER FUNCTIONS
// ============================================================

uint8_t calculateChecksum(const uint8_t *data, uint8_t len) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

bool validateChecksum(const uint8_t *packet, uint8_t len, uint8_t receivedChecksum) {
  uint8_t calculated = calculateChecksum(packet, len);
  return calculated == receivedChecksum;
}

// ============================================================
// INISIALISASI
// ============================================================
bool serialMasterInit() {
  Serial1.begin(SERIAL_MASTER_BAUD, SERIAL_8N1, serial_1_rxPin, serial_1_txPin);
  Serial1.setRxBufferSize(1024);
  
  gSerialReady = true;
  gLastTxMs = millis();
  gLastRxMs = millis();
  
  Serial.println("=== SERIAL MASTER COMM INIT ===");
  Serial.printf("Baud: %d\n", SERIAL_MASTER_BAUD);
  Serial.printf("RX Pin: %d\n", serial_1_rxPin);
  Serial.printf("TX Pin: %d\n", serial_1_txPin);
  Serial.println("================================");
  
  return gSerialReady;
}

// ============================================================
// TRANSMIT FUNCTIONS (Slave1 -> Master)
// ============================================================

bool sendStatusReply(uint16_t rpm1, uint16_t rpm2, uint16_t rpm3, uint16_t rpm4, uint8_t status) {
  if (!gSerialReady) {
    return false;
  }
  
  StatusReplyPacket packet = {};
  packet.header1 = PACKET_HEADER_1;
  packet.header2 = PACKET_HEADER_2;
  packet.cmdId = CMD_STATUS_REPLY;
  packet.rpmMotor1 = rpm1;
  packet.rpmMotor2 = rpm2;
  packet.rpmMotor3 = rpm3;
  packet.rpmMotor4 = rpm4;
  packet.status = status;
  packet.footer = PACKET_FOOTER;
  
  uint8_t *data = (uint8_t *)&packet;
  packet.checksum = calculateChecksum(data + 2, sizeof(StatusReplyPacket) - 4);
  
  size_t written = Serial1.write((uint8_t *)&packet, sizeof(StatusReplyPacket));
  
  if (written == sizeof(StatusReplyPacket)) {
    gTxCount++;
    gLastTxMs = millis();
    return true;
  }
  
  return false;
}

bool sendOdometryData(float posX, float posY, float heading) {
  if (!gSerialReady) {
    return false;
  }
  
  OdometryDataPacket packet = {};
  packet.header1 = PACKET_HEADER_1;
  packet.header2 = PACKET_HEADER_2;
  packet.cmdId = CMD_ODOMETRY_DATA;
  packet.posX = posX;
  packet.posY = posY;
  packet.heading = heading;
  packet.footer = PACKET_FOOTER;
  
  uint8_t *data = (uint8_t *)&packet;
  packet.checksum = calculateChecksum(data + 2, sizeof(OdometryDataPacket) - 4);
  
  size_t written = Serial1.write((uint8_t *)&packet, sizeof(OdometryDataPacket));
  
  if (written == sizeof(OdometryDataPacket)) {
    gTxCount++;
    gLastTxMs = millis();
    return true;
  }
  
  return false;
}

// ============================================================
// RECEIVE FUNCTIONS (Master -> Slave1)
// ============================================================

void processMotorControl(const uint8_t *data, uint8_t len) {
  if (len != sizeof(MotorControlPacket)) {
    gRxErrorCount++;
    return;
  }
  
  MotorControlPacket packet = {};
  memcpy(&packet, data, sizeof(MotorControlPacket));
  
  if (!validateChecksum(data + 2, sizeof(MotorControlPacket) - 4, packet.checksum)) {
    gRxErrorCount++;
    return;
  }
  
  gLastMotorCmd = packet;
  gMotorCmdValid = true;
  gRxCount++;
  gLastRxMs = millis();
}

void processMotorStop(const uint8_t *data, uint8_t len) {
  if (len != sizeof(MotorStopPacket)) {
    gRxErrorCount++;
    return;
  }
  
  MotorStopPacket packet = {};
  memcpy(&packet, data, sizeof(MotorStopPacket));
  
  if (!validateChecksum(data + 2, sizeof(MotorStopPacket) - 4, packet.checksum)) {
    gRxErrorCount++;
    return;
  }
  
  gStopRequested = true;
  gRxCount++;
  gLastRxMs = millis();
}

void processStatusRequest(const uint8_t *data, uint8_t len) {
  if (len != sizeof(MotorStopPacket)) {
    gRxErrorCount++;
    return;
  }
  
  MotorStopPacket packet = {};
  memcpy(&packet, data, sizeof(MotorStopPacket));
  
  if (!validateChecksum(data + 2, sizeof(MotorStopPacket) - 4, packet.checksum)) {
    gRxErrorCount++;
    return;
  }
  
  gStatusRequested = true;
  gRxCount++;
  gLastRxMs = millis();
}

// ============================================================
// PARSING STATE MACHINE (Non-Blocking)
// ============================================================
void serialMasterTick() {
  if (!gSerialReady) {
    return;
  }
  
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
          gRxState = 0;
        }
        break;
        
      case 2:  // Baca command ID
        gRxBuffer[2] = byte;
        gRxBufferIndex = 3;
        gRxState = 3;
        break;
        
      case 3:  // Baca payload
        gRxBuffer[gRxBufferIndex++] = byte;
        
        uint8_t cmdId = gRxBuffer[2];
        uint8_t expectedLen = 0;
        
        if (cmdId == CMD_MOTOR_CONTROL) {
          expectedLen = sizeof(MotorControlPacket);
        } else if (cmdId == CMD_MOTOR_STOP || cmdId == CMD_STATUS_REQUEST) {
          expectedLen = sizeof(MotorStopPacket);
        }
        
        if (gRxBufferIndex >= expectedLen) {
          if (cmdId == CMD_MOTOR_CONTROL) {
            processMotorControl(gRxBuffer, expectedLen);
          } else if (cmdId == CMD_MOTOR_STOP) {
            processMotorStop(gRxBuffer, expectedLen);
          } else if (cmdId == CMD_STATUS_REQUEST) {
            processStatusRequest(gRxBuffer, expectedLen);
          }
          
          gRxState = 0;
          gRxBufferIndex = 0;
        }
        
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

bool getMotorCommand(int16_t &vx, int16_t &vy, int16_t &vtheta) {
  if (gMotorCmdValid) {
    vx = gLastMotorCmd.vx;
    vy = gLastMotorCmd.vy;
    vtheta = gLastMotorCmd.vtheta;
    gMotorCmdValid = false;
    return true;
  }
  return false;
}

bool isStopRequested() {
  if (gStopRequested) {
    gStopRequested = false;
    return true;
  }
  return false;
}

bool isStatusRequested() {
  if (gStatusRequested) {
    gStatusRequested = false;
    return true;
  }
  return false;
}

bool isLinkAlive() {
  const uint32_t timeout = 500;
  return (millis() - gLastRxMs) < timeout;
}

void printStats() {
  Serial.println("=== SERIAL MASTER COMM STATS ===");
  Serial.printf("TX Count: %lu\n", gTxCount);
  Serial.printf("RX Count: %lu\n", gRxCount);
  Serial.printf("RX Error: %lu\n", gRxErrorCount);
  Serial.printf("Last TX: %lu ms ago\n", millis() - gLastTxMs);
  Serial.printf("Last RX: %lu ms ago\n", millis() - gLastRxMs);
  Serial.printf("Link: %s\n", isLinkAlive() ? "ALIVE" : "TIMEOUT");
  Serial.println("================================");
}

}  // namespace SerialMaster
