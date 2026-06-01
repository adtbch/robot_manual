// ============================================================
// ESP32-S3 MASTER MANIPULATOR 1
// ============================================================
// Fungsi:
// 1. Menerima input dari controller via ESP-NOW
// 2. Mengirim perintah motor ke Slave1 via Serial
// 3. Menerima status dari Slave1 via Serial
// ============================================================

#include "robot_config.h"

static EspNowControlPacket gLastRxPacket = {};
static uint32_t gLastSerialSendMs = 0;
static uint32_t gLastStatusRequestMs = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("===========================================");
  Serial.println("  ESP32-S3 MASTER MANIPULATOR 1");
  Serial.println("===========================================");
  
  // Inisialisasi ESP-NOW receiver
  bool espNowReady = espNowControlInit();
  if (!espNowReady) {
    Serial.println("ERROR: ESP-NOW init failed!");
  }
  
  // Inisialisasi Serial komunikasi dengan Slave1
  bool serialReady = SerialSlave1::serialSlave1Init();
  if (!serialReady) {
    Serial.println("ERROR: Serial Slave1 init failed!");
  }
  
  Serial.println("===========================================");
  Serial.println("System ready!");
  Serial.println("===========================================");
}

void loop() {
  const uint32_t now = millis();
  
  // ========================================
  // 1. UPDATE ESP-NOW (Terima dari Controller)
  // ========================================
  espNowControlTick();
  
  if (espNowControlReadPacket(gLastRxPacket)) {
    Serial.printf("[CONTROLLER] seq=%u x=%d y=%d w=%d connected=%u\n",
                  gLastRxPacket.seq,
                  gLastRxPacket.x,
                  gLastRxPacket.y,
                  gLastRxPacket.w,
                  gLastRxPacket.connected);
    
    // Kirim perintah motor ke Slave1 setiap 50ms
    if (now - gLastSerialSendMs >= 50) {
      gLastSerialSendMs = now;
      
      if (gLastRxPacket.connected) {
        // Konversi joystick ke velocity
        int16_t vx = gLastRxPacket.x * 4;      // Scale -127..127 -> -508..508
        int16_t vy = gLastRxPacket.y * 4;
        int16_t vtheta = gLastRxPacket.w * 4;
        
        // Kirim ke Slave1
        if (SerialSlave1::sendMotorControl(vx, vy, vtheta)) {
          Serial.printf("[SLAVE1 TX] vx=%d vy=%d vtheta=%d\n", vx, vy, vtheta);
        }
      } else {
        // Controller disconnect, stop motor
        SerialSlave1::sendMotorStop();
        Serial.println("[SLAVE1 TX] STOP");
      }
    }
  }
  
  // ========================================
  // 2. UPDATE SERIAL SLAVE1 (Terima dari Slave1)
  // ========================================
  SerialSlave1::serialSlave1Tick();
  
  // Cek status dari Slave1
  SerialSlave1::StatusReplyPacket status = {};
  if (SerialSlave1::getStatus(status)) {
    Serial.printf("[SLAVE1 RX] RPM: %u %u %u %u status=0x%02X\n",
                  status.rpmMotor1,
                  status.rpmMotor2,
                  status.rpmMotor3,
                  status.rpmMotor4,
                  status.status);
  }
  
  // Cek odometry dari Slave1
  SerialSlave1::OdometryDataPacket odom = {};
  if (SerialSlave1::getOdometry(odom)) {
    Serial.printf("[SLAVE1 RX] Odom: X=%.2f Y=%.2f H=%.2f\n",
                  odom.posX,
                  odom.posY,
                  odom.heading);
  }
  
  // Request status setiap 1 detik
  if (now - gLastStatusRequestMs >= 1000) {
    gLastStatusRequestMs = now;
    SerialSlave1::sendStatusRequest();
  }
  
  // ========================================
  // 3. SAFETY: Stop motor jika link timeout
  // ========================================
  if (!espNowControlIsLinkAlive() && !SerialSlave1::isLinkAlive()) {
    static uint32_t lastSafetyStopMs = 0;
    if (now - lastSafetyStopMs >= 500) {
      lastSafetyStopMs = now;
      SerialSlave1::sendMotorStop();
      Serial.println("[SAFETY] All links timeout, stopping motor");
    }
  }
}
