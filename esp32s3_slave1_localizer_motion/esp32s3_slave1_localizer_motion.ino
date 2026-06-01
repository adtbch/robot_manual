// ============================================================
// ESP32-S3 SLAVE 1 - LOCALIZER & MOTION CONTROL
// ============================================================
// Fungsi:
// 1. Menerima perintah motor dari Master via Serial
// 2. Kontrol 4 motor mecanum dengan PID
// 3. Kirim status RPM dan odometri ke Master
// ============================================================

#include "robot_config.h"

// ============================================================
// State BOOT button untuk trigger auto-tuner
// ============================================================
static uint32_t gBootPressStartMs = 0;
static bool gBootWasPressed = false;

// Paket terakhir untuk debugging/manual processing.
static EspNowControlPacket gLastRxPacket = {};

// Variabel untuk komunikasi serial dengan Master
static uint32_t gLastStatusSendMs = 0;
static uint32_t gLastOdomSendMs = 0;

// Variabel odometri (dummy untuk sekarang)
static float gPosX = 0.0f;
static float gPosY = 0.0f;
static float gHeading = 0.0f;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("===========================================");
  Serial.println("  ESP32-S3 SLAVE 1 - MOTION CONTROL");
  Serial.println("===========================================");
  
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  SetupMotors();
  pidControllerInit();
  setupEncoders();
  printSerialUsage();
  
  // Inisialisasi komunikasi serial dengan Master
  bool serialReady = SerialMaster::serialMasterInit();
  if (!serialReady) {
    Serial.println("ERROR: Serial Master comm init failed!");
  }
  
  Serial.println("===========================================");
  Serial.println("System ready!");
  Serial.println("Press BOOT pin for 3 seconds to start auto-tuner");
  Serial.println("===========================================");
}

void loop() {
  const uint32_t now = millis();
  
  // ========================================
  // 1. UPDATE ENCODER & RPM
  // ========================================
  convertEncoderToRPM();
  
  // ========================================
  // 2. AUTO-TUNER (jika aktif)
  // ========================================
  autoTunerTick(digitalRead(BOOT_BUTTON_PIN) == LOW);
  
  // ========================================
  // 3. SERIAL COMMUNICATION dengan Master
  // ========================================
  SerialMaster::serialMasterTick();
  
  // Terima perintah motor dari Master
  int16_t vx, vy, vtheta;
  if (SerialMaster::getMotorCommand(vx, vy, vtheta)) {
    Serial.printf("[MASTER RX] Motor: vx=%d vy=%d vtheta=%d\n", vx, vy, vtheta);
    
    // Konversi ke robot-centric dan jalankan motor
    driveRobotCentric(vx, vy, vtheta);
  }
  
  // Terima perintah stop dari Master
  if (SerialMaster::isStopRequested()) {
    Serial.println("[MASTER RX] STOP requested");
    motorStopAll();
  }
  
  // Terima request status dari Master
  if (SerialMaster::isStatusRequested()) {
    Serial.println("[MASTER RX] Status requested");

    SerialMaster::sendStatusReply(
      (uint16_t)abs(getEncoderVelocityRpm(0)),
      (uint16_t)abs(getEncoderVelocityRpm(1)),
      (uint16_t)abs(getEncoderVelocityRpm(2)),
      (uint16_t)abs(getEncoderVelocityRpm(3)),
      0x00
    );
  }
  
  // Kirim status RPM secara periodik (setiap 500ms)
  if (now - gLastStatusSendMs >= 500) {
    gLastStatusSendMs = now;

    SerialMaster::sendStatusReply(
      (uint16_t)abs(getEncoderVelocityRpm(0)),
      (uint16_t)abs(getEncoderVelocityRpm(1)),
      (uint16_t)abs(getEncoderVelocityRpm(2)),
      (uint16_t)abs(getEncoderVelocityRpm(3)),
      0x00
    );
  }
  
  // Kirim odometri secara periodik (setiap 100ms)
  if (now - gLastOdomSendMs >= 100) {
    gLastOdomSendMs = now;
    
    // TODO: Hitung odometri dari encoder
    // Untuk sekarang kirim dummy data
    SerialMaster::sendOdometryData(gPosX, gPosY, gHeading);
  }
  
  // ========================================
  // 4. SAFETY: Stop motor jika link timeout
  // ========================================
  if (!SerialMaster::isLinkAlive()) {
    static uint32_t lastSafetyStopMs = 0;
    if (now - lastSafetyStopMs >= 500) {
      lastSafetyStopMs = now;
      motorStopAll();
      Serial.println("[SAFETY] Master link timeout, stopping motor");
    }
  }
  
  // ========================================
  // 5. SERIAL COMMANDS dari USB (untuk debugging)
  // ========================================
  processSerialCommands();
  serialCommandsTick();
  serialContinuousTick();
}
