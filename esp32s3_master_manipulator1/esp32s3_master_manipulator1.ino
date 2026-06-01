// ============================================================
// ESP32-S3 MASTER MANIPULATOR 1
// ============================================================
// Fungsi:
// 1. Menerima input dari controller via ESP-NOW
// 2. Mengirim perintah motor roda ke Slave1 via Serial
// 3. Kontrol motor lengan sumbu X & Z dengan homing
// ============================================================

#include "robot_config.h"
#include "arm_config.h"

static EspNowControlPacket gLastRxPacket = {};
static uint32_t gLastSerialSendMs = 0;
static uint32_t gLastStatusRequestMs = 0;
static uint32_t gLastDebugMs = 0;

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

  // Inisialisasi encoder lengan
  armEncoderInit();

  // Inisialisasi motor lengan
  armMotorInit();

  // Inisialisasi servo
  armServoInit();

  // Mulai homing otomatis saat boot
  Serial.println("===========================================");
  Serial.println("Memulai homing sequence...");
  armHomingStart();
}

void loop() {
  const uint32_t now = millis();

  // ========================================
  // 1. ARM HOMING (prioritas tertinggi)
  // ========================================
  armHomingTick();

  // ========================================
  // 2. ARM MOVEMENT (jika tidak sedang homing)
  // ========================================
  if (!armHomingIsRunning()) {
    armMovementTick();
  }

  // ========================================
  // 3. UPDATE ESP-NOW (Terima dari Controller)
  // ========================================
  espNowControlTick();

  if (espNowControlReadPacket(gLastRxPacket)) {
    if (now - gLastSerialSendMs >= 50) {
      gLastSerialSendMs = now;

      if (gLastRxPacket.connected) {
        // Kirim perintah roda ke Slave1
        int16_t vx     = gLastRxPacket.x * 4;
        int16_t vy     = gLastRxPacket.y * 4;
        int16_t vtheta = gLastRxPacket.w * 4;
        SerialSlave1::sendMotorControl(vx, vy, vtheta);

        // Kontrol lengan dari joystick kanan (rx/ry) - hanya jika sudah homed
        if (armHomingIsComplete()) {
          // rx = sumbu X lengan, ry = sumbu Z lengan
          int armXPwm = map(gLastRxPacket.rx, -127, 127, -ARM_PWM_MAX, ARM_PWM_MAX);
          int armZPwm = map(gLastRxPacket.ry, -127, 127, -ARM_PWM_MAX, ARM_PWM_MAX);

          // Deadzone kecil untuk joystick
          if (abs(armXPwm) < 30) armXPwm = 0;
          if (abs(armZPwm) < 30) armZPwm = 0;

          armMotorSetPWM(motorX, armXPwm);
          armMotorSetPWM(motorZ, armZPwm);
        }

        // ----------------------------------------
        // KONTROL SERVO
        // ----------------------------------------

        // L2 / R2 = rotasi servo capit (analog 0-255)
        // L2 geser ke kiri, R2 geser ke kanan
        {
          static uint32_t lastServoRotateMs = 0;
          if (now - lastServoRotateMs >= 30) {  // Update 30ms sekali
            lastServoRotateMs = now;

            int currentAngle = armServoRotateGetAngle();
            // R2 = putar kanan, L2 = putar kiri, kecepatan proporsional
            int delta = (gLastRxPacket.r2Value / 50) - (gLastRxPacket.l2Value / 50);
            if (delta != 0) {
              armServoRotateSetAngle(currentAngle + delta);
            }
          }
        }

        // R1 = tutup capit, L1 = buka capit (toggle)
        {
          static uint32_t lastGripperMs = 0;
          static bool lastR1State = false;
          static bool lastL1State = false;

          bool r1Pressed = (gLastRxPacket.buttons & BTN_R1) != 0;
          bool l1Pressed = (gLastRxPacket.buttons & BTN_L1) != 0;

          // Edge detection: hanya trigger saat tombol baru ditekan
          if (now - lastGripperMs >= 200) {  // Debounce 200ms
            if (r1Pressed && !lastR1State) {
              armServoGripperClose();
              lastGripperMs = now;
            } else if (l1Pressed && !lastL1State) {
              armServoGripperOpen();
              lastGripperMs = now;
            }
          }
          lastR1State = r1Pressed;
          lastL1State = l1Pressed;
        }

        // Tombol OPTIONS = homing ulang
        if (gLastRxPacket.buttons & BTN_OPTIONS) {
          Serial.println("[CONTROLLER] Homing dipicu dari controller");
          armHomingStart();
        }

      } else {
        // Controller disconnect: stop semua
        SerialSlave1::sendMotorStop();
        armMotorStopAll();
      }
    }
  }

  // ========================================
  // 4. UPDATE SERIAL SLAVE1
  // ========================================
  SerialSlave1::serialSlave1Tick();

  SerialSlave1::StatusReplyPacket status = {};
  if (SerialSlave1::getStatus(status)) {
    Serial.printf("[SLAVE1] RPM: %u %u %u %u\n",
                  status.rpmMotor1, status.rpmMotor2,
                  status.rpmMotor3, status.rpmMotor4);
  }

  SerialSlave1::OdometryDataPacket odom = {};
  if (SerialSlave1::getOdometry(odom)) {
    Serial.printf("[SLAVE1] Odom: X=%.2f Y=%.2f H=%.2f\n",
                  odom.posX, odom.posY, odom.heading);
  }

  if (now - gLastStatusRequestMs >= 1000) {
    gLastStatusRequestMs = now;
    SerialSlave1::sendStatusRequest();
  }

  // ========================================
  // 5. DEBUG LENGAN setiap 2 detik
  // ========================================
  if (now - gLastDebugMs >= 2000) {
    gLastDebugMs = now;
    Serial.printf("[ARM] X=%ld homed=%d | Z=%ld homed=%d | homingRun=%d\n",
                  armEncoderGetCount(motorX), motorX.encoder.isHomed,
                  armEncoderGetCount(motorZ), motorZ.encoder.isHomed,
                  armHomingIsRunning());
    Serial.printf("[SERVO] Rotate=%d deg | Gripper=%d deg (%s)\n",
                  armServoRotateGetAngle(),
                  armServoGripperGetAngle(),
                  armServoGripperIsOpen() ? "OPEN" : "CLOSED");
  }

  // ========================================
  // 6. SAFETY: Stop semua jika semua link timeout
  // ========================================
  if (!espNowControlIsLinkAlive() && !SerialSlave1::isLinkAlive()) {
    static uint32_t lastSafetyStopMs = 0;
    if (now - lastSafetyStopMs >= 500) {
      lastSafetyStopMs = now;
      SerialSlave1::sendMotorStop();
      if (!armHomingIsRunning()) {
        armMotorStopAll();
      }
      Serial.println("[SAFETY] Link timeout, stopping all");
    }
  }
}

