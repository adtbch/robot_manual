/*
 * File: ambil_box.ino (MASTER FILE)
 * Deskripsi: File master untuk kontrol sistem arm box
 * Hardware: Arduino Mega / ESP32
 * 
 * Pin Configuration:
 * - Motor Putar: RPWM=15, LPWM=16, Encoder=40,39, Limit=3
 * - Motor Naik Turun: RPWM=6, LPWM=7, Encoder=41,42, Limit=11
 * - Motor Maju Mundur: RPWM=4, LPWM=5, Encoder=1,2, Limit=10
 * - Relay: Relay1=12, Relay2=13
 * - Servo: Pin=38
 * - UART: RX=38, TX=21
 * 
 * Author: Robot Manual Team
 * Date: 2 Juni 2026
 */

#include "armbox_config.h"

// State machine
enum SystemState {
  STATE_IDLE,
  STATE_HOMING,
  STATE_MANUAL,
  STATE_AUTO,
  STATE_ERROR
};

SystemState currentState = STATE_IDLE;
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_INTERVAL = 1000; // Update status setiap 1 detik

void setup() {
  // ====== EMERGENCY SAFETY: Force ALL motor pins to SAFE STATE immediately ======
  pinMode(MOTOR_PUTAR_RPWM, OUTPUT);
  pinMode(MOTOR_PUTAR_LPWM, OUTPUT);
  pinMode(MOTOR_NAIK_TURUN_RPWM, OUTPUT);
  pinMode(MOTOR_NAIK_TURUN_LPWM, OUTPUT);
  pinMode(MOTOR_MAJU_MUNDUR_RPWM, OUTPUT);
  pinMode(MOTOR_MAJU_MUNDUR_LPWM, OUTPUT);
  
  digitalWrite(MOTOR_PUTAR_RPWM, LOW);
  digitalWrite(MOTOR_PUTAR_LPWM, LOW);
  digitalWrite(MOTOR_NAIK_TURUN_RPWM, LOW);
  digitalWrite(MOTOR_NAIK_TURUN_LPWM, LOW);
  digitalWrite(MOTOR_MAJU_MUNDUR_RPWM, LOW);
  digitalWrite(MOTOR_MAJU_MUNDUR_LPWM, LOW);
  
  delay(500); // Give time for motor driver to stabilize
  // =============================================================================
  
  // Inisialisasi Serial Monitor
  Serial.begin(115200);
  delay(100);
  
  Serial.println("========================================");
  Serial.println("    SISTEM ARM BOX - AMBIL BOX");
  Serial.println("========================================");
  Serial.println("SAFETY: All motors forced STOP at boot");
  Serial.println("Versi: 2.0");
  Serial.println("Tanggal: 2 Juni 2026");
  Serial.println("========================================");
  
  // Inisialisasi semua komponen
  Serial.println("\nMemulai inisialisasi sistem...");
  
  // 1. Inisialisasi Encoder (harus pertama)
  initAllEncoders();
  delay(100);
  
  // 2. Inisialisasi Motor
  initArmPutar();
  delay(100);
  
  initArmNaikTurun();
  delay(100);
  
  initArmMajuMundur();
  delay(100);
  
  // 3. Inisialisasi Servo
  initArmServo();
  delay(100);
  
  // 4. Inisialisasi Relay
  initRelay();
  delay(100);
  
  // 5. Inisialisasi UART
  initUART();
  delay(100);

  // 6. Inisialisasi kontrol manual via Serial Monitor
  initSerialMonitorControl();
  delay(100);
  
  Serial.println("\n========================================");
  Serial.println("Inisialisasi selesai!");
  Serial.println("Sistem siap digunakan");
  Serial.println("========================================\n");
  
  Serial.println("PERINGATAN: Lakukan HOMING sebelum operasi!");
  Serial.println("Kirim perintah: HOMING\n");
  
  // Set state ke IDLE
  currentState = STATE_IDLE;
}

void loop() {
  // Baca perintah dari UART
  readUART();
  processCommand();

  // Baca perintah dari Serial Monitor
  readSerialMonitor();
  processSerialMonitorCommand();
  
  // Update status periodik
  if (millis() - lastStatusUpdate >= STATUS_INTERVAL) {
    printSystemStatus();
    lastStatusUpdate = millis();
  }
  
  // State machine
  switch (currentState) {
    case STATE_IDLE:
      // Tunggu perintah
      break;
      
    case STATE_HOMING:
      // Proses homing sedang berjalan
      break;
      
    case STATE_MANUAL:
      // Mode manual control
      break;
      
    case STATE_AUTO:
      // Mode automatic sequence
      runAutoSequence();
      break;
      
    case STATE_ERROR:
      // Error handling
      handleError();
      break;
  }
}

// Fungsi untuk print status sistem
void printSystemStatus() {
  // Hanya print jika ada perubahan signifikan atau diminta
  // Untuk menghindari spam di serial monitor
}

// Fungsi untuk menjalankan sequence otomatis
void runAutoSequence() {
  // Cek apakah sudah homing
  if (!isAllMotorHomed()) {
    Serial.println("ERROR: Sistem belum di-homing!");
    Serial.println("Lakukan homing terlebih dahulu");
    currentState = STATE_IDLE;
    return;
  }
  
  Serial.println("\n=== Memulai Sequence Otomatis ===");
  
  // 1. Buka gripper
  Serial.println("Step 1: Membuka gripper...");
  bukaGripper();
  delay(1000);
  
  // 2. Gerak ke posisi ambil
  Serial.println("Step 2: Bergerak ke posisi ambil...");
  putarKePosisi(500, 200);
  delay(500);
  majuMundurKePosisi(800, 200);
  delay(500);
  naikTurunKePosisi(300, 200);
  delay(500);
  
  // 3. Tutup gripper (ambil box)
  Serial.println("Step 3: Mengambil box...");
  tutupGripper();
  delay(1000);
  
  // 4. Angkat box
  Serial.println("Step 4: Mengangkat box...");
  naikTurunKePosisi(0, 200);
  delay(500);
  
  // 5. Putar ke posisi drop
  Serial.println("Step 5: Memutar ke posisi drop...");
  putarKePosisi(1000, 200);
  delay(500);
  
  // 6. Turunkan box
  Serial.println("Step 6: Menurunkan box...");
  naikTurunKePosisi(300, 200);
  delay(500);
  
  // 7. Buka gripper (lepas box)
  Serial.println("Step 7: Melepas box...");
  bukaGripper();
  delay(1000);
  
  // 8. Kembali ke home
  Serial.println("Step 8: Kembali ke home...");
  naikTurunKePosisi(0, 200);
  delay(500);
  majuMundurKePosisi(0, 200);
  delay(500);
  putarKePosisi(0, 200);
  delay(500);
  servoHome();
  
  Serial.println("=== Sequence Selesai ===\n");
  
  currentState = STATE_IDLE;
}

// Fungsi untuk error handling
void handleError() {
  Serial.println("ERROR: Sistem dalam kondisi error!");
  
  // Stop semua motor
  stopMotorPutar();
  stopMotorNaikTurun();
  stopMotorMajuMundur();
  
  // Matikan semua relay
  allRelayOff();
  
  // Tunggu reset manual
  Serial.println("Kirim perintah 'RESET' untuk keluar dari error state");
  
  delay(1000);
}

// Fungsi untuk demo test semua komponen
void testAllComponents() {
  Serial.println("\n=== TEST SEMUA KOMPONEN ===");
  
  // Test Motor Putar
  Serial.println("\n1. Test Motor Putar:");
  putarKanan(150);
  delay(2000);
  stopMotorPutar();
  delay(500);
  putarKiri(150);
  delay(2000);
  stopMotorPutar();
  delay(1000);
  
  // Test Motor Naik Turun
  Serial.println("\n2. Test Motor Naik Turun:");
  motorNaik(150);
  delay(2000);
  stopMotorNaikTurun();
  delay(500);
  motorTurun(150);
  delay(2000);
  stopMotorNaikTurun();
  delay(1000);
  
  // Test Motor Maju Mundur
  Serial.println("\n3. Test Motor Maju Mundur:");
  motorMaju(150);
  delay(2000);
  stopMotorMajuMundur();
  delay(500);
  motorMundur(150);
  delay(2000);
  stopMotorMajuMundur();
  delay(1000);
  
  // Test Servo
  Serial.println("\n4. Test Servo:");
  bukaGripper();
  delay(2000);
  tutupGripper();
  delay(2000);
  servoHome();
  delay(1000);
  
  // Test Relay
  Serial.println("\n5. Test Relay:");
  relay1On();
  delay(1000);
  relay1Off();
  delay(500);
  relay2On();
  delay(1000);
  relay2Off();
  delay(1000);
  
  Serial.println("\n=== TEST SELESAI ===\n");
}
