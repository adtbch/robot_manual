/*
 * File: homing.ino
 * Deskripsi: Proses homing untuk semua motor
 */

#include "armbox_config.h"

void setMotorPutarRaw(int speedSigned);
void setMotorNaikTurunRaw(int speedSigned);
void setMotorMajuMundurRaw(int speedSigned);

// Status homing
bool isHomingPutarDone = false;
bool isHomingNaikTurunDone = false;
bool isHomingMajuMundurDone = false;

// ==========================================
// HOMING - MOTOR PUTAR
// ==========================================
bool homingMotorPutar() {
  Serial.println("\n=== HOMING MOTOR PUTAR ===");
  Serial.println("Mencari limit switch...");
  
  isHomingPutarDone = false;
  
  // Fase 1: Cari limit switch dengan kecepatan sedang
  unsigned long timeout = millis() + HOMING_TIMEOUT;
  
  while (digitalRead(MOTOR_PUTAR_LIMIT) == HIGH) {
    // Timeout protection
    if (millis() > timeout) {
      Serial.println("✗ ERROR: Homing timeout!");
      stopMotorPutar();
      return false;
    }
    
    // Gerak ke arah limit switch
    setMotorPutarRaw(-HOMING_SPEED);
    delay(10);
  }
  
  // Stop saat limit switch tertekan
  stopMotorPutar();
  delay(200);
  
  Serial.println("Limit switch terdeteksi");
  
  // Fase 2: Mundur sedikit dari limit switch
  Serial.println("Mundur dari limit switch...");
  unsigned long backoffTime = millis() + 500; // Mundur 500ms
  
  while (millis() < backoffTime) {
    setMotorPutarRaw(HOMING_SPEED / 2);
    delay(10);
  }
  
  stopMotorPutar();
  delay(200);

  // Fase 3: Approach pelan ke limit switch
  Serial.println("Approach pelan...");
  
  while (digitalRead(MOTOR_PUTAR_LIMIT) == HIGH) {
    setMotorPutarRaw(-HOMING_SPEED / 3);
    delay(10);
  }
  
  stopMotorPutar();
  delay(100);
  
  // Reset encoder di posisi home
  resetEncoderPutar();
  
  isHomingPutarDone = true;
  Serial.println("✓ HOMING MOTOR PUTAR SELESAI\n");
  
  return true;
}

// ==========================================
// HOMING - MOTOR NAIK TURUN
// ==========================================
bool homingMotorNaikTurun() {
  Serial.println("\n=== HOMING MOTOR NAIK TURUN ===");
  Serial.println("Mencari limit switch...");
  
  isHomingNaikTurunDone = false;
  
  // Fase 1: Cari limit switch dengan kecepatan sedang
  unsigned long timeout = millis() + HOMING_TIMEOUT;
  
  while (digitalRead(MOTOR_NAIK_TURUN_LIMIT) == HIGH) {
    // Timeout protection
    if (millis() > timeout) {
      Serial.println("✗ ERROR: Homing timeout!");
      stopMotorNaikTurun();
      return false;
    }
    
    // Turun ke arah limit switch
    setMotorNaikTurunRaw(-HOMING_SPEED);
    delay(10);
  }
  
  // Stop saat limit switch tertekan
  stopMotorNaikTurun();
  delay(200);
  
  Serial.println("Limit switch terdeteksi");

  // Fase 2: Naik sedikit dari limit switch
  Serial.println("Naik dari limit switch...");
  unsigned long backoffTime = millis() + 500;
  
  while (millis() < backoffTime) {
    setMotorNaikTurunRaw(HOMING_SPEED / 2);
    delay(10);
  }
  
  stopMotorNaikTurun();
  delay(200);
  
  // Fase 3: Approach pelan ke limit switch
  Serial.println("Approach pelan...");
  
  while (digitalRead(MOTOR_NAIK_TURUN_LIMIT) == HIGH) {
    setMotorNaikTurunRaw(-HOMING_SPEED / 3);
    delay(10);
  }
  
  stopMotorNaikTurun();
  delay(100);
  
  // Reset encoder di posisi home
  resetEncoderNaikTurun();
  
  isHomingNaikTurunDone = true;
  Serial.println("✓ HOMING MOTOR NAIK TURUN SELESAI\n");
  
  return true;
}

// ==========================================
// HOMING - MOTOR MAJU MUNDUR
// ==========================================
bool homingMotorMajuMundur() {
  Serial.println("\n=== HOMING MOTOR MAJU MUNDUR ===");
  Serial.println("Mencari limit switch...");
  
  isHomingMajuMundurDone = false;
  
  // Fase 1: Cari limit switch
  unsigned long timeout = millis() + HOMING_TIMEOUT;
  
  while (digitalRead(MOTOR_MAJU_MUNDUR_LIMIT) == HIGH) {
    if (millis() > timeout) {
      Serial.println("✗ ERROR: Homing timeout!");
      stopMotorMajuMundur();
      return false;
    }
    
    // Mundur ke arah limit switch
    setMotorMajuMundurRaw(-HOMING_SPEED);
    delay(10);
  }
  
  stopMotorMajuMundur();
  delay(200);
  
  Serial.println("Limit switch terdeteksi");

  // Fase 2: Maju sedikit dari limit switch
  Serial.println("Maju dari limit switch...");
  unsigned long backoffTime = millis() + 500;
  
  while (millis() < backoffTime) {
    setMotorMajuMundurRaw(HOMING_SPEED / 2);
    delay(10);
  }
  
  stopMotorMajuMundur();
  delay(200);
  
  // Fase 3: Approach pelan ke limit switch
  Serial.println("Approach pelan...");
  
  while (digitalRead(MOTOR_MAJU_MUNDUR_LIMIT) == HIGH) {
    setMotorMajuMundurRaw(-HOMING_SPEED / 3);
    delay(10);
  }
  
  stopMotorMajuMundur();
  delay(100);
  
  // Reset encoder di posisi home
  resetEncoderMajuMundur();
  
  isHomingMajuMundurDone = true;
  Serial.println("✓ HOMING MOTOR MAJU MUNDUR SELESAI\n");
  
  return true;
}

// ==========================================
// HOMING SEMUA MOTOR
// ==========================================
bool homingAllMotors() {
  Serial.println("\n");
  Serial.println("========================================");
  Serial.println("   MEMULAI HOMING SEMUA MOTOR");
  Serial.println("========================================\n");
  
  unsigned long startTime = millis();
  
  // Homing motor putar
  if (!homingMotorPutar()) {
    Serial.println("✗ Homing Motor Putar GAGAL!");
    return false;
  }
  delay(500);
  
  // Homing motor naik turun
  if (!homingMotorNaikTurun()) {
    Serial.println("✗ Homing Motor Naik Turun GAGAL!");
    return false;
  }
  delay(500);
  
  // Homing motor maju mundur
  if (!homingMotorMajuMundur()) {
    Serial.println("✗ Homing Motor Maju Mundur GAGAL!");
    return false;
  }
  delay(500);
  
  unsigned long elapsedTime = (millis() - startTime) / 1000;
  
  Serial.println("========================================");
  Serial.println("   ✓ HOMING SELESAI!");
  Serial.print("   Waktu: ");
  Serial.print(elapsedTime);
  Serial.println(" detik");
  Serial.println("========================================\n");
  
  // Servo ke home position
  servoHome();
  
  return true;
}

// ==========================================
// CHECK HOMING STATUS
// ==========================================
bool isAllMotorHomed() {
  return (isHomingPutarDone && 
          isHomingNaikTurunDone && 
          isHomingMajuMundurDone);
}

bool isMotorPutarHomed() {
  return isHomingPutarDone;
}

bool isMotorNaikTurunHomed() {
  return isHomingNaikTurunDone;
}

bool isMotorMajuMundurHomed() {
  return isHomingMajuMundurDone;
}

// ==========================================
// PRINT HOMING STATUS
// ==========================================
void printHomingStatus() {
  Serial.println("\n=== HOMING STATUS ===");
  Serial.print("Motor Putar      : ");
  Serial.println(isHomingPutarDone ? "✓ HOMED" : "✗ NOT HOMED");
  
  Serial.print("Motor Naik Turun : ");
  Serial.println(isHomingNaikTurunDone ? "✓ HOMED" : "✗ NOT HOMED");
  
  Serial.print("Motor Maju Mundur: ");
  Serial.println(isHomingMajuMundurDone ? "✓ HOMED" : "✗ NOT HOMED");
  
  Serial.print("Semua Motor      : ");
  Serial.println(isAllMotorHomed() ? "✓ READY" : "✗ NOT READY");
  Serial.println("=====================\n");
}

// ==========================================
// RESET HOMING STATUS
// ==========================================
void resetHomingStatus() {
  isHomingPutarDone = false;
  isHomingNaikTurunDone = false;
  isHomingMajuMundurDone = false;
  Serial.println("Homing status: Reset");
}
