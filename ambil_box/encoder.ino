/*
 * File: encoder.ino
 * Deskripsi: Handler encoder untuk semua motor dengan interrupt
 */

#include "armbox_config.h"

// ==========================================
// VARIABEL ENCODER - MOTOR PUTAR
// ==========================================
volatile long encoderPutar = 0;
volatile bool lastStatePutarA = LOW;
volatile bool lastStatePutarB = LOW;

// ==========================================
// VARIABEL ENCODER - MOTOR NAIK TURUN
// ==========================================
volatile long encoderNaikTurun = 0;
volatile bool lastStateNaikTurunA = LOW;
volatile bool lastStateNaikTurunB = LOW;

// ==========================================
// VARIABEL ENCODER - MOTOR MAJU MUNDUR
// ==========================================
volatile long encoderMajuMundur = 0;
volatile bool lastStateMajuMundurA = LOW;
volatile bool lastStateMajuMundurB = LOW;

// ==========================================
// INISIALISASI SEMUA ENCODER
// ==========================================
void initAllEncoders() {
  Serial.println("Inisialisasi Encoder...");
  
  // Setup pin encoder motor putar
  pinMode(MOTOR_PUTAR_ENCODER_A, INPUT_PULLUP);
  pinMode(MOTOR_PUTAR_ENCODER_B, INPUT_PULLUP);
  
  // Setup pin encoder motor naik turun
  pinMode(MOTOR_NAIK_TURUN_ENCODER_A, INPUT_PULLUP);
  pinMode(MOTOR_NAIK_TURUN_ENCODER_B, INPUT_PULLUP);
  
  // Setup pin encoder motor maju mundur
  pinMode(MOTOR_MAJU_MUNDUR_ENCODER_A, INPUT_PULLUP);
  pinMode(MOTOR_MAJU_MUNDUR_ENCODER_B, INPUT_PULLUP);
  
  // Attach interrupt untuk encoder A (channel A)
  attachInterrupt(digitalPinToInterrupt(MOTOR_PUTAR_ENCODER_A), 
                  isrEncoderPutar, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(MOTOR_NAIK_TURUN_ENCODER_A), 
                  isrEncoderNaikTurun, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(MOTOR_MAJU_MUNDUR_ENCODER_A), 
                  isrEncoderMajuMundur, CHANGE);
  
  // Reset semua encoder
  resetAllEncoders();
  
  Serial.println("✓ Encoder initialized");
}

// ==========================================
// ISR - MOTOR PUTAR
// ==========================================
void isrEncoderPutar() {
  bool stateA = digitalRead(MOTOR_PUTAR_ENCODER_A);
  bool stateB = digitalRead(MOTOR_PUTAR_ENCODER_B);
  
  if (stateA != lastStatePutarA) {
    if (stateA == stateB) {
      encoderPutar++;
    } else {
      encoderPutar--;
    }
  }
  
  lastStatePutarA = stateA;
  lastStatePutarB = stateB;
}

// ==========================================
// ISR - MOTOR NAIK TURUN
// ==========================================
void isrEncoderNaikTurun() {
  bool stateA = digitalRead(MOTOR_NAIK_TURUN_ENCODER_A);
  bool stateB = digitalRead(MOTOR_NAIK_TURUN_ENCODER_B);
  
  if (stateA != lastStateNaikTurunA) {
    if (stateA == stateB) {
      encoderNaikTurun++;
    } else {
      encoderNaikTurun--;
    }
  }
  
  lastStateNaikTurunA = stateA;
  lastStateNaikTurunB = stateB;
}

// ==========================================
// ISR - MOTOR MAJU MUNDUR
// ==========================================
void isrEncoderMajuMundur() {
  bool stateA = digitalRead(MOTOR_MAJU_MUNDUR_ENCODER_A);
  bool stateB = digitalRead(MOTOR_MAJU_MUNDUR_ENCODER_B);
  
  if (stateA != lastStateMajuMundurA) {
    if (stateA == stateB) {
      encoderMajuMundur++;
    } else {
      encoderMajuMundur--;
    }
  }
  
  lastStateMajuMundurA = stateA;
  lastStateMajuMundurB = stateB;
}

// ==========================================
// GETTER FUNCTIONS
// ==========================================
long getEncoderPutar() {
  return encoderPutar;
}

long getEncoderNaikTurun() {
  return encoderNaikTurun;
}

long getEncoderMajuMundur() {
  return encoderMajuMundur;
}

// ==========================================
// RESET FUNCTIONS
// ==========================================
void resetEncoderPutar() {
  encoderPutar = 0;
  Serial.println("Encoder Putar: Reset to 0");
}

void resetEncoderNaikTurun() {
  encoderNaikTurun = 0;
  Serial.println("Encoder Naik Turun: Reset to 0");
}

void resetEncoderMajuMundur() {
  encoderMajuMundur = 0;
  Serial.println("Encoder Maju Mundur: Reset to 0");
}

void resetAllEncoders() {
  encoderPutar = 0;
  encoderNaikTurun = 0;
  encoderMajuMundur = 0;
  Serial.println("All Encoders: Reset to 0");
}

// ==========================================
// SETTER FUNCTIONS (untuk kalibrasi manual)
// ==========================================
void setEncoderPutar(long value) {
  encoderPutar = value;
  Serial.print("Encoder Putar: Set to ");
  Serial.println(value);
}

void setEncoderNaikTurun(long value) {
  encoderNaikTurun = value;
  Serial.print("Encoder Naik Turun: Set to ");
  Serial.println(value);
}

void setEncoderMajuMundur(long value) {
  encoderMajuMundur = value;
  Serial.print("Encoder Maju Mundur: Set to ");
  Serial.println(value);
}

// ==========================================
// DEBUG - PRINT SEMUA ENCODER
// ==========================================
void printAllEncoders() {
  Serial.print("Encoders -> Putar: ");
  Serial.print(encoderPutar);
  Serial.print(" | Naik Turun: ");
  Serial.print(encoderNaikTurun);
  Serial.print(" | Maju Mundur: ");
  Serial.println(encoderMajuMundur);
}

// ==========================================
// KONVERSI ENCODER KE DERAJAT/MM
// ==========================================
float encoderToDegreePutar() {
  return (float)encoderPutar * 360.0 / ENCODER_PPR_PUTAR;
}

float encoderToMMNaikTurun() {
  // Asumsi: 1 putaran = PITCH mm (untuk lead screw)
  return (float)encoderNaikTurun * LEAD_SCREW_PITCH / ENCODER_PPR_NAIK_TURUN;
}

float encoderToMMMajuMundur() {
  return (float)encoderMajuMundur * LEAD_SCREW_PITCH / ENCODER_PPR_MAJU_MUNDUR;
}
