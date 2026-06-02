/*
 * File: arm_relay.ino
 * Deskripsi: Kontrol relay untuk perangkat tambahan
 */

#include "armbox_config.h"

// Status relay
bool relay1State = false;
bool relay2State = false;

// Inisialisasi relay
void initRelay() {
  // Set pin mode untuk relay
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  
  // Matikan semua relay pada awal
  digitalWrite(RELAY_1_PIN, LOW);
  digitalWrite(RELAY_2_PIN, LOW);
  
  relay1State = false;
  relay2State = false;
  
  Serial.println("Relay initialized");
  Serial.println("Relay 1: OFF");
  Serial.println("Relay 2: OFF");
}

// Fungsi untuk nyalakan relay 1
void relay1On() {
  digitalWrite(RELAY_1_PIN, HIGH);
  relay1State = true;
  Serial.println("Relay 1: ON");
}

// Fungsi untuk matikan relay 1
void relay1Off() {
  digitalWrite(RELAY_1_PIN, LOW);
  relay1State = false;
  Serial.println("Relay 1: OFF");
}

// Fungsi untuk toggle relay 1
void relay1Toggle() {
  if (relay1State) {
    relay1Off();
  } else {
    relay1On();
  }
}

// Fungsi untuk nyalakan relay 2
void relay2On() {
  digitalWrite(RELAY_2_PIN, HIGH);
  relay2State = true;
  Serial.println("Relay 2: ON");
}

// Fungsi untuk matikan relay 2
void relay2Off() {
  digitalWrite(RELAY_2_PIN, LOW);
  relay2State = false;
  Serial.println("Relay 2: OFF");
}

// Fungsi untuk toggle relay 2
void relay2Toggle() {
  if (relay2State) {
    relay2Off();
  } else {
    relay2On();
  }
}

// Fungsi untuk nyalakan semua relay
void allRelayOn() {
  relay1On();
  relay2On();
  Serial.println("Semua Relay: ON");
}

// Fungsi untuk matikan semua relay
void allRelayOff() {
  relay1Off();
  relay2Off();
  Serial.println("Semua Relay: OFF");
}

// Fungsi untuk kontrol relay berdasarkan nomor dan state
void controlRelay(int relayNum, bool state) {
  if (relayNum == 1) {
    if (state) {
      relay1On();
    } else {
      relay1Off();
    }
  } else if (relayNum == 2) {
    if (state) {
      relay2On();
    } else {
      relay2Off();
    }
  } else {
    Serial.print("Error: Relay ");
    Serial.print(relayNum);
    Serial.println(" tidak tersedia");
  }
}

// Fungsi untuk mendapatkan status relay
bool getRelay1State() {
  return relay1State;
}

bool getRelay2State() {
  return relay2State;
}

// Fungsi untuk pulse relay (nyala sebentar lalu mati)
void relay1Pulse(unsigned long duration) {
  Serial.print("Relay 1: Pulse ");
  Serial.print(duration);
  Serial.println(" ms");
  
  relay1On();
  delay(duration);
  relay1Off();
}

void relay2Pulse(unsigned long duration) {
  Serial.print("Relay 2: Pulse ");
  Serial.print(duration);
  Serial.println(" ms");
  
  relay2On();
  delay(duration);
  relay2Off();
}
