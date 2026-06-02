/*
 * File: uart_communication.ino
 * Deskripsi: Komunikasi UART untuk kontrol arm box
 * Pin: RX = 38, TX = 21
 */

#include "armbox_config.h"

// Buffer untuk menerima data
String inputString = "";
bool stringComplete = false;

// Inisialisasi UART
void initUART() {
  // Inisialisasi Serial1 dengan baud rate 115200
#if defined(ESP32)
  Serial1.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
#else
  Serial1.begin(115200);
#endif
  
  inputString.reserve(200);
  
  Serial.println("UART Communication initialized");
  Serial.print("RX Pin: ");
  Serial.print(UART_RX_PIN);
  Serial.print(", TX Pin: ");
  Serial.println(UART_TX_PIN);
  Serial.println("Baud Rate: 115200");
}

// Fungsi untuk membaca data UART
void readUART() {
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
    
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

// Fungsi untuk parsing dan eksekusi perintah
void processCommand() {
  if (stringComplete) {
    inputString.trim();
    Serial.print("Command diterima: ");
    Serial.println(inputString);
    
    // Parse perintah
    if (inputString.startsWith("PUTAR_")) {
      handlePutarCommand(inputString);
    }
    else if (inputString.startsWith("NAIK_TURUN_")) {
      handleNaikTurunCommand(inputString);
    }
    else if (inputString.startsWith("MAJU_MUNDUR_")) {
      handleMajuMundurCommand(inputString);
    }
    else if (inputString.startsWith("SERVO_")) {
      handleServoCommand(inputString);
    }
    else if (inputString.startsWith("RELAY_")) {
      handleRelayCommand(inputString);
    }
    else if (inputString.startsWith("HOMING_ALL")) {
      if (homingAllMotors()) {
        Serial1.println("OK:HOMING_ALL_COMPLETE");
      } else {
        Serial1.println("ERROR:HOMING_FAILED");
      }
    }
    else if (inputString.startsWith("HOMING_STATUS")) {
      printHomingStatus();
    }
    else if (inputString.startsWith("HOMING")) {
      handleHomingCommand();
    }
    else if (inputString.startsWith("ENCODER_STATUS")) {
      printAllEncoders();
    }
    else if (inputString.startsWith("STATUS")) {
      sendStatus();
    }
    else if (inputString.startsWith("STOP")) {
      emergencyStop();
    }
    else {
      Serial1.println("ERROR:UNKNOWN_COMMAND");
      Serial.println("Error: Perintah tidak dikenali");
    }
    
    // Clear buffer
    inputString = "";
    stringComplete = false;
  }
}

// Handler untuk perintah motor putar
void handlePutarCommand(String cmd) {
  if (cmd == "PUTAR_KANAN") {
    putarKanan(200);
    Serial1.println("OK:PUTAR_KANAN");
  }
  else if (cmd == "PUTAR_KIRI") {
    putarKiri(200);
    Serial1.println("OK:PUTAR_KIRI");
  }
  else if (cmd == "PUTAR_STOP") {
    stopMotorPutar();
    Serial1.println("OK:PUTAR_STOP");
  }
  else if (cmd.startsWith("PUTAR_POS_")) {
    int pos = cmd.substring(10).toInt();
    putarKePosisi(pos, 180);
    Serial1.println("OK:PUTAR_POS");
  }
  else if (cmd == "PUTAR_HOME") {
    homingMotorPutar();
    Serial1.println("OK:PUTAR_HOME");
  }
}

// Handler untuk perintah motor naik turun
void handleNaikTurunCommand(String cmd) {
  if (cmd == "NAIK_TURUN_NAIK") {
    motorNaik(200);
    Serial1.println("OK:NAIK");
  }
  else if (cmd == "NAIK_TURUN_TURUN") {
    motorTurun(200);
    Serial1.println("OK:TURUN");
  }
  else if (cmd == "NAIK_TURUN_STOP") {
    stopMotorNaikTurun();
    Serial1.println("OK:NAIK_TURUN_STOP");
  }
  else if (cmd.startsWith("NAIK_TURUN_POS_")) {
    int pos = cmd.substring(15).toInt();
    naikTurunKePosisi(pos, 180);
    Serial1.println("OK:NAIK_TURUN_POS");
  }
  else if (cmd == "NAIK_TURUN_HOME") {
    homingMotorNaikTurun();
    Serial1.println("OK:NAIK_TURUN_HOME");
  }
}

// Handler untuk perintah motor maju mundur
void handleMajuMundurCommand(String cmd) {
  if (cmd == "MAJU_MUNDUR_MAJU") {
    motorMaju(200);
    Serial1.println("OK:MAJU");
  }
  else if (cmd == "MAJU_MUNDUR_MUNDUR") {
    motorMundur(200);
    Serial1.println("OK:MUNDUR");
  }
  else if (cmd == "MAJU_MUNDUR_STOP") {
    stopMotorMajuMundur();
    Serial1.println("OK:MAJU_MUNDUR_STOP");
  }
  else if (cmd.startsWith("MAJU_MUNDUR_POS_")) {
    int pos = cmd.substring(16).toInt();
    majuMundurKePosisi(pos, 180);
    Serial1.println("OK:MAJU_MUNDUR_POS");
  }
  else if (cmd == "MAJU_MUNDUR_HOME") {
    homingMotorMajuMundur();
    Serial1.println("OK:MAJU_MUNDUR_HOME");
  }
}

// Handler untuk perintah servo
void handleServoCommand(String cmd) {
  if (cmd == "SERVO_BUKA") {
    bukaGripper();
    Serial1.println("OK:SERVO_BUKA");
  }
  else if (cmd == "SERVO_TUTUP") {
    tutupGripper();
    Serial1.println("OK:SERVO_TUTUP");
  }
  else if (cmd == "SERVO_HOME") {
    servoHome();
    Serial1.println("OK:SERVO_HOME");
  }
  else if (cmd == "SERVO_SETENGAH") {
    gripperSetengah();
    Serial1.println("OK:SERVO_SETENGAH");
  }
  else if (cmd.startsWith("SERVO_ANGLE_")) {
    int angle = cmd.substring(12).toInt();
    setServoAngle(angle);
    Serial1.println("OK:SERVO_ANGLE");
  }
}

// Handler untuk perintah relay
void handleRelayCommand(String cmd) {
  if (cmd == "RELAY_1_ON") {
    relay1On();
    Serial1.println("OK:RELAY_1_ON");
  }
  else if (cmd == "RELAY_1_OFF") {
    relay1Off();
    Serial1.println("OK:RELAY_1_OFF");
  }
  else if (cmd == "RELAY_2_ON") {
    relay2On();
    Serial1.println("OK:RELAY_2_ON");
  }
  else if (cmd == "RELAY_2_OFF") {
    relay2Off();
    Serial1.println("OK:RELAY_2_OFF");
  }
  else if (cmd == "RELAY_ALL_ON") {
    allRelayOn();
    Serial1.println("OK:RELAY_ALL_ON");
  }
  else if (cmd == "RELAY_ALL_OFF") {
    allRelayOff();
    Serial1.println("OK:RELAY_ALL_OFF");
  }
}

// Handler untuk homing semua motor
void handleHomingCommand() {
  Serial.println("Memulai homing semua motor...");
  Serial1.println("INFO:HOMING_START");
  
  if (homingAllMotors()) {
    Serial.println("Homing selesai");
    Serial1.println("OK:HOMING_COMPLETE");
  } else {
    Serial.println("Homing gagal!");
    Serial1.println("ERROR:HOMING_FAILED");
  }
}

// Fungsi untuk mengirim status sistem
void sendStatus() {
  Serial1.print("STATUS:");
  Serial1.print("PUTAR=");
  Serial1.print(getPosisiPutar());
  Serial1.print(",NAIK_TURUN=");
  Serial1.print(getPosisiNaikTurun());
  Serial1.print(",MAJU_MUNDUR=");
  Serial1.print(getPosisiMajuMundur());
  Serial1.print(",SERVO=");
  Serial1.print(getCurrentServoAngle());
  Serial1.print(",RELAY1=");
  Serial1.print(getRelay1State() ? "ON" : "OFF");
  Serial1.print(",RELAY2=");
  Serial1.print(getRelay2State() ? "ON" : "OFF");
  Serial1.print(",HOMED=");
  Serial1.println(isAllMotorHomed() ? "YES" : "NO");
}

// Fungsi emergency stop
void emergencyStop() {
  Serial.println("EMERGENCY STOP!");
  
  stopMotorPutar();
  stopMotorNaikTurun();
  stopMotorMajuMundur();
  
  Serial1.println("OK:EMERGENCY_STOP");
}
