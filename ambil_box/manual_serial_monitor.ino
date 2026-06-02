/*
 * File: manual_serial_monitor.ino
 * Deskripsi: Kontrol manual melalui Serial Monitor (USB)
 * Tanggal: 2 Juni 2026
 */

#include "armbox_config.h"

String serialMonInput = "";
bool serialMonComplete = false;
int manualDefaultSpeed = 450;

enum ManualAction {
  ACTION_NONE,
  ACTION_PUTAR_KANAN,
  ACTION_PUTAR_KIRI,
  ACTION_PUTAR_POS,
  ACTION_NAIK,
  ACTION_TURUN,
  ACTION_NAIK_TURUN_POS,
  ACTION_MAJU,
  ACTION_MUNDUR,
  ACTION_MAJU_MUNDUR_POS
};

ManualAction pendingAction = ACTION_NONE;
long pendingValue = 0;

void printManualHelp();
void handleManualPutarCommand(const String &cmd);
void handleManualNaikTurunCommand(const String &cmd);
void handleManualMajuMundurCommand(const String &cmd);
void handleManualServoCommand(const String &cmd);
void handleManualRelayCommand(const String &cmd);
void sendStatusToSerial();
void emergencyStopSerial();
void setPendingAction(ManualAction action, long value);
const char *getPendingActionName();
void startPendingAction();

void initSerialMonitorControl() {
  serialMonInput.reserve(200);

  Serial.println("\n=== Manual Control via Serial Monitor ===");
  Serial.println("Set line ending to Newline.");
  Serial.println("Pilih gerakan, lalu kirim G. STOP untuk berhenti.");
  Serial.println("Ketik HELP untuk daftar perintah.");
}

void readSerialMonitor() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      serialMonComplete = true;
    } else if (inChar != '\r') {
      serialMonInput += inChar;
    }
  }
}

void processSerialMonitorCommand() {
  if (!serialMonComplete) {
    return;
  }

  serialMonInput.trim();
  if (serialMonInput.length() == 0) {
    serialMonInput = "";
    serialMonComplete = false;
    return;
  }

  String cmd = serialMonInput;
  cmd.toUpperCase();

  if (cmd == "HELP" || cmd == "?") {
    printManualHelp();
  } else if (cmd == "G" || cmd == "START") {
    startPendingAction();
  } else if (cmd.startsWith("SPEED_")) {
    int speed = cmd.substring(6).toInt();
    manualDefaultSpeed = constrain(speed, 0, PWM_MAX);
    Serial.print("OK:SPEED=");
    Serial.println(manualDefaultSpeed);
  } else if (cmd.startsWith("PUTAR_")) {
    handleManualPutarCommand(cmd);
  } else if (cmd.startsWith("NAIK_TURUN_")) {
    handleManualNaikTurunCommand(cmd);
  } else if (cmd.startsWith("MAJU_MUNDUR_")) {
    handleManualMajuMundurCommand(cmd);
  } else if (cmd.startsWith("SERVO_")) {
    handleManualServoCommand(cmd);
  } else if (cmd.startsWith("RELAY_")) {
    handleManualRelayCommand(cmd);
  } else if (cmd == "HOMING" || cmd == "HOMING_ALL") {
    Serial.println("INFO:HOMING_START");
    if (homingAllMotors()) {
      Serial.println("OK:HOMING_ALL_COMPLETE");
    } else {
      Serial.println("ERROR:HOMING_FAILED");
    }
  } else if (cmd == "HOMING_STATUS") {
    printHomingStatus();
  } else if (cmd == "ENCODER_STATUS") {
    printAllEncoders();
  } else if (cmd == "STATUS") {
    sendStatusToSerial();
  } else if (cmd == "STOP") {
    emergencyStopSerial();
  } else {
    Serial.println("ERROR:UNKNOWN_COMMAND");
    Serial.println("Ketik HELP untuk daftar perintah.");
  }

  if (cmd != "HELP" && cmd != "?") {
    printAllEncoders();
  }

  serialMonInput = "";
  serialMonComplete = false;
}

void handleManualPutarCommand(const String &cmd) {
  if (cmd == "PUTAR_KANAN") {
    setPendingAction(ACTION_PUTAR_KANAN, 0);
  } else if (cmd == "PUTAR_KIRI") {
    setPendingAction(ACTION_PUTAR_KIRI, 0);
  } else if (cmd == "PUTAR_STOP") {
    stopMotorPutar();
    Serial.println("OK:PUTAR_STOP");
  } else if (cmd.startsWith("PUTAR_POS_")) {
    long pos = cmd.substring(10).toInt();
    setPendingAction(ACTION_PUTAR_POS, pos);
  } else if (cmd == "PUTAR_HOME") {
    if (homingMotorPutar()) {
      Serial.println("OK:PUTAR_HOME");
    } else {
      Serial.println("ERROR:PUTAR_HOME");
    }
  } else {
    Serial.println("ERROR:PUTAR_COMMAND");
  }
}

void handleManualNaikTurunCommand(const String &cmd) {
  if (cmd == "NAIK_TURUN_NAIK") {
    setPendingAction(ACTION_NAIK, 0);
  } else if (cmd == "NAIK_TURUN_TURUN") {
    setPendingAction(ACTION_TURUN, 0);
  } else if (cmd == "NAIK_TURUN_STOP") {
    stopMotorNaikTurun();
    Serial.println("OK:NAIK_TURUN_STOP");
  } else if (cmd.startsWith("NAIK_TURUN_POS_")) {
    long pos = cmd.substring(15).toInt();
    setPendingAction(ACTION_NAIK_TURUN_POS, pos);
  } else if (cmd == "NAIK_TURUN_HOME") {
    if (homingMotorNaikTurun()) {
      Serial.println("OK:NAIK_TURUN_HOME");
    } else {
      Serial.println("ERROR:NAIK_TURUN_HOME");
    }
  } else {
    Serial.println("ERROR:NAIK_TURUN_COMMAND");
  }
}

void handleManualMajuMundurCommand(const String &cmd) {
  if (cmd == "MAJU_MUNDUR_MAJU") {
    setPendingAction(ACTION_MAJU, 0);
  } else if (cmd == "MAJU_MUNDUR_MUNDUR") {
    setPendingAction(ACTION_MUNDUR, 0);
  } else if (cmd == "MAJU_MUNDUR_STOP") {
    stopMotorMajuMundur();
    Serial.println("OK:MAJU_MUNDUR_STOP");
  } else if (cmd.startsWith("MAJU_MUNDUR_POS_")) {
    long pos = cmd.substring(16).toInt();
    setPendingAction(ACTION_MAJU_MUNDUR_POS, pos);
  } else if (cmd == "MAJU_MUNDUR_HOME") {
    if (homingMotorMajuMundur()) {
      Serial.println("OK:MAJU_MUNDUR_HOME");
    } else {
      Serial.println("ERROR:MAJU_MUNDUR_HOME");
    }
  } else {
    Serial.println("ERROR:MAJU_MUNDUR_COMMAND");
  }
}

void handleManualServoCommand(const String &cmd) {
  if (cmd == "SERVO_BUKA") {
    bukaGripper();
    Serial.println("OK:SERVO_BUKA");
  } else if (cmd == "SERVO_TUTUP") {
    tutupGripper();
    Serial.println("OK:SERVO_TUTUP");
  } else if (cmd == "SERVO_HOME") {
    servoHome();
    Serial.println("OK:SERVO_HOME");
  } else if (cmd == "SERVO_SETENGAH") {
    gripperSetengah();
    Serial.println("OK:SERVO_SETENGAH");
  } else if (cmd.startsWith("SERVO_ANGLE_")) {
    int angle = cmd.substring(12).toInt();
    setServoAngle(angle);
    Serial.println("OK:SERVO_ANGLE");
  } else {
    Serial.println("ERROR:SERVO_COMMAND");
  }
}

void handleManualRelayCommand(const String &cmd) {
  if (cmd == "RELAY_1_ON") {
    relay1On();
    Serial.println("OK:RELAY_1_ON");
  } else if (cmd == "RELAY_1_OFF") {
    relay1Off();
    Serial.println("OK:RELAY_1_OFF");
  } else if (cmd == "RELAY_2_ON") {
    relay2On();
    Serial.println("OK:RELAY_2_ON");
  } else if (cmd == "RELAY_2_OFF") {
    relay2Off();
    Serial.println("OK:RELAY_2_OFF");
  } else if (cmd == "RELAY_ALL_ON") {
    allRelayOn();
    Serial.println("OK:RELAY_ALL_ON");
  } else if (cmd == "RELAY_ALL_OFF") {
    allRelayOff();
    Serial.println("OK:RELAY_ALL_OFF");
  } else {
    Serial.println("ERROR:RELAY_COMMAND");
  }
}

void sendStatusToSerial() {
  Serial.print("STATUS:");
  Serial.print("PUTAR=");
  Serial.print(getPosisiPutar());
  Serial.print(",NAIK_TURUN=");
  Serial.print(getPosisiNaikTurun());
  Serial.print(",MAJU_MUNDUR=");
  Serial.print(getPosisiMajuMundur());
  Serial.print(",SERVO=");
  Serial.print(getCurrentServoAngle());
  Serial.print(",RELAY1=");
  Serial.print(getRelay1State() ? "ON" : "OFF");
  Serial.print(",RELAY2=");
  Serial.print(getRelay2State() ? "ON" : "OFF");
  Serial.print(",HOMED=");
  Serial.println(isAllMotorHomed() ? "YES" : "NO");
}

void emergencyStopSerial() {
  stopMotorPutar();
  stopMotorNaikTurun();
  stopMotorMajuMundur();
  Serial.println("OK:EMERGENCY_STOP");
}

void setPendingAction(ManualAction action, long value) {
  pendingAction = action;
  pendingValue = value;
  Serial.print("READY:");
  Serial.println(getPendingActionName());
}

const char *getPendingActionName() {
  switch (pendingAction) {
    case ACTION_PUTAR_KANAN:
      return "PUTAR_KANAN";
    case ACTION_PUTAR_KIRI:
      return "PUTAR_KIRI";
    case ACTION_PUTAR_POS:
      return "PUTAR_POS";
    case ACTION_NAIK:
      return "NAIK_TURUN_NAIK";
    case ACTION_TURUN:
      return "NAIK_TURUN_TURUN";
    case ACTION_NAIK_TURUN_POS:
      return "NAIK_TURUN_POS";
    case ACTION_MAJU:
      return "MAJU_MUNDUR_MAJU";
    case ACTION_MUNDUR:
      return "MAJU_MUNDUR_MUNDUR";
    case ACTION_MAJU_MUNDUR_POS:
      return "MAJU_MUNDUR_POS";
    default:
      return "NONE";
  }
}

void startPendingAction() {
  if (pendingAction == ACTION_NONE) {
    Serial.println("ERROR:NO_ACTION_SELECTED");
    return;
  }

  switch (pendingAction) {
    case ACTION_PUTAR_KANAN:
      putarKanan(manualDefaultSpeed);
      Serial.println("OK:START_PUTAR_KANAN");
      break;
    case ACTION_PUTAR_KIRI:
      putarKiri(manualDefaultSpeed);
      Serial.println("OK:START_PUTAR_KIRI");
      break;
    case ACTION_PUTAR_POS:
      putarKePosisi(pendingValue, manualDefaultSpeed);
      Serial.println("OK:PUTAR_POS");
      pendingAction = ACTION_NONE;
      break;
    case ACTION_NAIK:
      motorNaik(manualDefaultSpeed);
      Serial.println("OK:START_NAIK");
      break;
    case ACTION_TURUN:
      motorTurun(manualDefaultSpeed);
      Serial.println("OK:START_TURUN");
      break;
    case ACTION_NAIK_TURUN_POS:
      naikTurunKePosisi(pendingValue, manualDefaultSpeed);
      Serial.println("OK:NAIK_TURUN_POS");
      pendingAction = ACTION_NONE;
      break;
    case ACTION_MAJU:
      motorMaju(manualDefaultSpeed);
      Serial.println("OK:START_MAJU");
      break;
    case ACTION_MUNDUR:
      motorMundur(manualDefaultSpeed);
      Serial.println("OK:START_MUNDUR");
      break;
    case ACTION_MAJU_MUNDUR_POS:
      majuMundurKePosisi(pendingValue, manualDefaultSpeed);
      Serial.println("OK:MAJU_MUNDUR_POS");
      pendingAction = ACTION_NONE;
      break;
    default:
      Serial.println("ERROR:INVALID_ACTION");
      break;
  }
}

void printManualHelp() {
  Serial.println("\n--- COMMAND LIST ---");
  Serial.println("Pilih gerakan, lalu kirim G. STOP untuk berhenti.");
  Serial.println("PUTAR_KANAN | PUTAR_KIRI | PUTAR_POS_<nilai> | PUTAR_HOME");
  Serial.println("NAIK_TURUN_NAIK | NAIK_TURUN_TURUN | NAIK_TURUN_POS_<nilai> | NAIK_TURUN_HOME");
  Serial.println("MAJU_MUNDUR_MAJU | MAJU_MUNDUR_MUNDUR | MAJU_MUNDUR_POS_<nilai> | MAJU_MUNDUR_HOME");
  Serial.println("G | STOP");
  Serial.println("SERVO_BUKA | SERVO_TUTUP | SERVO_HOME | SERVO_SETENGAH");
  Serial.println("SERVO_ANGLE_<nilai>");
  Serial.println("RELAY_1_ON | RELAY_1_OFF | RELAY_2_ON | RELAY_2_OFF");
  Serial.println("RELAY_ALL_ON | RELAY_ALL_OFF");
  Serial.println("HOMING | HOMING_STATUS | ENCODER_STATUS | STATUS | STOP");
  Serial.println("SPEED_<0-255> | HELP");
  Serial.println("---------------------\n");
}
