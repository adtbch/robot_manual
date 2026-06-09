#include "armbox_config.h"

extern bool encoderMonitor;

// ============================================================
// Serial Command Handler (USB Serial + UART)
// ============================================================

#define SERIAL_BUFFER_SIZE 64
static char serialBuffer[SERIAL_BUFFER_SIZE];
static uint8_t bufferIndex = 0;

static String uartInput = "";
static bool uartComplete = false;

// ============================================================
// Setup
// ============================================================
void setupSerialCommand() {
  Serial.println("Serial Command Handler Ready!");
  Serial.println("Commands:");
  Serial.println("  motorW <pos>    - Move Motor W (Putar) to position");
  Serial.println("  motorZ <pos>    - Move Motor Z (Naik Turun) to position");
  Serial.println("  motorY <pos>    - Move Motor Y (Maju Mundur) to position");
  Serial.println("  pwmw <value>    - Motor W direct PWM (-1023..1023)");
  Serial.println("  pwmz <value>    - Motor Z direct PWM (-1023..1023)");
  Serial.println("  pwmy <value>    - Motor Y direct PWM (-1023..1023)");
  Serial.println("  servo0 <angle>  - Set servo angle (0-180)");
  Serial.println("  relay0 <0/1>    - Set relay 1 (0=ON, 1=OFF)");
  Serial.println("  relay1 <0/1>    - Set relay 2 (0=ON, 1=OFF)");
  Serial.println("  homing          - Start homing all motors");
  Serial.println("  status          - Show motor positions");
  Serial.println("  encoders        - Show encoder counts");
  Serial.println("  setpid <p> <i> <d> - Set & Save PID constants for Motor W");
  Serial.println("  showpid         - Print current PID constants");
  Serial.println("  monitor         - Toggle continuous encoder print (every 500ms)");
  Serial.println("  stop            - Stop all motors");
  Serial.println();
}

#define master_serial Serial1

void initUART() {
  master_serial.begin(921600, SERIAL_8N1, master_serial_rxPin, master_serial_txPin);
  uartInput.reserve(200);
  Serial.println("  UART initialized (master_serial @ 921600)");
}

// ============================================================
// Parse and Execute Command
// ============================================================
void parseAndExecuteCommand(char* cmd) {
  char* token = strtok(cmd, " ");
  if (token == NULL) return;

  for (char* p = token; *p; ++p) *p = tolower(*p);

  // MOTOR COMMANDS (position target)
  if (strcmp(token, "motor0") == 0 || strcmp(token, "motor1") == 0 || strcmp(token, "motor2") == 0 ||
      strcmp(token, "motorw") == 0 || strcmp(token, "motorz") == 0 || strcmp(token, "motory") == 0) {
    uint8_t motorId = 0;
    if (token[5] == '1' || token[5] == 'z') motorId = MOTOR_Z;
    else if (token[5] == '2' || token[5] == 'y') motorId = MOTOR_Y;
    else motorId = MOTOR_W;

    char* valueStr = strtok(NULL, " ");
    if (valueStr != NULL) {
      long targetPos = atol(valueStr);
      setMotorTarget(motorId, targetPos);
      char axis = (motorId == MOTOR_W) ? 'W' : ((motorId == MOTOR_Z) ? 'Z' : 'Y');
      Serial.printf("Motor %c target set to: %ld\n", axis, targetPos);
    } else {
      Serial.println("Error: motor requires position value");
    }
  }

  // MOTOR PWM COMMANDS (direct PWM — untuk manual jog dari Master)
  else if (strcmp(token, "pwmw") == 0 || strcmp(token, "pwmz") == 0 || strcmp(token, "pwmy") == 0) {
    uint8_t motorId = 0;
    if (token[3] == 'z') motorId = MOTOR_Z;
    else if (token[3] == 'y') motorId = MOTOR_Y;
    else motorId = MOTOR_W;

    char* valueStr = strtok(NULL, " ");
    if (valueStr != NULL) {
      int pwm = atoi(valueStr);
      stopMotorTarget(motorId);  // stop position control dulu
      pwmMotor(motorId, pwm);
    } else {
      Serial.println("Error: pwm requires value (-1023..1023)");
    }
  }

  // SERVO COMMANDS
  else if (strcmp(token, "servo0") == 0) {
    char* valueStr = strtok(NULL, " ");
    if (valueStr != NULL) {
      int angle = atoi(valueStr);
      if (angle >= servoMinAngle && angle <= servoMaxAngle) {
        setServoAngle(0, angle);
        Serial.printf("Servo 0 set to: %d degrees\n", angle);
      } else {
        Serial.println("Error: Servo angle must be 0-180");
      }
    } else {
      Serial.println("Error: servo0 requires angle value");
    }
  }

  // RELAY COMMANDS
  else if (strcmp(token, "relay0") == 0 || strcmp(token, "relay1") == 0) {
    uint8_t relayId = token[5] - '0';
    char* valueStr = strtok(NULL, " ");
    if (valueStr != NULL) {
      int val = atoi(valueStr); // 0=ON, 1=OFF
      relay(relayId, val);
    } else {
      Serial.println("Error: relay requires 0/1 value (0=ON, 1=OFF)");
    }
  }

  // HOMING
  else if (strcmp(token, "homing") == 0) {
    Serial.println("Starting homing...");
    homingAllMotors();
  }

  // STATUS
  else if (strcmp(token, "status") == 0) {
    Serial.printf("Motor W: pos=%ld\n", encoderMotorW);
    Serial.printf("Motor Z: pos=%ld\n", encoderMotorZ);
    Serial.printf("Motor Y: pos=%ld\n", encoderMotorY);
    Serial.printf("Servo: %d degrees\n", getCurrentServoAngle());
    for (size_t i = 0; i < relays.size(); i++) {
      Serial.printf("Relay %ld: %s\n", i, relays[i].state ? "ON" : "OFF");
    }
  }

  // ENCODERS
  else if (strcmp(token, "encoders") == 0) {
    printAllEncoders();
  }

  // PID COMMANDS
  else if (strcmp(token, "setpid") == 0) {
    char* pStr = strtok(NULL, " ");
    char* iStr = strtok(NULL, " ");
    char* dStr = strtok(NULL, " ");
    if (pStr != NULL && iStr != NULL && dStr != NULL) {
      float kp = atof(pStr);
      float ki = atof(iStr);
      float kd = atof(dStr);
      setPidW(kp, ki, kd);
    } else {
      Serial.println("Usage: setpid <p> <i> <d> (e.g. setpid 2.5 0.05 0.1)");
    }
  }
  else if (strcmp(token, "showpid") == 0) {
    showPidW();
  }

  // ENCODER MONITOR (continuous periodic)
  else if (strcmp(token, "monitor") == 0) {
    encoderMonitor = !encoderMonitor;
    Serial.printf("Encoder continuous monitor: %s\n", encoderMonitor ? "ON (every 500ms)" : "OFF");
  }

  // STOP
  else if (strcmp(token, "stop") == 0) {
    motorStopAll();
    stopAllMotorTargets();
    Serial.println("All motors stopped");
  }

  // MOTOR STOP INDIVIDUAL
  else if (strcmp(token, "stop0") == 0 || strcmp(token, "stop1") == 0 || strcmp(token, "stop2") == 0 ||
           strcmp(token, "stopw") == 0 || strcmp(token, "stopz") == 0 || strcmp(token, "stopy") == 0) {
    uint8_t motorId = 0;
    if (token[4] == '1' || token[4] == 'z') motorId = MOTOR_Z;
    else if (token[4] == '2' || token[4] == 'y') motorId = MOTOR_Y;
    else motorId = MOTOR_W;

    stopMotorTarget(motorId);
    char axis = (motorId == MOTOR_W) ? 'W' : ((motorId == MOTOR_Z) ? 'Z' : 'Y');
    Serial.printf("Motor %c stopped\n", axis);
  }

  // MOTOR DIRECTION COMMANDS
  else if (strcmp(token, "putar_kanan") == 0 || strcmp(token, "w_kanan") == 0) { pwmMotor(MOTOR_W, PWM_MEDIUM); Serial.println("W kanan"); }
  else if (strcmp(token, "putar_kiri") == 0 || strcmp(token, "w_kiri") == 0) { pwmMotor(MOTOR_W, -PWM_MEDIUM); Serial.println("W kiri"); }
  else if (strcmp(token, "naik") == 0 || strcmp(token, "naik_turun_naik") == 0 || strcmp(token, "z_naik") == 0) { pwmMotor(MOTOR_Z, PWM_MEDIUM); Serial.println("Z naik"); }
  else if (strcmp(token, "turun") == 0 || strcmp(token, "naik_turun_turun") == 0 || strcmp(token, "z_turun") == 0) { pwmMotor(MOTOR_Z, -PWM_MEDIUM); Serial.println("Z turun"); }
  else if (strcmp(token, "maju") == 0 || strcmp(token, "maju_mundur_maju") == 0 || strcmp(token, "y_maju") == 0) { pwmMotor(MOTOR_Y, PWM_MEDIUM); Serial.println("Y maju"); }
  else if (strcmp(token, "mundur") == 0 || strcmp(token, "maju_mundur_mundur") == 0 || strcmp(token, "y_mundur") == 0) { pwmMotor(MOTOR_Y, -PWM_MEDIUM); Serial.println("Y mundur"); }

  // SERVO PRESETS
  else if (strcmp(token, "buka") == 0) { setServoAngle(0, servoMaxAngle); Serial.println("Gripper buka"); }
  else if (strcmp(token, "tutup") == 0) { setServoAngle(0, servoMinAngle); Serial.println("Gripper tutup"); }
  else if (strcmp(token, "home") == 0) { setServoAngle(0, servoHomeAngle); Serial.println("Servo home"); }

  // RELAY TOGGLE
  else if (strcmp(token, "relay0_on") == 0) { relay(0, 0); }
  else if (strcmp(token, "relay0_off") == 0) { relay(0, 1); }
  else if (strcmp(token, "relay1_on") == 0) { relay(1, 0); }
  else if (strcmp(token, "relay1_off") == 0) { relay(1, 1); }

  else {
    Serial.printf("Unknown command: %s\n", token);
  }
}

// ============================================================
// Read Serial Input (non-blocking)
// ============================================================
void serialCommandTick() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        serialBuffer[bufferIndex] = '\0';
        parseAndExecuteCommand(serialBuffer);
        bufferIndex = 0;
      }
    } else if (bufferIndex < SERIAL_BUFFER_SIZE - 1) {
      serialBuffer[bufferIndex++] = c;
    } else {
      Serial.println("Error: Command too long");
      bufferIndex = 0;
    }
  }
}

// ============================================================
// UART Handling (non-blocking)
// ============================================================
void readUART() {
  while (master_serial.available()) {
    char c = (char)master_serial.read();
    if (c == '\n') uartComplete = true;
    else if (c != '\r') uartInput += c;
  }
}

void processUARTCommand() {
  if (!uartComplete) return;
  uartInput.trim();
  String cmd = uartInput;
  uartInput = "";
  uartComplete = false;

  Serial.printf("[MASTER-RX] %s\n", cmd.c_str());

  // Convert to char array for strtok
  char cmdBuf[SERIAL_BUFFER_SIZE];
  cmd.toCharArray(cmdBuf, SERIAL_BUFFER_SIZE);

  parseAndExecuteCommand(cmdBuf);
}
