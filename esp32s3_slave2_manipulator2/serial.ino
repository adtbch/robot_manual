#include "armbox_config.h"

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
  Serial.println("  motorW <pos>  - Move Motor W (Putar) to position");
  Serial.println("  motorZ <pos>  - Move Motor Z (Naik Turun) to position");
  Serial.println("  motorY <pos>  - Move Motor Y (Maju Mundur) to position");
  Serial.println("  servo0 <angle> - Set servo angle (0-180)");
  Serial.println("  relay0 <0/1>  - Set relay 1 on/off");
  Serial.println("  relay1 <0/1>  - Set relay 2 on/off");
  Serial.println("  homing        - Start homing all motors");
  Serial.println("  status        - Show motor positions");
  Serial.println("  encoders      - Show encoder counts");
  Serial.println("  stop          - Stop all motors");
  Serial.println();
}

void initUART() {
  Serial1.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  uartInput.reserve(200);
  Serial.println("  UART initialized");
}

// ============================================================
// Parse and Execute Command
// ============================================================
void parseAndExecuteCommand(char* cmd) {
  char* token = strtok(cmd, " ");
  if (token == NULL) return;

  for (char* p = token; *p; ++p) *p = tolower(*p);

  // MOTOR COMMANDS
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
      bool state = atoi(valueStr);
      setRelay(relayId, state);
      Serial.printf("Relay %d set to: %s\n", relayId, state ? "ON" : "OFF");
    } else {
      Serial.println("Error: relay requires 0/1 value");
    }
  }

  // HOMING
  else if (strcmp(token, "homing") == 0) {
    Serial.println("Starting homing...");
    homingAllMotors();
  }

  // STATUS
  else if (strcmp(token, "status") == 0) {
    for (size_t i = 0; i < motors.size(); i++) {
      char axis = (i == MOTOR_W) ? 'W' : ((i == MOTOR_Z) ? 'Z' : 'Y');
      Serial.printf("Motor %ld (Sumbu %c): pos=%ld\n", i, axis, getEncoderCount(i));
    }
    Serial.printf("Servo: %d degrees\n", getCurrentServoAngle());
    for (size_t i = 0; i < relays.size(); i++) {
      Serial.printf("Relay %ld: %s\n", i, relays[i].state ? "ON" : "OFF");
    }
  }

  // ENCODERS
  else if (strcmp(token, "encoders") == 0) {
    printAllEncoders();
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
  else if (strcmp(token, "relay0_on") == 0) { setRelay(0, true); Serial.println("Relay 1 ON"); }
  else if (strcmp(token, "relay0_off") == 0) { setRelay(0, false); Serial.println("Relay 1 OFF"); }
  else if (strcmp(token, "relay1_on") == 0) { setRelay(1, true); Serial.println("Relay 2 ON"); }
  else if (strcmp(token, "relay1_off") == 0) { setRelay(1, false); Serial.println("Relay 2 OFF"); }

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

  updateMotorPositioning();
}

// ============================================================
// UART Handling (non-blocking)
// ============================================================
void readUART() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
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

  // Convert to char array for strtok
  char cmdBuf[SERIAL_BUFFER_SIZE];
  cmd.toCharArray(cmdBuf, SERIAL_BUFFER_SIZE);

  Serial.printf("UART CMD: %s\n", cmdBuf);
  parseAndExecuteCommand(cmdBuf);
}
