#include "robot_config.h"

// ============================================================
// Serial Command Handler untuk Motor & Servo Control
// ============================================================
// Format command:
// - Motor: "motorX 1000" atau "motorZ 500" (target encoder position)
// - Servo: "servo1 90" atau "servo2 45" (angle 0-180)
// - Status: "status" (menampilkan posisi encoder dan status motor)
// - Stop: "stop" (stop semua motor)
// ============================================================

// Buffer untuk command serial
#define SERIAL_BUFFER_SIZE 64
static char serialBuffer[SERIAL_BUFFER_SIZE];
static uint8_t bufferIndex = 0;

// ============================================================
// Setup Serial Handler
// ============================================================
void setupSerialCommand() {
  Serial.println("Serial Command Handler Ready!");
  Serial.println("Commands:");
  Serial.println("  motorX <position>  - Move motor X to encoder position");
  Serial.println("  motorZ <position>  - Move motor Z to encoder position");
  Serial.println("  servo1 <angle>     - Set servo 1 angle (0-180)");
  Serial.println("  servo2 <angle>     - Set servo 2 angle (0-180)");
  Serial.println("  status             - Show motor positions");
  Serial.println("  stop               - Stop all motors");
  Serial.println();
}

// ============================================================
// Parse dan Execute Command
// ============================================================
void parseAndExecuteCommand(char* cmd) {
  // Tokenize command
  char* token = strtok(cmd, " ");
  if (token == NULL) return;

  // Convert to lowercase untuk case-insensitive
  for (char* p = token; *p; ++p) *p = tolower(*p);

  // ========== MOTOR X COMMAND ==========
  if (strcmp(token, "motorx") == 0) {
    char* valueStr = strtok(NULL, " ");
    if (valueStr != NULL) {
      long targetPos = atol(valueStr);
      setMotorTarget(0, targetPos); // Set target modular
      Serial.printf("Motor X target set to: %ld\n", targetPos);
    } else {
      Serial.println("Error: motorX requires position value");
    }
  }
  
  // ========== MOTOR Z COMMAND ==========
  else if (strcmp(token, "motorz") == 0) {
    char* valueStr = strtok(NULL, " ");
    if (valueStr != NULL) {
      long targetPos = atol(valueStr);
      setMotorTarget(1, targetPos); // Set target modular
      Serial.printf("Motor Z target set to: %ld\n", targetPos);
    } else {
      Serial.println("Error: motorZ requires position value");
    }
  }
  
  // ========== SERVO 1 COMMAND ==========
  else if (strcmp(token, "servo1") == 0) {
    char* valueStr = strtok(NULL, " ");
    if (valueStr != NULL) {
      int angle = atoi(valueStr);
      if (angle >= 0 && angle <= 180) {
        setServoAngle(0, angle);
        Serial.printf("Servo 1 set to: %d degrees\n", angle);
      } else {
        Serial.println("Error: Servo angle must be 0-180");
      }
    } else {
      Serial.println("Error: servo1 requires angle value");
    }
  }
  
  // ========== SERVO 2 COMMAND ==========
  else if (strcmp(token, "servo2") == 0) {
    char* valueStr = strtok(NULL, " ");
    if (valueStr != NULL) {
      int angle = atoi(valueStr);
      if (angle >= 0 && angle <= 180) {
        setServoAngle(1, angle);
        Serial.printf("Servo 2 set to: %d degrees\n", angle);
      } else {
        Serial.println("Error: Servo angle must be 0-180");
      }
    } else {
      Serial.println("Error: servo2 requires angle value");
    }
  }
    
  // ========== STOP COMMAND ==========
  else if (strcmp(token, "stop") == 0) {
    motorStopAll();
    stopAllMotorTargets();
    Serial.println("All motors stopped");
  }
  
  // ========== UNKNOWN COMMAND ==========
  else {
    Serial.printf("Unknown command: %s\n", token);
  }
}

// ============================================================
// Read Serial Input (non-blocking)
// ============================================================
void serialCommandTick() {
  // Read serial input
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Newline = execute command
    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        serialBuffer[bufferIndex] = '\0';  // Null terminate
        parseAndExecuteCommand(serialBuffer);
        bufferIndex = 0;  // Reset buffer
      }
    }
    // Add character to buffer
    else if (bufferIndex < SERIAL_BUFFER_SIZE - 1) {
      serialBuffer[bufferIndex++] = c;
    }
    // Buffer overflow protection
    else {
      Serial.println("Error: Command too long");
      bufferIndex = 0;
    }
  }
  
  // Terus jalankan update target motor di background setiap loop cycle
  updateMotorPositioning();
}
