// ============================================================
// ARM MOTOR CONTROL - Sumbu X & Z Lengan 1
// ============================================================
// File: arm_motor.ino
// Fungsi: Kontrol motor PWM untuk sumbu X dan Z
// ============================================================

#include "arm_config.h"

// ============================================================
// GLOBAL VARIABLES - Definisi motor X dan Z
// ============================================================

ArmMotor motorX = {
  .pwmPinL = MOTOR_X_PWM_L,
  .pwmPinR = MOTOR_X_PWM_R,
  .ledcChannelL = MOTOR_X_LEDC_CH_L,
  .ledcChannelR = MOTOR_X_LEDC_CH_R,
  .encPinA = MOTOR_X_ENC_A,
  .encPinB = MOTOR_X_ENC_B,
  .limitPin = MOTOR_X_LIMIT,
  .state = MOTOR_IDLE,
  .encoder = {0, 0, 0, CENTER_POSITION_X, false}
};

ArmMotor motorZ = {
  .pwmPinL = MOTOR_Z_PWM_L,
  .pwmPinR = MOTOR_Z_PWM_R,
  .ledcChannelL = MOTOR_Z_LEDC_CH_L,
  .ledcChannelR = MOTOR_Z_LEDC_CH_R,
  .encPinA = MOTOR_Z_ENC_A,
  .encPinB = MOTOR_Z_ENC_B,
  .limitPin = MOTOR_Z_LIMIT,
  .state = MOTOR_IDLE,
  .encoder = {0, 0, 0, CENTER_POSITION_Z, false}
};

// ============================================================
// INISIALISASI MOTOR PWM
// ============================================================

bool armMotorInit() {
  Serial.println("=== ARM MOTOR INIT ===");
  
  // Setup PWM untuk Motor X
  ledcSetup(motorX.ledcChannelL, ARM_PWM_FREQUENCY, ARM_PWM_RESOLUTION);
  ledcSetup(motorX.ledcChannelR, ARM_PWM_FREQUENCY, ARM_PWM_RESOLUTION);
  ledcAttachPin(motorX.pwmPinL, motorX.ledcChannelL);
  ledcAttachPin(motorX.pwmPinR, motorX.ledcChannelR);
  
  // Setup PWM untuk Motor Z
  ledcSetup(motorZ.ledcChannelL, ARM_PWM_FREQUENCY, ARM_PWM_RESOLUTION);
  ledcSetup(motorZ.ledcChannelR, ARM_PWM_FREQUENCY, ARM_PWM_RESOLUTION);
  ledcAttachPin(motorZ.pwmPinL, motorZ.ledcChannelL);
  ledcAttachPin(motorZ.pwmPinR, motorZ.ledcChannelR);
  
  // Setup limit switch pins (INPUT_PULLUP)
  pinMode(motorX.limitPin, INPUT_PULLUP);
  pinMode(motorZ.limitPin, INPUT_PULLUP);
  
  // Stop semua motor
  armMotorStopAll();
  
  Serial.println("Motor X PWM: L=" + String(MOTOR_X_PWM_L) + " R=" + String(MOTOR_X_PWM_R));
  Serial.println("Motor Z PWM: L=" + String(MOTOR_Z_PWM_L) + " R=" + String(MOTOR_Z_PWM_R));
  Serial.println("Motor X Limit: " + String(MOTOR_X_LIMIT));
  Serial.println("Motor Z Limit: " + String(MOTOR_Z_LIMIT));
  Serial.println("======================");
  
  return true;
}

// ============================================================
// KONTROL MOTOR PWM
// ============================================================

// Set PWM motor (-1023 to 1023)
// Positif = maju/naik/kanan, Negatif = mundur/turun/kiri
void armMotorSetPWM(ArmMotor &motor, int pwmValue) {
  // Clamp PWM value
  if (pwmValue > ARM_PWM_MAX) pwmValue = ARM_PWM_MAX;
  if (pwmValue < -ARM_PWM_MAX) pwmValue = -ARM_PWM_MAX;
  
  // Safety: Stop jika limit switch ditekan dan bergerak ke arah limit
  if (armLimitSwitchPressed(motor) && pwmValue < 0) {
    armMotorStop(motor);
    Serial.println("SAFETY: Limit switch hit, stopping motor");
    return;
  }
  
  if (pwmValue > 0) {
    // Maju/naik/kanan (PWM Right)
    ledcWrite(motor.ledcChannelR, pwmValue);
    ledcWrite(motor.ledcChannelL, 0);
  } else if (pwmValue < 0) {
    // Mundur/turun/kiri (PWM Left)
    ledcWrite(motor.ledcChannelL, -pwmValue);
    ledcWrite(motor.ledcChannelR, 0);
  } else {
    // Stop
    ledcWrite(motor.ledcChannelL, 0);
    ledcWrite(motor.ledcChannelR, 0);
  }
}

// Stop motor
void armMotorStop(ArmMotor &motor) {
  ledcWrite(motor.ledcChannelL, 0);
  ledcWrite(motor.ledcChannelR, 0);
  motor.state = MOTOR_IDLE;
}

// Stop semua motor
void armMotorStopAll() {
  armMotorStop(motorX);
  armMotorStop(motorZ);
  Serial.println("All arm motors stopped");
}

// ============================================================
// LIMIT SWITCH
// ============================================================

// Cek apakah limit switch ditekan (LOW = pressed)
bool armLimitSwitchPressed(ArmMotor &motor) {
  static uint32_t lastDebounceTime[2] = {0, 0};
  static bool lastState[2] = {HIGH, HIGH};
  
  // Tentukan index (0=X, 1=Z)
  int index = (&motor == &motorX) ? 0 : 1;
  
  bool currentState = digitalRead(motor.limitPin);
  uint32_t now = millis();
  
  // Debounce
  if (currentState != lastState[index]) {
    lastDebounceTime[index] = now;
  }
  
  if ((now - lastDebounceTime[index]) > DEBOUNCE_DELAY_MS) {
    lastState[index] = currentState;
    return (currentState == LOW);  // LOW = pressed
  }
  
  return false;
}

// ============================================================
// ENCODER FUNCTIONS
// ============================================================

// Get encoder count
long armEncoderGetCount(ArmMotor &motor) {
  noInterrupts();
  long count = motor.encoder.count;
  interrupts();
  return count;
}

// Reset encoder count
void armEncoderReset(ArmMotor &motor) {
  noInterrupts();
  motor.encoder.count = 0;
  interrupts();
}

// ============================================================
// MOVEMENT CONTROL
// ============================================================

// Move to target position (non-blocking)
bool armMoveToPosition(ArmMotor &motor, long targetPosition) {
  if (motor.state == MOTOR_HOMING) {
    Serial.println("ERROR: Cannot move, homing in progress");
    return false;
  }
  
  if (!motor.encoder.isHomed) {
    Serial.println("ERROR: Motor not homed yet");
    return false;
  }
  
  motor.encoder.targetPosition = targetPosition;
  motor.state = MOTOR_MOVING;
  
  Serial.println("Moving to position: " + String(targetPosition));
  return true;
}

// Move to center position
bool armMoveToCenter() {
  bool xOk = armMoveToPosition(motorX, motorX.encoder.centerPosition);
  bool zOk = armMoveToPosition(motorZ, motorZ.encoder.centerPosition);
  
  if (xOk && zOk) {
    Serial.println("Moving to center position");
    return true;
  }
  
  return false;
}

// Movement tick (panggil di loop)
void armMovementTick() {
  // Process motor X
  if (motorX.state == MOTOR_MOVING) {
    long currentPos = armEncoderGetCount(motorX);
    long error = motorX.encoder.targetPosition - currentPos;
    
    if (abs(error) < 10) {
      // Reached target
      armMotorStop(motorX);
      Serial.println("Motor X reached target");
    } else {
      // Simple proportional control
      int pwm = constrain(error * 2, -ARM_PWM_MAX, ARM_PWM_MAX);
      armMotorSetPWM(motorX, pwm);
    }
  }
  
  // Process motor Z
  if (motorZ.state == MOTOR_MOVING) {
    long currentPos = armEncoderGetCount(motorZ);
    long error = motorZ.encoder.targetPosition - currentPos;
    
    if (abs(error) < 10) {
      // Reached target
      armMotorStop(motorZ);
      Serial.println("Motor Z reached target");
    } else {
      // Simple proportional control
      int pwm = constrain(error * 2, -ARM_PWM_MAX, ARM_PWM_MAX);
      armMotorSetPWM(motorZ, pwm);
    }
  }
}
