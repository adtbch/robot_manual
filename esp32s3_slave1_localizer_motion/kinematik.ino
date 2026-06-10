// 1. ROBOT-CENTRIC (Maju/Geser berdasarkan arah badan robot)
void driveRobotCentric(int vx, int vy, int vtheta) {
  // Inverse kinematics Mecanum — output sebagai PWM
  int motor1 = vx + vy - vtheta;
  int motor2 = vx - vy + vtheta;
  int motor3 = vx - vy - vtheta;
  int motor4 = vx + vy + vtheta;

  skalaKecepatanPWM(motor1, motor2, motor3, motor4);
}

// 2. FIELD-CENTRIC (Maju/Geser berdasarkan arah lapangan, menggunakan data YAW)
void driveFieldCentric(int vx, int vy, int vtheta) {
  // Rotasi vx/vy dari field frame ke robot frame berdasarkan yaw
  // vx_r = vx_f * cos(yaw) - vy_f * sin(yaw)
  // vy_r = vx_f * sin(yaw) + vy_f * cos(yaw)
  float yawRad = getYaw() * (PI / 180.0f);
  float c = cosf(yawRad);
  float s = sinf(yawRad);

  int vxRot = roundf(vx * c - vy * s);
  int vyRot = roundf(vx * s + vy * c);

  driveRobotCentric(vxRot, vyRot, vtheta);
}

void driveFieldCentricWithYawCorrection(int vx, int vy, int yawTarget) {
  static unsigned long lastCallTime_ms = 0;
  unsigned long now_ms = millis();
  float actualDt_sec = (lastCallTime_ms == 0) ? 0.04f : (now_ms - lastCallTime_ms) * 0.001f;
  lastCallTime_ms = now_ms;

  float currentYaw = getYaw();
  int correctionYaw = pidComputeYaw(pidKinematicYaw, (float)yawTarget, currentYaw, actualDt_sec);
  driveFieldCentric(vx, vy, -correctionYaw);
}

void driveRobotCentricRpm(int vx, int vy, int vtheta) {
  int motorFR = vx + vy - vtheta;
  int motorFL = vx - vy + vtheta;
  int motorBR = vx - vy - vtheta;
  int motorBL = vx + vy + vtheta;

  int maxVal = max(max(abs(motorFR), abs(motorFL)), max(abs(motorBR), abs(motorBL)));
  if (maxVal > maxrpm) {
    motorFR = motorFR * maxrpm / maxVal;
    motorFL = motorFL * maxrpm / maxVal;
    motorBR = motorBR * maxrpm / maxVal;
    motorBL = motorBL * maxrpm / maxVal;
  }

  rpmMotor(motorFR, motorFL, motorBR, motorBL);
}

// 3. FUNGSI SCALING (PWM — dipakai oleh driveRobotCentric)
void skalaKecepatanPWM(int motor1, int motor2, int motor3, int motor4) {
  int maxInput = abs(motor1);
  if (abs(motor2) > maxInput) maxInput = abs(motor2);
  if (abs(motor3) > maxInput) maxInput = abs(motor3);
  if (abs(motor4) > maxInput) maxInput = abs(motor4);

  if (maxInput > maxPwm) {
    motor1 = (motor1 * maxPwm) / maxInput;
    motor2 = (motor2 * maxPwm) / maxInput;
    motor3 = (motor3 * maxPwm) / maxInput;
    motor4 = (motor4 * maxPwm) / maxInput;
  }

  pwmMotor(0, motor1);
  pwmMotor(1, motor2);
  pwmMotor(2, motor3);
  pwmMotor(3, motor4);
}
