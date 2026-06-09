// Batas performa motor Anda (Bisa dimaksimalkan hingga maxrpm di robot_config)
const int kinematicMaxrpm = 500;

// 1. ROBOT-CENTRIC (Maju/Geser berdasarkan arah badan robot)
void driveRobotCentric(int vx, int vy, int vtheta) {
  // Rumus standar kinematik balik Mecanum Wheel
  int motor1 = vx + vy - vtheta; // Depan Kiri
  int motor2 = vx - vy + vtheta; // Depan Kanan
  int motor3 = vx - vy - vtheta; // Belakang Kiri
  int motor4 = vx + vy + vtheta; // Belakang Kanan

  skalaKecepatanPWM(motor1, motor2, motor3, motor4);
}

// 2. FIELD-CENTRIC (Maju/Geser berdasarkan arah lapangan, menggunakan data YAW)
void driveFieldCentric(int vx, int vy, int vtheta) {
  // Rotasi vx/vy dari field frame ke robot frame berdasarkan yaw
  // vx_r =  vx_f * cos(yaw) + vy_f * sin(yaw)
  // vy_r = -vx_f * sin(yaw) + vy_f * cos(yaw)
  float yawRad = getYaw() * (PI / 180.0f);
  float c = cosf(yawRad);
  float s = sinf(yawRad);

  int vxRot = roundf(vx * c - vy * s);
  int vyRot = roundf(vx * s + vy * c);

  // vtheta (rotasi) tetap robot-relative (standard field-centric convention)
  driveRobotCentric(vxRot, vyRot, vtheta);
}

void driveFieldCentricWithYawCorrection(int vx, int vy, int yawTarget) {
  float currentYaw = getYaw();
  // pidComputeYaw return int PWM — langsung sebagai vtheta
  int correctionYaw = pidComputeYaw(pidKinematicYaw, (float)yawTarget, currentYaw, 0.04f);
  driveFieldCentric(vx, vy, -correctionYaw);
}

// 3. FUNGSI SCALING (Memastikan rasio kecepatan tetap sama jika melebihi maxrpm)
void skalaKecepatanRPM(int motor1, int motor2, int motor3, int motor4) {
  // Cari nilai absolut tertinggi di antara keempat roda
  int maxInput = abs(motor1);
  if (abs(motor2) > maxInput) maxInput = abs(motor2);
  if (abs(motor3) > maxInput) maxInput = abs(motor3);
  if (abs(motor4) > maxInput) maxInput = abs(motor4);

  // Jika ada roda yang melebihi batas maxrpm, kecilkan semua roda secara proporsional
  if (maxInput > kinematicMaxrpm) {
    motor1 = (motor1 * kinematicMaxrpm) / maxInput;
    motor2 = (motor2 * kinematicMaxrpm) / maxInput;
    motor3 = (motor3 * kinematicMaxrpm) / maxInput;
    motor4 = (motor4 * kinematicMaxrpm) / maxInput;
  }

  // Kirim hasil akhir ke fungsi driver motor Anda
  // Pastikan urutan parameter sesuai dengan urutan fisik roda robot Anda
  rpmMotor(motor1, motor2, motor3, motor4); 
  // pwmMotor(0, motor1); // Motor 1 - Depan Kiri
  // pwmMotor(1, motor2); // Motor 2 - Depan Kanan
  // pwmMotor(2, motor3); // Motor 3 - Belakang Kiri
  // pwmMotor(3, motor4); // Motor 4 - Belakang Kanan
}

void skalaKecepatanPWM(int motor1, int motor2, int motor3, int motor4) {
  // Cari nilai absolut tertinggi di antara keempat roda
  int maxInput = abs(motor1);
  if (abs(motor2) > maxInput) maxInput = abs(motor2);
  if (abs(motor3) > maxInput) maxInput = abs(motor3);
  if (abs(motor4) > maxInput) maxInput = abs(motor4);

  // Jika ada roda yang melebihi batas maxPwm, kecilkan semua roda secara proporsional
  if (maxInput > maxPwm) {
    motor1 = (motor1 * maxPwm) / maxInput;
    motor2 = (motor2 * maxPwm) / maxInput;
    motor3 = (motor3 * maxPwm) / maxInput;
    motor4 = (motor4 * maxPwm) / maxInput;
  }

  pwmMotor(0, motor1); // Motor 1 - Depan Kiri
  pwmMotor(1, motor2); // Motor 2 - Depan Kanan
  pwmMotor(2, motor3); // Motor 3 - Belakang Kiri
  pwmMotor(3, motor4); // Motor 4 - Belakang Kanan
}