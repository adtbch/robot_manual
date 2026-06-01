// Batas performa motor Anda (Bisa dimaksimalkan hingga maxrpm di robot_config)
const int kinematicMaxrpm = 500;

// 1. ROBOT-CENTRIC (Maju/Geser berdasarkan arah badan robot)
void driveRobotCentric(int vx, int vy, int vtheta) {
  // Rumus standar kinematik balik Mecanum Wheel
  int rpm1 = vx + vy - vtheta; // Depan Kiri
  int rpm2 = vx + vy + vtheta; // Depan Kanan
  int rpm3 = vx + vy + vtheta; // Belakang Kiri
  int rpm4 = vx + vy - vtheta; // Belakang Kanan

  skalaKecepatan(rpm1, rpm2, rpm3, rpm4);
}

// 2. FIELD-CENTRIC (Maju/Geser berdasarkan arah lapangan, menggunakan data YAW)
void driveFieldCentric(int vx, int vy, int vtheta) {
  driveRobotCentric(vx, vy, vtheta);
}

// 3. FUNGSI SCALING (Memastikan rasio kecepatan tetap sama jika melebihi maxrpm)
void skalaKecepatan(int motor1, int motor2, int motor3, int motor4) {
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
