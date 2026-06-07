// ============================================================
// DOKUMENTASI PROJECT & STRUKTUR FILE
// ============================================================
// File ini menjelaskan tujuan dan fungsi utama dari setiap file yang
// ada di dalam proyek ESP32-S3 Slave 1 (Motion & Lokalisasi).

/*
1. pid.cpp & pid.h
   - Berisi fungsi utama perhitungan PID (computePID) yang didesain secara modular, non-blocking,
     dan menggunakan dynamic C++ std::vector untuk menyimpan state error, integral, derivative,
     dan lastTime per indeks secara dinamis tanpa perlu deklarasi ukuran array statis.
   - Menyediakan fitur control best practices seperti Anti-Windup (Back-Calculation), Setpoint
     Weighting (2-DOF), Derivative on Measurement, dan Derivative Filtering (Low-pass).
   - Menyediakan fungsi legacy (pid_init, pid_compute, pid_reset) untuk backward compatibility.

2. pid_controller.ino
   - Berisi adapter pidCompute() yang bertugas menjembatani fungsi global computePID() dengan
     kontrol putaran motor roda robot.
   - Mengatur proses inisialisasi PID gains, reset satu/seluruh PID motor, penyimpanan parameter
     ke Non-Volatile Storage (NVS) Preferences, serta interface kontrol kecepatan linear roda
     rpmMotorControl() dan rpmMotorControlTargets().

3. motor.ino
   - Mengatur konfigurasi pin PWM & arah motor fisik BTS7960/BTS7970 untuk 4 roda Mecanum.
   - Menyediakan fungsi SetupMotors() untuk setup LEDC PWM dan arah putaran motor.
   - Menyediakan fungsi pwmMotor(idMotor, pwmValue) untuk mengontrol motor secara open-loop.
   - Menyediakan fungsi utama rpmMotor(rpm1, rpm2, rpm3, rpm4) yang mengontrol kecepatan target
     individual keempat roda secara closed-loop menggunakan computePID() dengan memuat parameter
     hasil tuning otomatis dari NVS secara dinamis per roda.

4. encoder.ino
   - Berisi fungsi pembacaan data encoder internal quadrature dari ke-4 motor DC Mecanum
     secara non-blocking berbasis Interupsi Perangkat Keras (attachInterruptArg).
   - Menyediakan filter Low-Pass IIR digital dalam convertEncoderToRPM() untuk menghitung
     kecepatan aktual RPM secara halus dan bebas noise frekuensi tinggi.
   - Menyediakan interface getter kecepatan aktual seperti getEncoderVelocityRpm() dan
     getEncoderVelocityRadS() untuk digunakan oleh PID controller dan Auto-Tuner.

5. kinematik.ino
   - Mengimplementasikan rumus Forward dan Inverse Kinematics untuk sasis roda Mecanum 4.
   - Menyediakan driveRobotCentric(vx, vy, vtheta) untuk pergerakan relatif terhadap arah sasis.
   - Menyediakan driveFieldCentric(vx, vy, vtheta) untuk pergerakan relatif terhadap koordinat
     lapangan global menggunakan data orientasi yaw dari sensor lokalisasi/IMU.
   - Menyediakan skalaKecepatan() untuk membatasi input RPM proporsional agar tidak melebihi
     batas maksimum kemampuan motor fisik (maxrpm = 150).

6. serial_commands.ino & serial_commands.h
   - Implementasi sistem antarmuka baris perintah (Serial CLI) non-blocking via Serial USB (115200 baud).
   - Mengizinkan operator melakukan pengujian motor satu per satu (independen) atau bersamaan,
     menjalankan sekuensial tes, men-trigger auto-tuner per motor maupun seluruhnya, serta
     melakukan pengujian gerakan kinematik ROBOT (Robot-Centric) dan FIELD (Field-Centric).
   - Menyediakan proteksi darurat lewat perintah STOP (Emergency Stop).

7. autoTuner.ino
   - Berisi implementasi sistem State-Machine Auto-Tuning PID RPM otomatis secara non-blocking.
   - Menggunakan Welford's Online Algorithm untuk kestabilan steady-state, kalkulasi rise-time,
     serta scoring system berbasis penalti untuk mencari parameter optimal (Kp, Ki, Kd) secara
     mandiri per motor, lalu menyimpannya langsung ke NVS.

8. esp32s3_slave1_locaizer_motion.ino
   - File utama sketch Arduino (main sketch) yang menyatukan seluruh program.
   - Berisi setup() untuk inisialisasi Serial, driver motor, PID controller, encoder interupsi,
     serta pembacaan komunikasi radio ESP-NOW.
   - Berisi loop() yang berjalan kontinu (Super-Loop) untuk memanggil convertEncoderToRPM(),
     espNowControlTick(), dan processSerialCommands() setiap saat secara non-blocking.

9. robot_config.h
   - Header utama konfigurasi global yang berisi definisi pin-pin motor & encoder, parameter
     fisik robot (radius roda, TPR/PPR encoder), batas PWM, frekuensi LEDC, nama namespace NVS,
     dan deklarasi fungsi antar-file (.ino) agar dapat saling memanggil secara modular.

10. error.ino
    - Berisi sistem penanganan galat (error handling) produksi, logging non-fatal melalui
      fungsi logError(), dan sistem peringatan jika terjadi error komputasi matematika (NaN/Inf).
*/
