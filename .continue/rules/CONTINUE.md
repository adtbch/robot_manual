# KRAI 2026 Robot Manual — Project Guide

Selamat datang di Panduan Proyek Firmware Robot Manual KRAI 2026! Panduan ini dirancang untuk membantu tim pengembang memahami arsitektur, standar penulisan kode, alur kerja (workflow), serta tugas-tugas umum dalam pengembangan dan pemeliharaan firmware robot.

---

## 1. Project Overview (Ikhtisar Proyek)

Proyek ini berisi firmware multi-board berbasis **ESP32** dan **ESP32-S3** untuk mengontrol Robot Manual KRAI 2026. Robot dikendalikan secara nirkabel menggunakan controller PS4 (DualShock 4) dengan dua jalur komunikasi redundan (WSN-31 Radio UART & WiFi ESP-NOW).

### Teknologi Utama
* **Mikrokontroler:** ESP32 (Controller) & ESP32-S3 (Sasis, Master, dan Slave Manipulator).
* **Framework:** Arduino Framework (API murni, non-blocking, tanpa FreeRTOS).
* **Komunikasi:** ESP-NOW (WiFi langsung) & WSN-31 (Radio Transceiver UART biner).
* **Kinematika:** Sasis Roda Mecanum 4 dengan Navigasi Field-Centric.
* **Sensor/Lokalisasi:** IMU MPU6050/MPU9250 (yaw drift correction) & 4x Encoder Roda (odometri).

### Arsitektur Sistem & Aliran Data
```
  [ PS4 DualShock 4 Controller ]
                | Bluetooth Classic
                v
       [ ESP32 Controller ]
                | 
                +---> Jalur A: WSN-31 Radio UART (Biner @115200) ----\
                +---> Jalur B: ESP-NOW WiFi (Langsung) -------------> [ ESP32-S3 Master ]
                                                                             |
                                     /---------------------------------------/
                                    | UART Link (@921600 bps)
                                    v
                       [ ESP32-S3 Slave 1 (Motion) ]
                       * Kontrol 4 Motor Mecanum (BTS7960)
                       * Sensor IMU MPU6050 & Odometri Roda
                       * Auto-Tuning PID Closed-Loop & CLI
                                    |
                                    | UART Link (@921600/115200 bps)
                                    v
                       [ ESP32-S3 Slave 2 (Manipulator 2) ]
                       * Sumbu W (Motor Putar / Rotasi)
                       * Sumbu Z & Y (Motor PWM + Encoder)
                       * Servo Ujung Lengan & Relay Solenoid
```

---

## 2. Getting Started (Memulai)

### Prasyarat Software & Dependency
1. **Arduino IDE** (versi 2.x direkomendasikan) ATAU **Arduino CLI**.
2. **ESP32 Board Package** (v2.0.x atau lebih baru).
3. **Library Wajib:**
   * `PS4Controller` oleh *aed3* (untuk board controller).
   * `MPU9250` oleh *hideakitai* ATAU `MPU6050` (untuk Slave 1).
   * `Encoder` oleh *Paul Stoffregen* (untuk Slave 2 / Encoder manual).
   * `Preferences` (sudah bawaan ESP32 SDK untuk NVS flash).

### Panduan Instalasi & Build (`arduino-cli`)
Berikut adalah perintah dasar untuk kompilasi dan upload menggunakan **Arduino CLI**:

```bash
# 1. Kompilasi & Upload untuk ESP32 Controller
arduino-cli compile --fqbn "esp32:esp32:esp32:PartitionScheme=huge_app" esp32controller
arduino-cli upload -p COM14 --fqbn "esp32:esp32:esp32:PartitionScheme=huge_app" esp32controller

# 2. Kompilasi & Upload untuk ESP32-S3 Master
arduino-cli compile --fqbn esp32:esp32:esp32s3 esp32s3_master
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3 esp32s3_master

# 3. Kompilasi & Upload untuk ESP32-S3 Slave 1 (Motion)
arduino-cli compile --fqbn esp32:esp32:esp32s3 esp32s3_slave1_localizer_motion
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3 esp32s3_slave1_localizer_motion

# 4. Membuka Serial Monitor
arduino-cli monitor -p /dev/ttyACM0 -b 115200
```
*Catatan: Sesuaikan nomor port serial (`COMxx` pada Windows, `/dev/ttyACMxx` atau `/dev/ttyUSBxx` pada Linux/macOS) sesuai yang terdeteksi.*

---

## 3. Project Structure (Struktur Proyek)

Struktur repositori diatur berdasarkan unit mikrokontroler fisik yang digunakan:

* **`esp32controller/` & `esp32controllerV2/`**
  Firmware untuk remote control nirkabel. Membaca stick PS4 via Bluetooth dan mengirimkan data kontrol secara berkala.
  * *File Kunci:* `esp32controller.ino` (loop utama), `config.h` (pinout & MAC target), `ps4_bluetooth.ino` (driver input).

* **`esp32s3_master/`**
  Otak utama robot. Menerima data joystick, mengelola state strategi robot, mengontrol manipulator 1 secara lokal, dan meneruskan perintah pergerakan ke Slave 1 & Slave 2.
  * *File Kunci:* `esp32s3_master.ino`, `mecanum_control.ino` (logika kemudi), `gripper_control.ino` (manipulator 1), `armbox_control.ino` (logika manipulator 2).

* **`esp32s3_slave1_localizer_motion/`**
  Sistem navigasi, lokalisasi, dan penggerak sasis. Menghitung odometri sasis, membaca data orientasi Yaw dari IMU, menerapkan PID closed-loop motor, dan melakukan auto-tuning PID motor roda secara otomatis.
  * *File Kunci:* `esp32s3_slave1_localizer_motion.ino`, `kinematik.ino` (Mecanum equations), `mpu.ino` (fusion filter yaw), `autoTuner.ino` (autotuner PID), `serial_commands.ino` (CLI).

* **`esp32s3_slave2_manipulator2/`**
  Sistem kontrol lengan manipulator 2 (pengambilan box). Mengatur pergerakan 3 sumbu (W, Z, Y) berbasis PID position hold, limit switch, servo gripper/wrist, dan solenoid valve relay.
  * *File Kunci:* `esp32s3_slave2_manipulator2.ino`, `arm.ino` (logika target & homing), `serial.ino` (parser command UART).

* **`example/` & `UjiCoba/`**
  Kumpulan file contoh pengujian komponen individual (motor, servo, encoder, auto-tuner terisolasi).

---

## 4. Development Workflow & Coding Standards

Demi kestabilan sistem tertanam (embedded) dan kemudahan kolaborasi tim, ikuti aturan standar berikut:

### Aturan Kompilasi Multi-File Arduino IDE
1. Main `.ino` (misal `esp32s3_master.ino`) harus ada di root subfolder dan akan dikompilasi pertama secara alfabetis.
2. File tambahan di dalam folder yang sama (misal `motor.ino`, `encoder.ino`) dikompilasi bersama secara otomatis.
3. Semua deklarasi fungsi global dan tipe data bersama **wajib diletakkan di `robot_config.h`** (bukan di dalam `.ino` lain) agar tidak terjadi error kompilasi *undeclared identifier*.
4. Gunakan `extern` di `robot_config.h` untuk variabel global, dengan inisialisasi aktual di main `.ino`.

### Konvensi Clean Code Arduino
* **Hapus Dead Code:** Fungsi yang tidak dipanggil di mana pun harus dihapus bersih (bukan hanya di-comment).
* **Penamaan Deskriptif (Bahasa Indonesia/Inggris):** Hindari nama variabel pendek yang membingungkan. Gunakan `motorTargetRpm` (bukan `pwm`), `limitSwitchState` (bukan `val`).
* **Satuan di Nama Variabel:** Wajib menyertakan satuan sebagai akhiran nama: `_deg` (derajat), `_rad` (radian), `_rpm` (RPM), `_ms` (milidetik), `_sec` (detik).
* **Single Responsibility:** Satu fungsi hanya melakukan satu tugas (misal: `pidComputeYaw` hanya menghitung kalkulasi PID, `driveFieldCentric` hanya menangani rotasi koordinat).
* **Konstanta:** Gunakan `static constexpr` untuk konstanta konfigurable lokal guna optimasi memori.
* **Constrain Output:** Batasi output PWM motor hasil PID dengan batas maksimum (`±maxPwm` / `±1023`).
* **Zero Dynamic Allocation:** Dilarang keras menggunakan kelas `String` Arduino yang memicu fragmentasi memori (heap exhaustion) dan membuat ESP32 crash secara acak. Gunakan buffer statis char dan format `snprintf`/`printf`.

---

## 5. Key Concepts (Konsep Kunci)

### Navigasi Field-Centric vs Robot-Centric
* **Robot-Centric:** Pergerakan maju/mundur/geser didasarkan pada arah hadap sasis robot.
* **Field-Centric:** Arah gerak robot disesuaikan dengan arah koordinat lapangan global. Saat stick ditekan ke depan, robot akan bergerak ke depan lapangan meskipun badan robot sedang berputar. Hal ini dicapai dengan menggunakan matriks rotasi 2D berdasarkan pembacaan sudut Yaw dari sensor IMU:
  $$\Delta X_{global} = \Delta X_{local} \cos(\theta) - \Delta Y_{local} \sin(\theta)$$
  $$\Delta Y_{global} = \Delta X_{local} \sin(\theta) + \Delta Y_{local} \cos(\theta)$$

### Auto-Tuning PID Closed-Loop (Slave 1)
Sasis roda mecanum membutuhkan nilai PID yang presisi agar robot bergerak lurus tanpa melenceng. Slave 1 dilengkapi dengan state-machine Auto-Tuner non-blocking:
1. **Welford's Online Algorithm:** Digunakan untuk mengkalkulasi rata-rata error, overshoot, rise-time, dan variansi kestabilan RPM motor secara real-time tanpa mengonsumsi banyak RAM.
2. **Heuristic Adjustment:** Sistem otomatis menaikkan/menurunkan parameter $K_p$, $K_i$, $K_d$ di setiap siklus pengujian berdasarkan skor penalti performa motor. Hasil optimal otomatis disimpan langsung ke NVS Preferences dengan nama namespace `"pid_tuning"`.

### Failsafe Watchdog (Keamanan Robot)
* Jika koneksi UART dari Master terputus (packet loss) selama **lebih dari 500 milidetik**, Slave 1 akan secara otomatis mereset seluruh akumulasi integral PID dan menghentikan seluruh putaran roda (motor target RPM = 0) untuk mencegah robot melaju liar secara tidak terkendali.

---

## 6. Common Tasks (Tugas Umum)

### 1. Menjalankan Auto-Tuning PID Roda (Slave 1)
Saat mengganti motor PG45 atau roda mecanum baru, disarankan untuk melakukan tuning ulang parameter PID:
1. Hubungkan PC ke port Serial USB Slave 1.
2. Buka Serial Monitor pada baud rate `115200`.
3. Ketik perintah: `AUTOTUNE RPM ALL` lalu tekan Enter.
4. Sasis robot akan memutar roda satu per satu secara otomatis. Pastikan robot dalam kondisi digantung (tidak menyentuh lantai).
5. Setelah selesai, parameter Kp, Ki, Kd baru akan otomatis tersimpan ke memori flash ESP32 (NVS).

### 2. Kalibrasi Sensor IMU (Slave 1)
Jika pembacaan arah hadap robot (Yaw) mengalami pergeseran (drift) yang parah:
1. Letakkan robot di permukaan datar dan diamkan.
2. Hubungkan ke Serial Monitor Slave 1.
3. Ketik perintah: `CALIB_GYRO` dan tekan Enter.
4. ESP32-S3 akan memproses kalibrasi baseline sensor gyro dan menyimpannya di flash. Proses ini memerlukan waktu beberapa detik di mana robot **tidak boleh digoyang sama sekali**.

### 3. Mengontrol Gerakan Sasis via Serial CLI (Slave 1)
Untuk menguji gerakan sasis mecanum tanpa menggunakan controller PS4:
* Menggerakkan robot maju dengan kecepatan 100 secara Robot-Centric selama 2 detik:
  `ROBOT 100 0 0 2000`
* Bergerak geser kanan secara Field-Centric:
  `FIELD 0 100 0`
* Berhenti darurat seketika:
  `STOP`

---

## 7. Troubleshooting (Pemecahan Masalah)

### Masalah 1: Sumbu Lengan Bergerak ke Arah yang Salah
* **Penyebab:** Polaritas motor BTS7960 terbalik atau kabel Fasa A/B encoder terpasang terbalik.
* **Solusi:** Tukar posisi kabel fisik Fasa A dan Fasa B pada encoder, ATAU balik logika arah perhitungan encoder pada fungsi interupsi (`ticks++` menjadi `ticks--` atau sebaliknya) di file `encoder.ino`.

### Masalah 2: Limit Switch Terbaca False-Trigger (Noise)
* **Penyebab:** Noise elektromagnetik tinggi dari motor DC mempengaruhi pin input digital ESP32-S3.
* **Solusi:** Terapkan teknik **double-read** dengan jeda waktu pendek (2 milidetik) sebelum menyimpulkan status limit switch tertekan:
  ```cpp
  bool bacaLimitSwitch(int pin) {
      if (digitalRead(pin) == LOW) {
          delayMicroseconds(2000); // Jeda filter noise
          return digitalRead(pin) == LOW;
      }
      return false;
  }
  ```

### Masalah 3: Kehilangan Data Komunikasi UART Master <-> Slave
* **Penyebab:** Baud rate terlalu tinggi (921600 bps) tanpa kabel ground bersama (*common ground*), atau ukuran buffer serial bawaan terlalu kecil.
* **Solusi:**
  1. Hubungkan pin GND dari seluruh board ESP32-S3 secara langsung dengan kabel tembaga tebal.
  2. Pilin kabel TX/RX bersama kabel GND (twisted-pair).
  3. Perbesar buffer penerima UART dengan memanggil `Serial1.setRxBufferSize(1024);` sebelum `Serial1.begin(...)` di kode setup.

### Masalah 4: Mismatch Baud Rate Master <-> Slave 2 (Manipulator 2)
* **Penyebab:** `esp32s3_master/manipulator_serial.ino` diinisialisasi pada baud rate `921600`, sedangkan dokumentasi `esp32s3_slave2_manipulator2/README.md` menyatakan baud rate UART menggunakan `115200`.
* **Solusi:** Jika komunikasi serial ke Slave 2 tidak merespon, periksa baud rate yang diatur pada pemanggilan `Serial2.begin` di kedua sisi dan samakan (gunakan `115200` untuk keandalan kabel panjang, atau `921600` untuk kecepatan tinggi).

---

## 8. References (Referensi & Dokumentasi Pendukung)

Untuk informasi lebih rinci mengenai bagian-bagian spesifik, silakan baca dokumentasi internal berikut:
* `workflow.md` — Panduan alur kerja arsitektur sistem multi-MCU murni non-blocking.
* `esp32s3_slave1_localizer_motion/documentasi_project.md` — Detail fungsi modular sasis, kinematik, dan driver odometri.
* `esp32s3_slave1_localizer_motion/documentasi_komunikasi_master.md` — Detail protokol data biner serial Master ke Slave 1.
* `esp32s3_slave1_localizer_motion/documentasi_perintah_serial.md` — Daftar lengkap perintah CLI Serial Monitor.
* `esp32s3_slave2_manipulator2/README.md` — Spesifikasi pinout, preset servo, relay, dan perintah kontrol lengan box.
* `esp32s3_slave2_manipulator2/WIRING_DIAGRAM.md` — Diagram pengawatan sirkuit manipulator 2.
