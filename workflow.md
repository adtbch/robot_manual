# PANDUAN TEKNIS & ALUR KERJA (WORKFLOW) FIRMWARE ESP32-S3
## ARSITEKTUR MULTI-MCU ROBOT MANUAL KRAI 2026
*(Metodologi Pengembangan: Standar Arduino API Murni - Tanpa FreeRTOS)*

Dokumen ini berfungsi sebagai cetak biru arsitektur perangkat lunak (*software architecture*) yang sepenuhnya menggunakan kemudahan **Arduino Framework** standar pada tiga chip **ESP32-S3**.

Keputusan arsitektur: **Didesain untuk Pemula**. Kita TIDAK menggunakan FreeRTOS (seperti *Task*, *Mutex*, atau *Semaphore*) dan TIDAK menggunakan driver native tingkat rendah (seperti `pcnt.h` atau `mcpwm.h`). Semua kode menggunakan struktur *Super-Loop* dengan fungsi asinkron standar `millis()` agar mudah dipahami, di-debug, dan dikembangkan oleh pemula (seperti siswa SMA/SMK atau mahasiswa tingkat 1).

---

## 1. ARSITEKTUR HARDWARE & ALIRAN DATA SISTEM

Setiap unit ESP32-S3 menjalankan program di satu *Core* utama dengan metode *Super-Loop* (`void setup()` dan `void loop()`).

```text
  +-------------------------------------------------+
  |             ESP32-S3 JOYSTICK (REMOTE)          |
  +------------------------+------------------------+
                           |
                           | Radio UART Nirkabel (Modul Transceiver)
                           v
  +-------------------------------------------------+
  |             ESP32-S3 SLAVE 1 (MOTION)           | <=== Fungsi di loop(): 
  |  - 4x Motor Mecanum (Encoder via Interrupt)     |      - Baca MPU6050 (I2C) & Hitung Odometri
  |  - 4x Roda Omni Eksternal (Odometry)            |      - Parsing UART Master (Target Navigasi)
  |  - IMU MPU6050 (Koneksi I2C Wire.h @400kHz)     |      - Update PID Navigasi & Inverse Kinematics
  |                                                 |      - Update PID RPM & PWM (analogWrite)
  |                                                 |      - AutoTuner (Jika Aktif)
  +------------------------^------------------------+
                           |
                           | UART Protokol (11-Byte Frame @921600 bps)
                           v
  +-------------------------------------------------+
  |             ESP32-S3 MASTER (MANIPULATOR 1)     | <=== Fungsi di loop():
  |  - Sumbu X & Z Lengan 1 (Motor PWM + Encoder)   |      - State Machine Strategi (Game Logic)
  |  - 2x Limit Switch (Proteksi & Homing)          |      - Parsing Input dari Slave 1 (Joystick)
  |  - 2x Servo (Capit & Rotasi)                    |      - Update PID Lengan 1 & Cek Limit Switch
  |                                                 |      - Kirim Target ke Slave 1 & Slave 2
  +------------------------+------------------------+
                           |
                           | UART Protokol (12-Byte Frame @921600 bps)
                           v
  +-------------------------------------------------+
  |             ESP32-S3 SLAVE 2 (MANIPULATOR 2)    | <=== Fungsi di loop():
  |  - Base Rotation / Putar (Motor PWM + Encoder)  |      - Parsing UART Master (Target Posisi)
  |  - Sumbu Z & Y Lengan 2 (Motor PWM + Encoder)   |      - Cek Failsafe Timeout
  |  - Wrist / Ujung Lengan (Servo)                 |      - Update PID 3 Sumbu & Gerakkan PWM
  +-------------------------------------------------+
```

---

## 2. KONSEP PEMROGRAMAN ARDUINO STANDAR (NON-BLOCKING)

Karena kita tidak menggunakan FreeRTOS untuk membagi tugas secara paksa, kunci agar robot tidak "nge-lag" atau *hang* adalah **Pemrograman Non-Blocking dengan `millis()`** dan **Penggunaan Interupsi Hardware (`attachInterrupt`)**.

### A. Membaca Encoder Motor dengan Interupsi Standar
Alih-alih menggunakan `pcnt.h` (ESP-IDF), kita menggunakan fungsi Arduino `attachInterrupt()` pada pin fasa A encoder untuk mendeteksi putaran motor:
```cpp
volatile int32_t ticks_motor_1 = 0;

void IRAM_ATTR hitung_encoder_motor_1() {
    // Baca Fasa B untuk menentukan arah putaran
    if (digitalRead(PIN_ENC_FL_B) == HIGH) {
        ticks_motor_1++;
    } else {
        ticks_motor_1--;
    }
}

void setup() {
    pinMode(PIN_ENC_FL_A, INPUT_PULLUP);
    pinMode(PIN_ENC_FL_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_FL_A), hitung_encoder_motor_1, RISING);
}
```

### B. Menggerakkan Motor dengan `analogWrite()` (Standar Arduino)
Alih-alih menggunakan `mcpwm.h` (ESP-IDF) yang kompleks, kita menggunakan perintah `analogWrite()` standar untuk mengatur kecepatan motor:
```cpp
void putar_motor_roda_kiri_depan(int kecepatan_pwm) { // kecepatan -255 s.d 255
    if (kecepatan_pwm > 0) {
        analogWrite(PIN_MOTOR_FL_RPWM, kecepatan_pwm);
        analogWrite(PIN_MOTOR_FL_LPWM, 0);
    } else if (kecepatan_pwm < 0) {
        analogWrite(PIN_MOTOR_FL_RPWM, 0);
        analogWrite(PIN_MOTOR_FL_LPWM, -kecepatan_pwm);
    } else {
        analogWrite(PIN_MOTOR_FL_RPWM, 0);
        analogWrite(PIN_MOTOR_FL_LPWM, 0);
    }
}
```

### C. Menghindari `delay()` (Non-Blocking Loop)
Pemisahan frekuensi pembaruan fungsi (misal: PID jalan tiap 20ms, lokalisasi tiap 10ms) dikendalikan penuh menggunakan pencatat waktu `millis()`:
```cpp
uint32_t waktu_terakhir_pid = 0;

void loop() {
    uint32_t waktu_sekarang = millis();
    
    // Fungsi ini dipanggil secepat mungkin tanpa dibatasi
    baca_dan_terjemahkan_perintah_masuk();
    
    // Fungsi ini dieksekusi HANYA jika sudah lewat 20 milidetik (50 Hz)
    if (waktu_sekarang - waktu_terakhir_pid >= 20) {
        float selisih_waktu_detik = (waktu_sekarang - waktu_terakhir_pid) / 1000.0;
        waktu_terakhir_pid = waktu_sekarang;
        
        perbarui_posisi_robot_di_lapangan();
        kendalikan_kecepatan_roda_otomatis(selisih_waktu_detik);
    }
}
```

---

## 3. PARSING SERIAL UART BINARY NON-BLOCKING

Gunakan metode pembacaan byte demi byte berbasis State Machine di dalam loop Arduino untuk menghindari delay pembacaan buffer serial:

```cpp
void baca_dan_terjemahkan_perintah_masuk() {
    static int state = 0;
    static uint8_t buffer[sizeof(paket_koordinat_t)];
    
    while (Serial1.available() > 0) {
        uint8_t byte_in = Serial1.read();
        
        // ... Logika state machine parsing ...
    }
}
```

---

## 4. CARA KOMPILASI MENGGUNAKAN ARDUINO IDE

1. Buka **Arduino IDE**.
2. Masuk ke **Tools -> Board -> Boards Manager**, cari **esp32**, tekan **Install**.
3. Pilih Board: **ESP32S3 Dev Module**.
4. Atur konfigurasi penting:
   * **USB CDC On Boot**: *Enabled*.
   * **Flash Size**: *8MB* atau *16MB*.

---

## 5. PANDUAN KOMUNIKASI SERIAL BERKECEPATAN TINGGI (921600 BPS)

1. **Panjang Kabel Maksimal 15-20 cm:** Pastikan letak ESP Master, Slave 1, dan Slave 2 berdekatan.
2. **Pilin Kabel Ground (Twisted GND Pair):** Pilin kabel RX dan TX bersama kabel Ground (GND).
3. **Hubungkan Seluruh Ground (Common Ground):** Semua pin GND dari ketiga ESP32-S3 wajib menyatu.
4. **Gunakan Ring Buffer yang Lebih Besar:** `Serial1.setRxBufferSize(1024);` sebelum memanggil `Serial1.begin()`.
5. **Deteksi Data Rusak Menggunakan Checksum:** Wajib.

---

## 6. STRATEGI PEMBAGIAN STRUKTUR FILE YANG MODULAR

```text
robot_manual/
│
├── esp32s3_master_manipulator1/        # PROYEK 1: ESP MASTER
│   ├── esp32s3_master_manipulator1.ino
│   ├── pinout.h
│   ├── config.h
│   ├── motor_arm1_pid_control.h / .cpp
│   └── uart_master_transmitter.h / .cpp
│
├── esp32s3_slave1_localizer_motion/    # PROYEK 2: ESP SLAVE 1
│   ├── esp32s3_slave1_localizer_motion.ino
│   ├── pinout.h
│   ├── config.h
│   ├── sensor_omni_odometry.h / .cpp
│   ├── motor_mecanum_kinematics.h / .cpp
│   ├── pid_autotuner.h / .cpp
│   └── uart_slave_receiver.h / .cpp
│
└── esp32s3_slave2_manipulator2/        # PROYEK 3: ESP SLAVE 2
    ├── esp32s3_slave2_manipulator2.ino
    ├── pinout.h
    ├── config.h
    ├── motor_arm2_pid_control.h / .cpp
    └── uart_slave_receiver.h / .cpp
```

---

## 7. TACTICAL TROUBLESHOOTING CHEATSHEET (PANDUAN LAPANGAN)

| Gejala Kerusakan | Kemungkinan Penyebab | Tindakan Diagnosis / Solusi |
| :--- | :--- | :--- |
| **Sumbu lengan bergerak ke arah salah** | Fasa sinyal motor BTS7960 terbalik atau koneksi A/B encoder terbalik | Tukar koneksi kabel Fasa A dan Fasa B pada encoder fisik, ATAU balik nilai penambahan `ticks++` menjadi `ticks--` pada fungsi interrupt. |
| **Sumbu lengan menabrak batas mekanik** | Sensor limit switch tidak terbaca atau rusak | Ukur tegangan pin limit switch. Pastikan pada loop utama ada pengecekan rutin untuk memotong sinyal PWM. |
| **Program Robot terasa lambat / nge-lag** | Terdapat `delay()` pada kode, atau ada proses di dalam `loop()` yang memakan waktu lama | Hapus semua fungsi `delay()` dan ganti dengan sistem waktu `millis()`. Matikan *print debugging* yang tidak perlu. |
| **Pergerakan Roda Mecanum terputus-putus** | Packet loss pada UART akibat baudrate tinggi | Pasang kabel ground tembaga tebal antar-ESP. Naikkan buffer RX serial dengan `Serial1.setRxBufferSize(1024)`. |

---

## 8. STRATEGI MENULIS KODE YANG RAMAH PEMULA (EMBEDDED CLEAN CODE)

Terapkan 5 pilar "Clean Code Arduino" ini di seluruh file proyek Anda:

### A. Penamaan Bahasa Indonesia yang Deskriptif
* **🛑 Buruk:** `init_mt()` , `updPos()` , `drvM()`
* **✔️ Bersih:** `siapkan_pin_penggerak_roda()`, `perbarui_posisi_robot_di_lapangan()`, `putar_motor_roda_kiri_depan()`

### B. Single Responsibility (Satu Fungsi Mengerjakan Satu Tugas Saja)
Pecah blok fungsi yang sangat panjang menjadi fungsi kecil-kecil agar mudah dicari kerusakannya.
* **✔️ Bersih:** Memisahkan fungsi `hitung_rumus_pid(...)`, fungsi `baca_putaran_encoder()`, dan `putar_motor_roda(...)`.

### C. Hindari Variabel Global Liar (Bungkus Variabel ke Struct)
```cpp
struct PengaturPID {
    float kp, ki, kd;
    float error_sebelumnya;
    float akumulasi_integral;
};
PengaturPID pid_roda_kiri = {1.2, 0.01, 0.1, 0.0, 0.0};
```

### D. Zero Dynamic Allocation (Dilarang Menggunakan Tipe Data `String`)
* **🛑 Pemicu Crash/Hang:** `Serial.println("X: " + String(posisi_x));`
* **✔️ Solusi Aman (Format `printf`):** `Serial.printf("X: %.2f\n", posisi_x);`

### E. Desain Berbasis State Machine (Status Kerja)
Gunakan *switch-case* dipadukan dengan *enum* untuk mengontrol tahapan gerakan robot.

---

## 9. SISTEM AUTO-TUNER PID TERINTEGRASI (SLAVE 1)

Proyek ESP Slave 1 kini dilengkapi dengan modul **Auto-Tuner PID dinamis**.
Sistem ini menggunakan metode State-Machine non-blocking untuk secara otomatis mencari nilai `Kp, Ki, Kd` terbaik untuk keempat motor roda mecanum secara bergiliran.
*   **Cara Pakai**: Kirimkan perintah teks `"autotune"` melalui Serial Monitor pada ESP Slave 1. Robot akan secara otomatis menguji motor roda 1 hingga roda 4 lalu mencetak nilai parameter terbaik.
