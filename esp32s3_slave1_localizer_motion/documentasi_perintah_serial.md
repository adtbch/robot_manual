# DOKUMENTASI ANTARMUKA PERINTAH SERIAL (SERIAL CLI)
## ESP32-S3 SLAVE 1: MOTION & LOKALISASI

Dokumen ini menjelaskan seluruh perintah baris (CLI) yang didukung oleh **ESP32-S3 Slave 1** melalui koneksi Serial USB utama (`Serial0` USB virtual atau Pin RX/TX utama) pada kecepatan **115200 bps**. Fitur ini diimplementasikan di `serial_commands.ino` dan diproses secara non-blocking dalam super-loop utama robot.

---

## 1. ANTARMUKA SERIAL & SAFETY FIRST
* **Baud Rate:** `115200`
* **Format Baris:** Diakhiri dengan karakter Newline (`\n` atau LF).
* **Case-Insensitive:** Perintah dapat ditulis dalam huruf besar maupun kecil (otomatis dikonversi ke UPPERCASE).
* **Fitur Utama Keamanan (STOP):** Kapan saja robot bergerak liar atau Anda ingin membatalkan autotuning, kirimkan perintah `STOP` untuk mematikan seluruh motor seketika dan mereset memori integral PID.

---

## 2. DAFTAR PERINTAH BARIS (CLI)

### A. Perintah Darurat & Bantuan

#### 1. `STOP`
* **Deskripsi:** Menghentikan seluruh putaran motor secara darurat (*Emergency Stop*) dan menggagalkan proses Auto-Tuning yang sedang berjalan.
* **Contoh:**
  ```text
  STOP
  ```
* **Respon Serial:** `EMERGENCY STOP!`

#### 2. `HELP`
* **Deskripsi:** Menampilkan panduan cepat format seluruh perintah yang didukung ke Serial Monitor.
* **Contoh:**
  ```text
  HELP
  ```

---

### B. Perintah Auto-Tuning & Reset PID RPM

Sistem menggunakan parameter PID RPM terpisah untuk masing-masing roda. Gunakan perintah ini untuk melakukan tuning otomatis atau mereset data.

#### 1. `AUTOTUNE RPM ALL`
* **Deskripsi:** Menjalankan siklus pencarian parameter PID ($K_p, K_i, K_d$) secara otomatis untuk ke-4 roda berurutan (Front Right -> Front Left -> Back Right -> Back Left). Parameter terbaik otomatis disimpan ke NVS.
* **Contoh:**
  ```text
  AUTOTUNE RPM ALL
  ```

#### 2. `AUTOTUNE RPM <idx> [Kp] [Ki] [Kd]`
* **Deskripsi:** Melakukan auto-tuning khusus pada satu motor tertentu saja. Anda dapat (secara opsional) memberikan nilai awal Kp, Ki, dan Kd untuk mempercepat proses pencarian (konvergensi). Jika dikosongkan, robot menggunakan nilai *default safety* (Kp=0.1, Ki=0.5, Kd=0.0).
* **Parameter:**
  * `<idx>`: Indeks roda/motor (`0`: Front Right, `1`: Front Left, `2`: Back Right, `3`: Back Left).
  * `[Kp]` *[Opsional]*: Tebakan awal Proportional.
  * `[Ki]` *[Opsional]*: Tebakan awal Integral.
  * `[Kd]` *[Opsional]*: Tebakan awal Derivative.
* **Contoh:**
  ```text
  AUTOTUNE RPM 1 10.5 2.0 0.1
  ```

#### 3. `RESET_PID`
* **Deskripsi:** Menghapus seluruh data parameter PID RPM yang tersimpan di NVS (Flash) dan memuat kembali nilai *default*.
* **Contoh:**
  ```text
  RESET_PID
  ```

---

### C. Perintah Kendali & Tes Motor Roda

#### 1. `MOTOR <idx> <rpm>`
* **Deskripsi:** Memutar satu roda tertentu secara kontinu pada target RPM tertentu menggunakan closed-loop PID.
* **Parameter:**
  * `<idx>`: Indeks roda (`0`-`3`).
  * `<rpm>`: Kecepatan target RPM (misal `100` atau `-150` untuk mundur).
* **Contoh:**
  ```text
  MOTOR 1 120
  ```

#### 2. `MOTORS <r1> <r2> <r3> <r4>`
* **Deskripsi:** Memutar keempat roda secara bersamaan secara kontinu dengan target RPM masing-masing menggunakan closed-loop PID.
* **Parameter:**
  * `<r1>` s.d `<r4>`: Target RPM masing-masing roda.
* **Contoh:**
  ```text
  MOTORS 100 100 100 100
  ```

#### 3. `MOVE RPM <idx> <rpm> <ms>`
* **Deskripsi:** Memutar satu roda dengan target RPM tertentu selama durasi waktu tertentu, lalu otomatis berhenti.
* **Parameter:**
  * `<idx>`: Indeks roda (`0`-`3`).
  * `<rpm>`: Target RPM.
  * `<ms>`: Durasi putaran dalam milidetik.
* **Contoh:**
  ```text
  MOVE RPM 0 200 3000
  ```

#### 4. `MOVE PWM <idx> <pwm> <ms>`
* **Deskripsi:** Memutar satu roda secara langsung (*open-loop* tanpa PID) menggunakan sinyal PWM selama durasi waktu tertentu.
* **Parameter:**
  * `<idx>`: Indeks roda (`0`-`3`).
  * `<pwm>`: Nilai duty cycle PWM (`-1023` s.d `1023`).
  * `<ms>`: Durasi waktu dalam milidetik.
* **Contoh:**
  ```text
  MOVE PWM 2 500 2000
  ```

#### 5. `SEQ RPM <rpm> <ms>`
* **Deskripsi:** Memutar Roda 0 $\rightarrow$ Roda 1 $\rightarrow$ Roda 2 $\rightarrow$ Roda 3 bergantian masing-masing selama `<ms>` milidetik pada target `<rpm>`.
* **Contoh:**
  ```text
  SEQ RPM 150 1500
  ```

#### 6. `SEQ PWM <pwm> <ms>`
* **Deskripsi:** Sama seperti `SEQ RPM`, tetapi menggunakan kontrol *open-loop* dengan sinyal duty-cycle `<pwm>`.
* **Contoh:**
  ```text
  SEQ PWM 400 1000
  ```

---

### D. Perintah Pengujian Gerakan Kinematik Sasis

#### 1. `ROBOT <vx> <vy> <w> <ms>`
* **Deskripsi:** Menggerakkan sasis robot berdasarkan **Robot-Centric Kinematics** (relatif terhadap badan robot) selama durasi waktu tertentu.
* **Parameter:**
  * `<vx>`: Kecepatan linear maju-mundur.
  * `<vy>`: Kecepatan linear geser kanan-kiri.
  * `<w>`: Kecepatan putar sasis.
  * `<ms>`: Durasi pergerakan dalam milidetik.
* **Contoh:**
  ```text
  ROBOT 100 0 0 2000
  ```

#### 2. `FIELD <vx> <vy> <w> <ms>`
* **Deskripsi:** Menggerakkan sasis robot berdasarkan **Field-Centric Kinematics** (relatif terhadap koordinat lapangan global, menggunakan data yaw dari IMU MPU9250) selama durasi waktu tertentu.
* **Parameter:**
  * `<vx>`: Kecepatan linear maju-mundur relatif lapangan.
  * `<vy>`: Kecepatan linear geser kanan-kiri relatif lapangan.
  * `<w>`: Kecepatan putar sasis.
  * `<ms>`: Durasi pergerakan dalam milidetik.
* **Contoh:**
  ```text
  FIELD 100 0 0 2500
  ```

---

### E. Perintah IMU / MPU9250 & Yaw

#### 1. `CALIB_GYRO`
* **Deskripsi:** Menjalankan kalibrasi penuh sensor gyroscope, accelerometer, dan magnetometer (kecuali pada mode gyro-only di mana mag di-skip). Hasil kalibrasi otomatis disimpan ke NVS.
* **Contoh:**
  ```text
  CALIB_GYRO
  ```

#### 2. `RESET_GYRO`
* **Deskripsi:** Menghapus data kalibrasi IMU dari NVS, memaksa robot untuk menggunakan nilai *default* saat boot berikutnya.
* **Contoh:**
  ```text
  RESET_GYRO
  ```

#### 3. `YAW`
* **Deskripsi:** Membaca dan menampilkan nilai sudut Yaw saat ini dalam derajat (rentang `-180.00` s.d `180.00`).
* **Contoh:**
  ```text
  YAW
  ```
* **Respon Serial:** `Yaw: 12.34 deg`

#### 4. `SET_YAW_PID <kp> <ki> <kd>`
* **Deskripsi:** Mengubah konstanta PID untuk koreksi yaw kinematik lapangan (field-centric) secara real-time dan menyimpannya ke NVS.
* **Contoh:**
  ```text
  SET_YAW_PID 2.5 0.01 0.1
  ```

#### 5. `SHOW_YAW_PID`
* **Deskripsi:** Menampilkan nilai konstanta PID yaw yang aktif saat ini.
* **Contoh:**
  ```text
  SHOW_YAW_PID
  ```
* **Respon Serial:** `Yaw PID: Kp=2.500 Ki=0.010 Kd=0.100`

---

## 3. PANDUAN DE-BUGGING CEPAT
1. **Roda Berputar Terbalik saat Di-test?**
   * Jalankan `MOVE PWM 0 300 2000` (Front Right), perhatikan arah putar roda. Jika ke belakang, ubah kutub motor atau konfigurasi arah pin pada BTS7960.
2. **Urutan Roda Salah?**
   * Kirim perintah `SEQ PWM 400 1500`. Putaran roda harus berurutan secara fisik: Front Right (0) $\rightarrow$ Front Left (1) $\rightarrow$ Back Right (2) $\rightarrow$ Back Left (3).
3. **Mereset Nilai Hasil Tuning?**
   * Gunakan `RESET_PID` untuk menghapus tuning NVS dan kembalikan ke nilai bawaan. Gunakan `RESET_GYRO` jika yaw bergeser sangat parah (drift) di luar kewajaran.
