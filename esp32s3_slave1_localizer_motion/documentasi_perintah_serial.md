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

### B. Perintah Auto-Tuning PID RPM

Sistem menggunakan parameter PID RPM terpisah untuk masing-masing roda. Gunakan perintah ini untuk melakukan tuning otomatis.

#### 1. `AUTOTUNE RPM ALL`
* **Deskripsi:** Menjalankan siklus pencarian parameter PID ($K_p, K_i, K_d$) secara otomatis untuk ke-4 roda berurutan (Front Right -> Front Left -> Back Right -> Back Left). Parameter terbaik otomatis disimpan ke NVS.
* **Contoh:**
  ```text
  AUTOTUNE RPM ALL
  ```

#### 2. `AUTOTUNE RPM <idx> [Kp] [Ki] [Kd]`
* **Deskripsi:** Melakukan auto-tuning khusus pada satu motor tertentu saja. Anda dapat (secara opsional) memberikan nilai awal Kp, Ki, dan Kd untuk mempercepat proses pencarian (konvergensi) jika Anda sudah mengetahui kisaran angka yang bagus. Jika nilai awal ini dikosongkan, robot akan otomatis menggunakan nilai *default safety* (Kp=0.1, Ki=0.5, Kd=0.0).
* **Parameter:**
  * `<idx>`: Indeks roda/motor (`0`: Front Right, `1`: Front Left, `2`: Back Right, `3`: Back Left).
  * `[Kp]` *(Opsional)*: Nilai tebakan awal untuk parameter Proportional.
  * `[Ki]` *(Opsional)*: Nilai tebakan awal untuk parameter Integral.
  * `[Kd]` *(Opsional)*: Nilai tebakan awal untuk parameter Derivative.
* **Contoh 1 (Tuning dari default safety):**
  ```text
  AUTOTUNE RPM 0
  ```
* **Contoh 2 (Fast Tuning dengan tebakan nilai awal Kp=10.5, Ki=2.0, Kd=0.1):**
  ```text
  AUTOTUNE RPM 1 10.5 2.0 0.1
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
* **Deskripsi:** Memutar keempat roda secara bersamaan dengan target RPM masing-masing menggunakan closed-loop PID.
* **Parameter:**
  * `<r1>`: RPM Front Right (Roda 0)
  * `<r2>`: RPM Front Left (Roda 1)
  * `<r3>`: RPM Back Right (Roda 2)
  * `<r4>`: RPM Back Left (Roda 3)
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
* **Deskripsi:** Memutar satu roda secara langsung (*open-loop* tanpa PID) menggunakan sinyal PWM selama durasi waktu tertentu. Sangat berguna untuk menguji hardware driver BTS7960 dan orientasi kabel.
* **Parameter:**
  * `<idx>`: Indeks roda (`0`-`3`).
  * `<pwm>`: Nilai duty cycle PWM (`-1023` s.d `1023`).
  * `<ms>`: Durasi waktu dalam milidetik.
* **Contoh:**
  ```text
  MOVE PWM 2 500 2000
  ```

#### 5. `SEQ RPM <rpm> <ms>`
* **Deskripsi:** Melakukan pengujian sekuensial (bergantian) untuk memastikan urutan pemasangan roda sudah benar. Memutar Roda 0 $\rightarrow$ Roda 1 $\rightarrow$ Roda 2 $\rightarrow$ Roda 3 masing-masing selama `<ms>` milidetik pada target `<rpm>`.
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

Gunakan perintah ini untuk menguji kinematik mekanis dari sasis mecanum 4 roda.

#### 1. `ROBOT <vx> <vy> <w> <ms>`
* **Deskripsi:** Menggerakkan sasis robot berdasarkan **Robot-Centric Kinematics** (maju/geser relatif terhadap arah hadap sasis robot) selama durasi waktu tertentu.
* **Parameter:**
  * `<vx>`: Kecepatan linear maju-mundur (X-axis).
  * `<vy>`: Kecepatan linear geser kanan-kiri (Y-axis).
  * `<w>`: Kecepatan putar sudut / angular yaw (rotasi sasis).
  * `<ms>`: Durasi pergerakan dalam milidetik.
* **Contoh (Maju lurus selama 2 detik):**
  ```text
  ROBOT 100 0 0 2000
  ```
* **Contoh (Geser kanan lurus selama 2 detik):**
  ```text
  ROBOT 0 100 0 2000
  ```

#### 2. `FIELD <vx> <vy> <w> <ms>`
* **Deskripsi:** Menggerakkan sasis robot berdasarkan **Field-Centric Kinematics** (maju/geser relatif terhadap arah koordinat lapangan global, menggunakan feedback orientasi Yaw dari IMU MPU6050) selama durasi waktu tertentu.
* **Parameter:**
  * `<vx>`: Kecepatan linear maju-mundur relatif terhadap lapangan global.
  * `<vy>`: Kecepatan linear geser kanan-kiri relatif terhadap lapangan global.
  * `<w>`: Kecepatan putar sudut sasis.
  * `<ms>`: Durasi pergerakan dalam milidetik.
* **Contoh:**
  ```text
  FIELD 100 0 0 2500
  ```

---

## 3. PANDUAN DE-BUGGING CEPAT
1. **Roda Berputar Terbalik saat Di-test?**
   * Kirim perintah `MOVE PWM 0 300 2000` (Front Right), perhatikan arah putar roda. Jika roda berputar ke belakang, arah pin arah pada BTS7960 terbalik atau orientasi kutub motor terbalik.
2. **Urutan Roda Salah?**
   * Kirim perintah `SEQ PWM 400 1500`. Roda harus berputar berurutan secara fisik dari: Front Right (Roda 0) $\rightarrow$ Front Left (Roda 1) $\rightarrow$ Back Right (Roda 2) $\rightarrow$ Back Left (Roda 3).
3. **Mereset Nilai Hasil Tuning?**
   * Jalankan `AUTOTUNE RPM ALL`, biarkan siklus 12 kali selesai per motor. Hasil tuning terbaik otomatis dimuat ke sistem.
