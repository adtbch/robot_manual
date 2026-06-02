# BLUEPRINT ARSITEKTUR & ALUR ALGORITMA
## ESP32-S3 SLAVE 1: MOTION & LOKALISASI (NON-BLOCKING & MODULAR)
*(Metodologi Pengembangan: Standar Arduino API Murni - Tanpa FreeRTOS)*

Dokumen ini berfungsi sebagai panduan cetak biru konseptual (*conceptual blueprint*) yang sangat terperinci mengenai alur kerja internal, arsitektur non-blocking, serta seluruh algoritma modular yang ditanamkan pada **ESP32-S3 Slave 1** (`esp32s3_slave1_localizer_motion`). Panduan ini dirancang untuk memastikan performa maksimal tanpa delay (`delay()`) dengan pembacaan sensor dan pembaruan koordinat secara kontinu (*real-time*).

---

## 1. STRUKTUR UTAMA PROGRAM (PANTANG `delay()` - FULLY NON-BLOCKING)

Program berjalan menggunakan metode **Super-Loop Non-Blocking** (satu thread loop linear berurutan). Seluruh pembacaan sensor (Encoder, IMU/MPU, Radio/Serial UART) dan pembaruan koordinat dilakukan **setiap saat** pada setiap iterasi `loop()` tanpa hambatan waktu. Untuk bagian kontrol yang membutuhkan waktu sampel tertentu (misalnya perhitungan PID), digunakan **metode interval asinkron berbasis `millis()` atau kalkulasi `dt` dinamis** tanpa pernah menghentikan sistem (*blocking*).

### Visualisasi Alur Kerja Super-Loop Non-Blocking

```text
               +----------------------------------------+
               |     MULAI (void setup())               |
               |     - siapkan_penerima_perintah()      |
               |     - siapkan_semua_motor_roda()       |
               |     - siapkan_sensor_lokasi()          |
               +-------------------+--------------------+
                                   |
                                   v
               +----------------------------------------+
               |     void loop() (Super-Loop Utama)     |
               +-------------------+--------------------+
                                   |
         +-------------------------+-------------------------+
         |                                                   |
         | (1) SETIAP SAAT / REAL-TIME CONTINUOUS            | (2) ASINKRON / DYNAMIC dt CONTROLLER
         v                                                   v
+------------------------------------+             +--------------------------------------+
| - bacaSensor()                     |             | - hitungKinematik()                  |
|   * Baca data paket Serial Radio   |             |   * Inverse Kinematics ke Target RPM |
|   * Baca IMU MPU6050 (Gyro Z Yaw)  |             | - pidRpm()                           |
|   * Ambil data 4 Encoder Motor     |             |   * Memanggil computePID() per roda  |
| - updateOdometri()                 |             |   * Tulis PWM via setMotorSpeed()    |
|   * Forward Kinematics 4 Roda      |             | - failsafe_watchdog()                |
|   * Update Koordinat Lapangan      |             |   * Deteksi timeout UART (500ms)     |
| - deteksi_input_cli_autotune()     |             |   * Penghentian darurat jika loss    |
+------------------------------------+             +--------------------------------------+
```

---

## 2. DETAIL ALGORITMA LOKALISASI (SENSOR ODOMETRY & IMU)

Tujuan algoritma ini adalah untuk menghitung posisi koordinat $X$ dan $Y$ robot di lapangan, serta sudut hadap robot (*Heading*) secara *real-time* dan kontinu tanpa jeda.

### A. Pengukuran Rotasi Roda (4 Encoder Internal Mecanum via Interrupt)
1. **4 Roda Mecanum & 4 Encoder**: Robot menggunakan 4 roda mecanum yang masing-masing digerakkan oleh motor dengan encoder internal quadrature (Front Left, Front Right, Rear Left, Rear Right).
2. **Fasa A dan Fasa B**: Setiap encoder menghasilkan dua sinyal gelombang kotak (Fasa A dan B) untuk mendeteksi jumlah putaran dan arah.
3. **Interrupt Tepi Naik (Rising Edge)**: Program mendengarkan tepi naik pin Fasa A menggunakan fungsi interupsi `attachInterrupt()` untuk masing-masing dari ke-4 encoder.
4. **Logika Pendeteksi Arah yang Efisien**: Saat terjadi interupsi pada roda $i$:
   * Jika Pin Fasa B bernilai `HIGH`, arah putaran adalah maju $\rightarrow$ nilai pencacah (*ticks*) bertambah $1$.
   * Jika Pin Fasa B bernilai `LOW`, arah putaran adalah mundur $\rightarrow$ nilai pencacah (*ticks*) berkurang $1$.
5. **Atomic Read**: Pembacaan ticks encoder pada main loop dilakukan dengan mematikan interupsi sejenak (`noInterrupts()`) lalu menyalakannya kembali (`interrupts()`) untuk mencegah terjadinya *race condition* data 32-bit.

### B. Forward Kinematics 4 Roda Mecanum (Lokalisasi Koordinat)
Untuk menghitung perpindahan lokal robot ($\Delta X_{local}$, $\Delta Y_{local}$) berdasarkan perubahan putaran roda ($\Delta Ticks$) dari 4 encoder internal:
1. **Konversi Ticks ke Jarak Linear**:
   Perubahan jarak linear untuk masing-masing roda ($\Delta S_i$ dalam mm) dihitung dari selisih ticks saat ini dengan iterasi sebelumnya ($\Delta Ticks_i$):
   $$\Delta S_i = \frac{\Delta Ticks_i}{\text{TPR}} \times 2\pi R$$
   *Di mana $\text{TPR}$ adalah Ticks Per Revolution dari encoder internal, dan $R$ adalah radius roda mecanum dalam mm.*

2. **Rumus Forward Kinematics**:
   Perpindahan lokal robot dalam satu iterasi super-loop dihitung dari kombinasi linear perpindahan ke-4 roda:
   $$\Delta X_{local} = \frac{1}{4} (\Delta S_{FL} + \Delta S_{FR} + \Delta S_{RL} + \Delta S_{RR})$$
   $$\Delta Y_{local} = \frac{1}{4} (-\Delta S_{FL} + \Delta S_{FR} + \Delta S_{RL} - \Delta S_{RR})$$

### C. Orientasi Hadap (MPU6050 via I2C Fast Mode)
1. MPU6050 membaca nilai percepatan sudut (*gyroscope*) pada sumbu Z secara berkala via bus I2C pada kecepatan 400kHz (Fast Mode).
2. **Kalkulasi Integration Non-Blocking**:
   Sudut hadap absolut robot (*Yaw* / $\theta$ dalam radian) diperoleh dengan mengintegrasikan kecepatan sudut terhadap waktu aktual ($\Delta t_{imu}$):
   $$\theta = \theta + (\text{GyroZ} \times \Delta t_{imu})$$
   *Di mana $\Delta t_{imu}$ adalah selisih waktu presisi berbasis `micros()` dari pembacaan sensor IMU terakhir.*

### D. Transformasi Koordinat Lapangan (Matriks Rotasi 2D)
Agar perpindahan lokal robot ($\Delta X_{local}$, $\Delta Y_{local}$) dapat dipetakan terhadap koordinat dunia/lapangan global, kita menerapkan matriks rotasi menggunakan sudut orientasi hadap robot saat ini ($\theta$):
$$\Delta X_{global} = \Delta X_{local} \cos(\theta) - \Delta Y_{local} \sin(\theta)$$
$$\Delta Y_{global} = \Delta X_{local} \sin(\theta) + \Delta Y_{local} \cos(\theta)$$

Hasil perpindahan global ditambahkan secara akumulatif ke koordinat posisi global robot:
$$\text{odom\_x\_mm} = \text{odom\_x\_mm} + \Delta X_{global}$$
$$\text{odom\_y\_mm} = \text{odom\_y\_mm} + \Delta Y_{global}$$
$$\text{odom\_yaw\_rad} = \theta$$

---

## 3. DETAIL ALGORITMA KENDALI GERAK & NAVIGASI (KINEMATICS & PID)

Tujuan algoritma ini adalah menggerakkan robot dari posisi aktual saat ini menuju ke posisi target koordinat lapangan yang diinginkan secara halus, presisi, dan modular.

### Alur Kerja Modular dari Target Koordinat ke PWM Motor

```text
       TARGET KOORDINAT                   POSISI AKTUAL ROBOT
    (X_tgt, Y_tgt, Yaw_tgt)        (odom_x_mm, odom_y_mm, odom_yaw_rad)
               |                                     |
               +------------------+------------------+
                                  |
                                  v
                         [ Hitung Selisih ]
                      (Err_X, Err_Y, Err_Yaw)
                                  |
                                  v
                        [ Matriks Rotasi 2D ]
                    (Rotasikan Error ke Frame Robot)
                                  |
                                  v
                        [ PID Navigasi Posisi ]
                      (Vx, Vy, Omega Target Sasis)
                                  |
                                  v
                       [ INVERSE KINEMATICS ]
             (Mengubah Target Sasis ke RPM untuk tiap Roda)
                                  |
            +---------------+-----+-----+---------------+
            |               |           |               |
            v               v           v               v
       Target RPM       Target RPM  Target RPM      Target RPM
        Roda FL          Roda FR     Roda RL         Roda RR
            |               |           |               |
            +---------------+-----+-----+---------------+
                                  |
                                  v
                   [ PID KECEPATAN (RPM) PER RODA ]
              (Membandingkan Target RPM vs Aktual RPM)
                                  |
                                  v
                      Output PWM (0 s.d 255)
                                  |
                                  v
                   [ FUNGSI PENGGERAK MOTOR FISIK ]
                          analogWrite()
```

### A. Algoritma Inverse Kinematics Mecanum
Mengubah kecepatan translasi target sasis ($V_x$, $V_y$, $\omega$) menjadi target kecepatan putar sudut (rad/s) untuk individu ke-4 roda mecanum berdasarkan letak sumbu $X$ ($L_x$) dan $Y$ ($L_y$) serta jari-jari roda ($R$):
*   $$\omega_{FL} = \frac{1}{R} (V_x - V_y - (L_x + L_y)\omega)$$
*   $$\omega_{FR} = \frac{1}{R} (V_x + V_y + (L_x + L_y)\omega)$$
*   $$\omega_{RL} = \frac{1}{R} (V_x + V_y - (L_x + L_y)\omega)$$
*   $$\omega_{RR} = \frac{1}{R} (V_x - V_y + (L_x + L_y)\omega)$$
Lalu kecepatan sudut dikonversi ke Target RPM: $\text{RPM} = \frac{\omega \times 60}{2\pi}$.

### B. Algoritma PID (Proportional-Integral-Derivative)
Sebuah fungsi `computePID()` yang *reusable* diterapkan ganda: Pertama, untuk merubah error jarak (Navigasi Posisi) menjadi target kecepatan sasis. Kedua, merubah error RPM roda (Kecepatan Inner-Loop) menjadi nilai kendali tegangan PWM.
1. **$P$ (Proportional)**: Memberikan respon proporsional dari error saat ini.
2. **$I$ (Integral)**: Mengakumulasi total error terhadap waktu ($\int error \cdot dt$) untuk menghilangkan *steady-state error* (robot berhenti padahal belum di titik nol). Menggunakan proteksi *anti-windup* agar tidak meluap tak terkendali.
3. **$D$ (Derivative)**: Menghitung laju perubahan error ($\frac{\Delta error}{\Delta t}$) untuk melakukan "pengereman" agar robot tidak kebablasan (*overshoot*).

---

## 4. SISTEM KEAMANAN (FAILSAFE WATCHDOG)

Untuk mencegah kecelakaan fatal saat robot melaju liar karena terputusnya sinyal joystick nirkabel atau koneksi UART:
1. **Pembaruan Detak Jantung (*Heartbeat*)**: Setiap kali Slave 1 berhasil menerjemahkan paket serial UART biner dari Master, variabel `last_packet_ms` di-update dengan nilai `millis()`.
2. **Pendeteksi Waktu Kritis**: Di dalam super-loop, program secara aktif menghitung selisih waktu = `millis() - last_packet_ms`.
3. **Aksi Darurat**: Jika paket tidak kunjung datang dan selisih melebihi toleransi kritis **500 milidetik**, maka sistem *failsafe* secara otomatis dieksekusi: Seluruh variabel integral PID di-reset, dan sinyal tegangan motor diturunkan ke `0` (Rem mendadak).

---

## 5. DETAIL ALGORITMA AUTO-TUNER PID (STATE-MACHINE HEURISTIK)

Algoritma untuk mengotomatisasi pencarian parameter $K_p$, $K_i$, dan $K_d$ terbaik untuk keempat motor roda tanpa menghentikan sistem (*blocking*). Proses dipicu via *Serial CLI* dengan command `autotune`.

### A. Pengumpulan Metrik (*Welford's Online Algorithm*)
Saat menguji satu motor pada target kecepatan tertentu (selama misal 5000 ms), algoritma terus mencatat data aktual RPM untuk mendapatkan metrik performa:
1. **Rata-rata Error Absolut** terhadap target.
2. **Overshoot Maksimal** (lonjakan puncak awal kecepatan dalam persen terhadap target).
3. **Rise Time** (waktu dalam milidetik dari 10% hingga mencapai 90% kecepatan).
4. **Variansi Kestabilan** (menggunakan *Welford's Method* untuk mengetahui seberapa halus kecepatan di fasa konstan).

### B. Penghitungan Skor & Penyesuaian (Heuristik)
Pada akhir satu siklus 5000 ms, seluruh metrik dikalkulasikan ke dalam satu nilai "Skor Penalti" (semakin rendah = semakin baik).
Sistem menggunakan *Rules-Based Heuristic* (sama dengan *Fuzzy Logic* sederhana) untuk menyesuaikan parameter untuk pengujian di siklus berikutnya:
*   Jika **Overshoot Terlalu Tinggi** ($>15\%$): Turunkan nilai $K_p$ cukup ekstrem, dan perlahan naikkan $K_d$ sebagai peredam kejut.
*   Jika **Rise Time Sangat Lambat** ($>3000$ ms): Naikkan drastis nilai $K_p$ dan $K_i$.
*   Jika **Error Rata-rata Masih Besar**: Tambahkan akumulasi dorongan pada $K_i$.

Hal ini dilakukan sebanyak 15 siklus per roda untuk menghasilkan nilai yang sangat optimal tanpa campur tangan operator manusia.
