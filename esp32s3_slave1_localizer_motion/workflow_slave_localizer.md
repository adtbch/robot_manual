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
                 (Target RPM: FL, FR, RL, RR)
                                  |
                                  v
                            [ pidRpm() ]
                                  |
            +---------------------+---------------------+
            |                     |                     |
            v                     v                     v
      [ computePID(0) ]     [ computePID(1) ]     [ ... Roda 2 & 3 ]
      (PID Roda FL)         (PID Roda FR)         (PID Roda RL & RR)
            |                     |                     |
            +---------------------+---------------------+
                                  |
                                  v
                          [ setMotorSpeed() ]
                       (PWM ke Driver BTS7960)
```

### A. Algoritma PID Navigasi Posisi (Outer-Loop)
1. **Hitung Error Jarak**: Jarak target dikurangi posisi aktual saat ini.
2. **Transformasikan Error ke Frame Robot**: Agar robot tahu harus bergerak maju/mundur ($V_x$) atau menyamping ($V_y$) dari sudut pandangnya sendiri, error global diputar menggunakan sudut Yaw robot saat ini ($\theta$).
3. **Komputasi PID**: Error lokal diumpankan ke rumus PID Navigasi untuk menghasilkan target kecepatan linier sasis ($V_x$, $V_y$, $\omega$).
4. **Deadband Filter**: Jika posisi robot sudah berada di bawah toleransi (misal selisih kurang dari 5mm), paksa output kecepatan ke 0 agar robot tidak bergetar mencari-cari titik presisi secara berlebihan.

### B. Algoritma Inverse Kinematics Mecanum (4 Roda)
Mengubah target kecepatan sasis ($V_x$, $V_y$, $\omega$) menjadi target kecepatan putar individu dari 4 roda Mecanum (Target RPM) berdasarkan geometri sasis (jarak dari titik pusat robot ke roda arah X ($L_x$) dan arah Y ($L_y$)), serta radius roda ($R$):
$$\text{RPM}_{FL} = \frac{60}{2\pi R} (V_x - V_y - (L_x + L_y)\omega)$$
$$\text{RPM}_{FR} = \frac{60}{2\pi R} (V_x + V_y + (L_x + L_y)\omega)$$
$$\text{RPM}_{RL} = \frac{60}{2\pi R} (V_x + V_y - (L_x + L_y)\omega)$$
$$\text{RPM}_{RR} = \frac{60}{2\pi R} (V_x - V_y + (L_x + L_y)\omega)$$

### C. Desain Fungsi Modular (Single Responsibility Principle)
Sistem ini diprogram secara modular agar kode mudah dipelihara (*maintainable*), mudah dibaca, dan mudah di-debug secara terpisah.

#### 1. Pembacaan Sensor Global (`bacaSensor()`)
Membaca semua data sensor secara asinkron tanpa delay.
```cpp
void bacaSensor() {
  // 1. Baca paket perintah radio/UART dari Master (jika ada)
  bacaRadioUART();
  
  // 2. Baca kecepatan sudut sumbu Z dari IMU MPU6050
  bacaGyroZ();
  
  // 3. Baca encoder internal 4 motor secara aman (atomic)
  ambilTicksEncoder();
}
```

#### 2. Pembaruan Lokalisasi (`updateOdometri()`)
Melakukan perhitungan forward kinematics dan update koordinat global robot setiap saat.
```cpp
void updateOdometri() {
  // Hitung delta ticks untuk 4 encoder
  // Terapkan Forward Kinematics & Matriks Rotasi 2D
  // Perbarui variabel global odom_x_mm, odom_y_mm, odom_yaw_rad
}
```

#### 3. Hitung Inverse Kinematics (`hitungInverseKinematics()`)
Mengonversi kecepatan target sasis menjadi target RPM roda mecanum.
```cpp
void hitungInverseKinematics(float vx, float vy, float omega, float &rpmFL, float &rpmFR, float &rpmRL, float &rpmRR) {
  float L_sum = Lx + Ly;
  float multiplier = 60.0 / (2.0 * PI * R_wheel);
  
  rpmFL = multiplier * (vx - vy - (L_sum * omega));
  rpmFR = multiplier * (vx + vy + (L_sum * omega));
  rpmRL = multiplier * (vx + vy - (L_sum * omega));
  rpmRR = multiplier * (vx - vy + (L_sum * omega));
  
  // Batasi RPM target agar tidak melebihi kapasitas maksimum motor
  rpmFL = constrain(rpmFL, -maxrpm, maxrpm);
  rpmFR = constrain(rpmFR, -maxrpm, maxrpm);
  rpmRL = constrain(rpmRL, -maxrpm, maxrpm);
  rpmRR = constrain(rpmRR, -maxrpm, maxrpm);
}
```

#### 4. Inti Kontroler PID RPM Modular (`pidRpm()`)
Fungsi ini bertugas mengontrol RPM masing-masing dari ke-4 roda. Fungsi ini memanggil fungsi komputasi PID umum (`computePID`) untuk setiap roda secara modular.
```cpp
void pidRpm(float targetFL, float targetFR, float targetRL, float targetRR) {
  // Ambil parameter PID dari preferensi memori EEPROM/Preferences
  double kp = pidConfig.kp;
  double ki = pidConfig.ki;
  double kd = pidConfig.kd;
  
  double minInt = -2000.0;
  double maxInt = 2000.0;

  // 1. Dapatkan RPM aktual saat ini dari ke-4 encoder
  float actFL = dapatkanRpmAktual(0);
  float actFR = dapatkanRpmAktual(1);
  float actRL = dapatkanRpmAktual(2);
  float actRR = dapatkanRpmAktual(3);

  // 2. Komputasi PID untuk masing-masing roda secara modular
  double pwmFL = computePID(0, targetFL, actFL, kp, ki, kd, minInt, maxInt);
  double pwmFR = computePID(1, targetFR, actFR, kp, ki, kd, minInt, maxInt);
  double pwmRL = computePID(2, targetRL, actRL, kp, ki, kd, minInt, maxInt);
  double pwmRR = computePID(3, targetRR, actRR, kp, ki, kd, minInt, maxInt);

  // 3. Validasi arah PWM (konstrain sesuai arah target)
  pwmFL = batasiPwmSesuaiArah(targetFL, pwmFL);
  pwmFR = batasiPwmSesuaiArah(targetFR, pwmFR);
  pwmRL = batasiPwmSesuaiArah(targetRL, pwmRL);
  pwmRR = batasiPwmSesuaiArah(targetRR, pwmRR);

  // 4. Kirim sinyal kontrol ke hardware driver motor
  setMotorSpeed(0, pwmFL); // Motor FL
  setMotorSpeed(1, pwmFR); // Motor FR
  setMotorSpeed(2, pwmRL); // Motor RL
  setMotorSpeed(3, pwmRR); // Motor RR
}
```

#### 5. Fungsi Komputasi PID Umum (`computePID()`)
Rumus PID dengan fitur *Anti-Windup*, *Derivative Filtering*, *Setpoint Weighting (2-DOF)*, dan *Dynamic dt* (seperti template yang telah teruji).
```cpp
double computePID(int index, double setpoint, double input, double Kp, double Ki, double Kd, double Minintegral, double Maxintegral) {
  // Validasi index & nilai input (NaN/Inf)
  // Hitung dt dinamis secara non-blocking
  // Terapkan Setpoint Weighting (2-DOF) untuk mengurangi overshoot
  // Hitung P Term
  // Akumulasikan integral (I Term)
  // Terapkan Derivative on Measurement & Low-pass Filter untuk mencegah spike D-term
  // Hitung total output sebelum saturasi
  // Batasi output (constrain)
  // Terapkan Anti-Windup (Back-Calculation) untuk mencegah integral windup saat saturasi
  // Kembalikan nilai output PWM
}
```

#### 6. Driver Output Motor (`setMotorSpeed()`)
Mengirimkan sinyal duty cycle PWM fisik ke pin driver BTS7960.
```cpp
void setMotorSpeed(int index, float pwm) {
  // Berdasarkan indeks motor (0=FL, 1=FR, 2=RL, 3=RR):
  // Set Pin Arah (Direction pins)
  // Tulis analogWrite() pada pin PWM driver BTS7960
}
```

---

## 4. DETAIL ALGORITMA AUTO-TUNER PID (HEURISTIK NON-BLOCKING)

Algoritma ini bertugas mencari konstanta PID ($K_p$, $K_i$, $K_d$) terbaik untuk motor roda secara otomatis menggunakan metode State-Machine berulang tanpa menghentikan sistem utama.

### Keterangan Penting: Independensi PID Per Motor
Sistem ini menggunakan **PID RPM yang terpisah (independen) untuk masing-masing dari 4 motor**. Tiap motor memiliki karakteristik fisik, gesekan, beban, dan toleransi yang berbeda-beda. Oleh karena itu, Auto-Tuner akan berjalan secara bergantian untuk menyetel (*tuning*) setiap motor secara individual (Front Left -> Front Right -> Rear Left -> Rear Right) dan menyimpan parameter masing-masing ke Non-Volatile Storage (NVS) secara terpisah.

### Logika & Alur State-Machine Auto-Tuner (`autoTuner.ino`)
Di dalam [autoTuner.ino](autoTuner.ino), proses tuning diimplementasikan secara asinkron berbasis **State-Machine Non-Blocking** yang dipanggil secara periodik melalui `autoTunerTick()`. Hal ini menjamin super-loop utama tetap responsif mendeteksi input darurat (failsafe) bahkan saat pengujian motor sedang berjalan.

```text
  [ STATUS_DIAM ] === (Trigger tombol BOOT / CLI) ===> [ STATUS_WAIT_RELEASE ]
                                                               |
                                                               v
                                                      [ AT_MOTOR_INIT ]
                                              (Load init gain dari NVS per motor)
                                                               |
                                                               v
                                                    [ AT_CYCLE_START ] <-------+
                                              (Set PID gain saat ini & reset)  |
                                                               |               | (Ulangi
                                                               v               |  Siklus
                                                     [ AT_CYCLE_RUN ]          |  hingga
                                              (Motor berputar, ambil metrik)   |  12 kali)
                                                               |               |
                                                               v               |
                                                   [ AT_CYCLE_FINISH ]         |
                                             (Hitung Skor & Sesuaikan KpKiKd)  |
                                                               |               |
                                                               +======> [ COOLDOWN ]
                                                               |
                                                  (Selesai 12 Siklus)
                                                               |
                                                               v
                                                      [ AT_MOTOR_SHOW ]
                                             (Tampilkan hasil sementara & save NVS)
                                                               |
                                                               v
                                                   [ LANJUT MOTOR BERIKUTNYA ]
                                                (FL -> FR -> RL -> RR -> DONE)
```

### A. Pengumpulan Metrik Kinerja (Welford's Algorithm)
Selama fase `AT_CYCLE_RUN` (motor diputar dengan target kecepatan konstan `AUTOTUNE_TARGET_VEL` [rad/s] $\approx$ `kTargetRpm` [RPM] selama `AUTOTUNE_RUN_MS` milidetik), sistem mengumpulkan data secara non-blocking setiap interval `kPidTickMs` (40ms):
1. **Error Rata-rata**: Akumulasi selisih RPM target vs RPM aktual secara absolut dibagi jumlah sampel.
2. **Overshoot**: Lonjakan kecepatan tertinggi yang melewati batas target RPM:
   $$\text{Overshoot (\%)} = \frac{\text{Peak RPM} - \text{Target RPM}}{\text{Target RPM}} \times 100\%$$
3. **Rise Time (10% to 90%)**: Waktu yang dibutuhkan motor sejak pertama kali mencapai 10% target RPM hingga mencapai 90% target RPM (dalam milidetik).
4. **Burst Detection**: Deteksi lonjakan kecepatan ekstrem di 500ms pertama (kecepatan melebihi 130% target) untuk mengidentifikasi perilaku tidak stabil (*instability*).
5. **Kestabilan Steady-State (Welford's Online Algorithm)**: Menghitung rata-rata (*mean*) dan variansi fluktuasi secara real-time pada 40% durasi terakhir pengujian (jendela steady-state) untuk mengukur tingkat kekasaran/getaran motor secara presisi.

### B. Algoritma Penilaian (Scoring System)
Setiap akhir siklus pengujian (`AT_CYCLE_FINISH`), performa parameter PID dinilai menggunakan rumus skor penalti (skor lebih rendah $\rightarrow$ performa lebih baik):
* **Penalti Akurasi**: Kuadrat dari rata-rata error absolut dikalikan bobot ($W_{error} = 12.0$).
* **Penalti Steady-State**: Kuadrat dari error rata-rata steady-state dikalikan bobot ($18.0$) ditambah variansi steady-state dikalikan bobot ($4.0$) untuk menghukum osilasi tinggi.
* **Penalti Overshoot**: Menggunakan penalti eksponensial di mana overshoot $>15\%$ mendapat penalti kuadrat yang sangat besar ($W_{overshoot} \times 2.0$), overshoot $8-15\%$ mendapat penalti standar ($W_{overshoot}$), dan overshoot $<3\%$ diberi penghargaan.
* **Penalti Akselerasi**: Jika *Rise Time* lambat ($>3$ detik) atau sedang ($1.5-3$ detik) ditambahkan penalti linear.
* **Sistem Reward (Potongan Skor)**: Jika rise time cepat ($<1.5$ detik), overshoot rendah ($<8\%$), dan rata-rata error kecil ($<2.5$ RPM), skor dikurangi $15.0$ sebagai tanda parameter tersebut optimal.
* **Failsafe Limit Penalti**: Parameter $K_p$, $K_i$, $K_d$ yang keluar dari batas aman di-constrain dan diberikan penalti berat tambahan.

### C. Penyesuaian Parameter Berbasis Heuristik Multi-Tahap (Multi-Stage Precision)
Untuk mencari parameter secara efisien, penyesuaian parameter dibagi menjadi tiga tahap presisi:
1. **Precision::COARSE (Siklus 1-5)**: Penyesuaian agresif berbasis persentase multiplier untuk mendekati rentang parameter yang bekerja.
   * Jika **Overshoot Terlahu Tinggi ($>15\%$)**: Kurangi $K_p$ menjadi $75\%$ dan naikkan $K_d$ menjadi $150\%$.
   * Jika **Akselerasi Terlalu Lambat ($>3$ detik)**: Naikkan $K_p$ menjadi $140\%$ dan $K_i$ menjadi $120\%$.
   * Jika **Overshoot Sedang & Rise Time Cepat**: Naikkan $K_d$ menjadi $130\%$ dan kurangi $K_p$ sedikit ($95\%$).
   * Jika **Terjadi Burst (Sistem Bergetar Hebat)**: Kurangi $K_p$ menjadi $70\%$, kurangi $K_i$ menjadi $80\%$, dan naikkan $K_d$ menjadi $180\%$.
2. **Precision::FINE (Siklus 6-9)**: Penyesuaian bertahap menggunakan langkah tetap (`gKpStep = 1.5`, `gKiStep = 0.6`, `gKdStep = 0.04`).
3. **Precision::ULTRA_FINE (Siklus 10-12)**: Penyesuaian mikro untuk menghaluskan riak kontrol (`gKpStep = 0.5`, `gKiStep = 0.2`, `gKdStep = 0.01`).

Jika dalam tahap tertentu tidak ada perbaikan skor selama 3 siklus berturut-turut (stagnasi), state machine akan memaksa transisi ke tahap presisi berikutnya lebih awal. Parameter dengan skor terbaik yang diperoleh sepanjang 12 siklus disimpan secara otomatis ke preferensi NVS ESP32 per roda.

---

## 5. DETAIL ALGORITMA PENGAMAN (FAILSAFE WATCHDOG)

Untuk mencegah kecelakaan fatal saat robot lepas kendali akibat hilangnya sinyal kontrol dari Master:
1. **Timestamp Detak Jantung**: Setiap kali Slave 1 berhasil mem-parse paket serial UART/Radio biner dari Master, variabel waktu `last_packet_ms` diperbarui dengan waktu CPU saat ini (`millis()`).
2. **Komparator Timeout**: Di dalam loop kontrol motor (tiap loop), sistem membandingkan:
   $$\Delta t_{watchdog} = \text{millis}() - \text{last\_packet\_ms}$$
3. **Aksi Darurat**: Jika $\Delta t_{watchdog}$ melebihi **500 milidetik**, watchdog secara asinkron memicu fungsi penanganan darurat `failsafe_stop()`.
4. **Rem Semua Motor**: Seluruh register PWM motor di-set ke `0`, dan perhitungan integral PID pada ke-4 motor di-reset ke nol untuk menghentikan posisi robot seketika dan menghindari kejut listrik saat tersambung kembali.

---

## 6. ANTARMUKA PENGUJIAN MOTOR & GERAK VIA SERIAL CLI (`setial.ino`)

Untuk memudahkan proses pengujian hardware (*hardware bring-up*) dan pemeliharaan secara modular, program menyediakan antarmuka baris perintah (**Serial CLI**) non-blocking melalui UART utama (`Serial0` USB) pada Baudrate 115200. Antarmuka ini memungkinkan operator menguji motor satu per satu atau secara bersamaan dengan parameter presisi.

### Fitur Utama Antarmuka Serial CLI
1. **Kendali Motor Tunggal (Individu)**: Operator dapat memutar satu motor tertentu saja dengan target PWM atau RPM untuk durasi tertentu, sangat berguna untuk memastikan putaran kabel motor tidak terbalik.
2. **Kendali Sekuensial**: Menguji setiap motor satu per satu secara bergantian untuk memverifikasi posisi roda (FL $\rightarrow$ FR $\rightarrow$ RL $\rightarrow$ RR).
3. **Kendali Kinematik (XYDelta)**: Operator dapat mengirim perintah gerak berbasis kinematik untuk menggerakkan sasis robot ke arah linear X, Y, dan orientasi sudut tertentu.
4. **Trigger Autotuning Terintegrasi**: Menginisiasi pencarian otomatis konstanta PID untuk motor tertentu maupun keseluruhan motor.
5. **Aborsi Darurat (STOP)**: Memberikan perintah penghentian seketika untuk semua jenis gerakan yang sedang berjalan.

### Daftar Perintah Serial CLI yang Tersedia

```text
+------------------------------+-------------------------------------------------------------+
| FORMAT PERINTAH SERIAL       | PENJELASAN FUNGSI & CARA KERJA                             |
+------------------------------+-------------------------------------------------------------+
| STOP                         | Menghentikan seketika sequence, gerakan motor, atau         |
|                              | proses auto-tuning yang sedang berjalan (Emergency Stop).    |
+------------------------------+-------------------------------------------------------------+
| AUTOTUNE RPM <idx>           | Menjalankan Auto-Tuner PID khusus untuk motor indeks <idx>  |
|                              | (0: Front Left, 1: Front Right, 2: Rear Left, 3: Rear Right)|
+------------------------------+-------------------------------------------------------------+
| AUTOTUNE RPM ALL             | Menjalankan Auto-Tuner PID sekuensial untuk semua motor     |
+------------------------------+-------------------------------------------------------------+
| MOVE RPM <idx> <rpm> <ms>    | Memutar motor <idx> dengan kecepatan target <rpm> selama    |
|                              | durasi waktu <ms> milidetik menggunakan feedback PID.       |
+------------------------------+-------------------------------------------------------------+
| MOVE PWM <idx> <pwm> <ms>    | Memutar motor <idx> secara open-loop dengan duty cycle PWM   |
|                              | <pwm> [-1023 s.d 1023] selama durasi waktu <ms> milidetik.  |
+------------------------------+-------------------------------------------------------------+
| SEQ RPM <rpm> <ms>           | Memutar roda satu-persatu (FL -> FR -> RL -> RR)            |
|                              | dengan target <rpm> selama <ms> milidetik per motor.        |
+------------------------------+-------------------------------------------------------------+
| SEQ PWM <pwm> <ms>           | Memutar roda satu-persatu (FL -> FR -> RL -> RR)            |
|                              | dengan duty-cycle <pwm> selama <ms> milidetik per motor.     |
+------------------------------+-------------------------------------------------------------+
| MOVE KINEMATIC <x> <y> <w>   | Menggerakkan sasis robot berdasarkan koordinat kinematik   |
|                              | target kecepatan X, Y, dan kecepatan sudut rotasi W.        |
+------------------------------+-------------------------------------------------------------+
```
*(Catatan: CLI bersifat case-insensitive, perintah otomatis dikonversi ke UPPERCASE).*
