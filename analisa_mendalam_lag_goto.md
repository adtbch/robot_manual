# Analisis Mendalam Penyebab Lag, Gagap (Stuttering), dan Freeze pada GOTO (Slave1)

Dokumen ini menyajikan analisis teknis mendalam mengenai perbedaan performa antara perintah `KN` (lancar dan responsif) dan `GOTO` (lag, tersendat, hingga menyebabkan mikrokontroler freeze/hang).

---

## 1. Perbandingan Arsitektur Eksekusi: GOTO vs KN

| Parameter | Perintah KN (Kinematika Kecepatan) | Perintah GOTO (Target Posisi/Waypoint) |
|---|---|---|
| **Frekuensi Input** | 20 ms (langsung dari Master) | 20 ms (langsung dari Master) |
| **Variabel Target** | Kecepatan lokal ($v_x, v_y, \omega$) | Koordinat absolut ($x, y$ cm, $\theta$ deg) |
| **Kontroler di Slave1** | Kinematika roda langsung (`driveFieldCentricWithYawCorrection`) | Kontroler P Posisi + MPU6050 Yaw PID (`waypointTick`) |
| **Status Internal** | Tanpa status (Stateless) | State Machine: `IDLE`, `RUNNING`, `REACHED` |
| **Output Motor** | Diperbarui kontinu setiap loop | Dihentikan paksa (`rpmMotor(0,0,0,0)`) saat `REACHED` |
| **Feedback UART** | Tidak ada | Mengirim teks `"WP: REACHED"` ke Master |
| **Debug USB Serial** | Tidak ada | Mencetak log teks panjang ke USB PC |

---

## 2. Akar Masalah Teknis Utama

Ada 4 akar masalah mengapa mode `GOTO` menyebabkan lag, gerakan tersendat (ngegas-ngerem), dan freeze:

### A. Konflik Toleransi Posisi vs Ukuran Step (Step Size vs Tolerance Discrepancy)
*   **Data Lapangan**:
    *   Ukuran step pergerakan manual Master (`GOTO_STEP_CM`) = **$0.85\text{ cm}$** per tick (20 ms).
    *   Toleransi posisi default Slave1 (`wpTolPos_m`) = **$0.05\text{ meter}$ ($5.0\text{ cm}$)**.
*   **Mekanisme Masalah**:
    1.  Master mengirim target baru: `goto 1 0 0` ($1\text{ cm}$).
    2.  Slave1 mengecek apakah target berada dalam toleransi:
        $$\text{Jarak} = |1\text{ cm} - 0\text{ cm}| = 1\text{ cm} < 5\text{ cm} \text{ (Toleransi)}$$
    3.  Karena berada dalam toleransi, Slave1 mengeksekusi blok kode penanganan target tercapai:
        ```cpp
        wpState = WaypointState::REACHED;
        wpNotifyReachedToMaster();
        rpmMotor(0, 0, 0, 0); // Rem aktif
        Serial.printf("[WP] Already at target..."); // Debug print
        ```
    4.  Robot **berhenti total**, tidak bergerak, lalu mengirim pesan reached.
    5.  Master menerima status, lalu menambah target ke `goto 2 0 0` ($2.55\text{ cm} \approx 3\text{ cm}$).
    6.  Jarak masih $3\text{ cm} < 5\text{ cm}$. Slave1 kembali mengerem motor, mengirim pesan, dan mencetak debug.
    7.  Proses ini berulang sampai target melompat di atas $5\text{ cm}$ (misal ke $6\text{ cm}$).
*   **Akibat**: Robot tidak mau bergerak saat awal stick didorong, melainkan masuk ke dalam loop "pengereman aktif" dan membanjiri buffer serial dengan log error.

### B. Pemblokiran Serial USB (115200 Baud) pada Loop Cepat
*   **Data Lapangan**:
    *   Baud rate USB Serial ke PC: **$115200\text{ bps}$** (ditetapkan di `Slave1motion.ino`: `Serial.begin(115200)`).
    *   Panjang pesan debug per tick: `"[WP] Already at target pos=(0.000,0.000)m yaw=0.0deg\n"` (sekitar 58 karakter).
*   **Perhitungan Transmisi**:
    *   Waktu kirim string 58 karakter pada $115200\text{ baud}$:
        $$T_{tx} = \frac{58 \times 10 \text{ bit/char}}{115200 \text{ bps}} \approx 5.03\text{ ms}$$
    *   Jika Master mengirim perintah `goto` setiap 20 ms, dan Slave1 mencetak log ini berulang kali dalam loop cepat (karena terperangkap di kondisi toleransi di atas), buffer TX ring-buffer milik USB hardware ESP32 ($64\text{ byte}$ hingga $256\text{ byte}$) akan **penuh dalam sekejap**.
    *   Ketika buffer penuh, fungsi bawaan Arduino `Serial.printf()` berubah menjadi fungsi **blocking** (sinkron/busy-wait) hingga ada ruang di buffer untuk menulis data.
*   **Akibat**: Loop utama Slave1 melambat drastis dari sub-milidetik menjadi $>10\text{ ms}$ per iterasi. Ini merusak kalkulasi waktu delta integral/derivatif PID motor, mengacaukan pembacaan data sensor gyro MPU6050 DMP, dan menyebabkan mikrokontroler tampak "freeze/stuck" (tidak responsif).

### C. Efek "Braking & Jumping" akibat State Lockout
*   **Mekanisme Masalah**:
    1.  Ketika koordinat target akhirnya melebihi batas toleransi $5\text{ cm}$ (misalnya target = $6\text{ cm}$), Slave1 mengubah status menjadi `RUNNING`.
    2.  Begitu status menjadi `RUNNING`, baris kode berikut aktif di `serial.ino`:
        ```cpp
        if (getWaypointState() != WaypointState::RUNNING) {
            startWaypoint(x_cm, y_cm, yaw, speed);
        }
        ```
    3.  Ini berarti Slave1 mengunci target pada $6\text{ cm}$ dan **mengabaikan semua pembaruan koordinat berikutnya** dari Master (misalnya target `7 cm`, `8 cm`, `9 cm` yang dikirim per 20 ms diabaikan).
    4.  Robot meluncur dengan PID menuju $6\text{ cm}$. Ketika robot mencapai koordinat $1.1\text{ cm}$ (yang berjarak $4.9\text{ cm}$ dari target $6\text{ cm}$, yaitu masuk toleransi $5\text{ cm}$), Slave1 langsung:
        *   Mengeset `wpState = WaypointState::REACHED`.
        *   Memanggil `rpmMotor(0,0,0,0)` (rem mendadak).
        *   Mengirim feedback `"WP: REACHED"` ke Master.
    5.  Karena status sekarang sudah `REACHED` (bukan `RUNNING`), Slave1 akhirnya menerima perintah `goto` berikutnya dari Master yang saat itu mungkin sudah menumpuk di koordinat $15\text{ cm}$.
    6.  Robot mendeteksi target baru $15\text{ cm}$ ($>5\text{ cm}$ toleransi), sehingga status kembali menjadi `RUNNING`. Robot kembali ngegas penuh menuju $15\text{ cm}$.
*   **Akibat**: Robot bergerak tersendat-sendat secara ekstrem (ngegas $\rightarrow$ ngerem mendadak $\rightarrow$ diam sejenak $\rightarrow$ loncat target $\rightarrow$ ngegas lagi). Gerakan ini sangat tidak mulus dan merusak kestabilan mekanik robot.

### D. Overhead Parsing Teks Berulang (`strtok` & `atof`)
*   Meskipun UART berjalan pada baudrate tinggi ($921600\text{ bps}$), parsing teks ASCII `"goto <x> <y> <yaw>"` menggunakan fungsi library C standard seperti `strtok()` dan `atof()` memerlukan alokasi memori lokal, pencarian karakter delimiter, dan komputasi floating point string parsing yang memakan waktu CPU berharga.
*   Dibandingkan dengan `KN` biner (yang tinggal memetakan offset memori pointer langsung ke struct biner), pemrosesan teks membebani CPU ESP32 di thread utama loop pergerakan.

### E. Freeze Akibat USB Serial/UART Blocking (Seperti `delay()` Tersembunyi)

**Masalah ini adalah yang paling berbahaya karena menyebabkan Slave1 benar-benar freeze (hang), bukan hanya lag/stuttering.** Ini terjadi karena fungsi `wpNotifyReachedToMaster()` dan `Serial.printf()` di `waypointTick()` / `startWaypoint()` memicu blocking I/O yang tidak terlihat.

#### E1. Hubungan Tol-Print-Freeze

Dengan kode OLD (atau bahkan setelah fix #1/#2a jika toleransi masih 5cm):

1.  **Loop Tak Berujung**:
    *   Target Master = 0.85cm, posisi robot ≈ 0cm.
    *   Jarak = 0.85cm < toleransi 5cm.
    *   `startWaypoint()` (atau `waypointTick()`) mendeteksi "target tercapai".
    *   Memanggil: `Serial.printf("[WP] Already at target...")` (USB).
    *   Memanggil: `wpNotifyReachedToMaster()` → `Serial1.println("WP: REACHED")` (UART1 ke Master).
    *   Memanggil: `rpmMotor(0,0,0,0)` → motor berhenti.
2.  **Iterasi Berikutnya** (20ms kemudian):
    *   Master kirim target baru 1.70cm (masih < 5cm).
    *   Hal yang sama terjadi lagi.
3.  **Terus Berulang** sampai Master mencapai target > 5cm (≈ setelah 6-7 step = 120-140ms).

Selama 120-140ms itu, **USB Serial dan UART1 terus-menerus kebanjiran data**.

#### E2. USB Serial `printf` Blocking → Freeze Total

*   **Kondisi A - USB terhubung ke PC**:
    *   Buffer TX USB hardware = 256 byte (ukuran default ESP32-S3 CDC).
    *   String `[WP] Already at target...` ≈ 58 byte.
    *   Jika loop mencetak string ini setiap iterasi (setiap ~1-5ms), buffer penuh dalam **< 25ms**.
    *   Saat buffer penuh, `Serial.printf()` menjadi **blocking/busy-wait** menunggu PC membaca data.
    *   Jika PC Serial Monitor lambat membaca, CPU Slave1 terhenti (freeze) selama puluhan ms hingga buffer kosong.
    *   Efeknya = seperti `delay(10..50)` yang tidak terduga di tengah loop PID motor dan MPU.

*   **Kondisi B - USB TIDAK terhubung (operasi baterai)**:
    *   **INILAH FREEZE PALING BERBAHAYA.**
    *   Ketika USB tidak terhubung, driver CDC ESP32 di beberapa versi Core tidak pernah mengosongkan buffer TX.
    *   `Serial.printf()` tetap menulis ke buffer. Setelah buffer penuh (256 byte / ≈ 4 panggilan print), `printf()` memanggil fungsi internal driver yang menunggu buffer dikirim ke host **yang tidak ada**.
    *   Akibatnya: `Serial.printf()` **blocking SELAMANYA** (infinite loop / hang).
    *   Robot berhenti total. Tidak ada watchdog yang bisa menyelamatkan.
    *   **Ini persis seperti `delay(INFINITY)`** — tanpa ada `delay()` di kode sumber.

#### E3. UART1 Backpressure (Master Tidak Membaca Cukup Cepat)

*   `wpNotifyReachedToMaster()` memanggil `Serial1.println("WP: REACHED")` ke UART1 (921600 baud).
*   UART1 memiliki hardware FIFO TX 128 byte + software buffer tambahan.
*   Jika Master sibuk (loop berat: ESP-NOW, motor PID, gripper, dll), Master mungkin tidak sempat membaca UART1 RX buffer.
*   Ketika buffer TX Slave1 penuh, `Serial1.println()` blocking.
*   Karena `wpNotifyReachedToMaster()` dipanggil di dalam `waypointTick()` (loop utama Slave1), blocking ini menghentikan SEMUA fungsi Slave1: PID motor, update MPU, pembacaan encoder.
*   Robot tidak bisa bergerak (wheel velocity tidak di-update) selama blocking berlangsung.

#### E4. Mengapa KN Tidak Pernah Freeze?

Perbandingan kritis:

| Aspek | KN Path | GOTO Path (OLD) |
|---|---|---|
| `Serial.printf()` ke USB | **Tidak pernah** | `[WP] Already at target...` (berulang) |
| `Serial.println()` ke Master | **Tidak pernah** | `"WP: REACHED"` (berulang) |
| `rpmMotor(0,0,0,0)` | Tidak pernah | Setiap kali target dalam toleransi |

Kesimpulan: **GOTO path memiliki 3 panggilan blocking I/O yang tidak dimiliki KN path.** Kombinasi ketiganya (USB print + UART print + rem mendadak) menciptakan kondisi freeze sempurna.

#### E5. Verifikasi Eksperimental (Cara Deteksi)

Untuk membuktikan freeze disebabkan oleh USB Serial blocking:

1.  **Uji dengan USB terputus**: Jalankan Slave1 tanpa kabel USB (hanya power baterai). Kirim perintah GOTO dari Master. Jika robot freeze total (tidak bergerak, LED indikator mati/hidup statis), maka USB serial adalah penyebabnya.
2.  **Uji dengan baud rate USB tinggi**: Ubah `Serial.begin(115200)` menjadi `Serial.begin(921600)` di `Slave1motion.ino`. Jika freeze berkurang atau hilang, indikasi kuat bahwa blocking USB adalah akar masalah.
3.  **Hapus sementara semua `Serial.printf`**: Comment baris `Serial.printf("[WP] ...")` di `waypoint.ino` dan `startWaypoint.ino`. Jika freeze hilang, maka sudah terkonfirmasi.

---

## 3. Strategi Perbaikan (Checkpoint Implementasi)

Untuk membuat pergerakan `GOTO` sama responsif dan mulusnya dengan `KN`, kita harus melakukan restrukturisasi total:

1.  **Hapus Log Debug USB Serial dari Loop Utama**:
    *   Hapus `Serial.printf("[WP] Already at target...")` dan `Serial.printf("[WP] Reached! ...")` yang dijalankan berulang kali dalam loop pergerakan. Log debug hanya boleh dicetak sekali saat transisi state terjadi, atau dinonaktifkan sepenuhnya.
2.  **Sinkronisasi Berbasis Status Biner (Binary Handshake)**:
    *   Master tidak boleh meng-increment target secara buta jika Slave1 belum siap menerima target berikutnya.
    *   Slave1 mengirimkan pembaruan status `wpState` secara berkala atau per kejadian menggunakan paket biner `0xAA 0xCC` yang ringan (4 byte), bebas overhead teks.
    *   Master meng-increment target `gTargetX/Y_cm` hanya ketika Slave1 mengirim status `REACHED` atau `IDLE`.
3.  **Hilangkan Pengereman Paksa pada Koordinat Menengah**:
    *   Ketika menggunakan pergerakan manual increment (joystick), `waypointTick()` tidak boleh memanggil `rpmMotor(0,0,0,0)` jika mendeteksi target tercapai, kecuali jika input joystick dari user telah dilepas (kecepatan = 0).
    *   Target harus diperbarui secara kontinu (`applyWaypointTarget` terus-menerus tanpa memedulikan status `RUNNING`) agar kontroler posisi dapat mengikutinya secara mulus (seperti melacak lintasan/trajectory tracking), alih-alih menguncinya hingga sampai baru di-update.
