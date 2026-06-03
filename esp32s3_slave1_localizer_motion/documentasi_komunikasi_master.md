# DOKUMENTASI KOMUNIKASI SERIAL (MASTER <-> SLAVE)
## ESP32-S3 SLAVE 1: MOTION & LOKALISASI

Dokumen ini menjelaskan protokol komunikasi data antar mikrokontroler dari **ESP32 Master** ke **ESP32-S3 Slave 1 (Localizer & Motion)**.

---

## 1. SPESIFIKASI KONEKSI HARDWARE
Komunikasi berjalan menggunakan jalur **Serial1** (UART1) dengan pengaturan non-blocking.
* **Baud Rate:** `921600 bps`
* **Format Data:** `8 Data bits, No Parity, 1 Stop bit (SERIAL_8N1)`
* **Pin RX Slave (Serial 1):** `GPIO 21` (Terhubung ke TX Master)
* **Pin TX Slave (Serial 1):** `GPIO 20` (Terhubung ke RX Master)

---

## 2. PROTOKOL PENGIRIMAN DATA KINEMATIK (Dari Master ke Slave)

Master dapat mengirimkan data pergerakan kinematik sasis Mecanum melalui pengiriman format string teks ASCII murni. Pesan dipisahkan oleh spasi (Space) dan wajib diakhiri dengan karakter garis baru (Newline/`\n`).

### A. Format Dasar (Tanpa Durasi)
Format pengiriman yang paling umum digunakan saat Master mengirim data pergerakan secara kontinu (misal: data dari joystick).

**Struktur Pesan:**
`<Vx> <Vy> <W>\n`

**Parameter:**
1. `<Vx>`: Kecepatan linear ke arah depan/belakang (X-axis). Nilai positif untuk maju, negatif untuk mundur.
2. `<Vy>`: Kecepatan linear geser ke arah kanan/kiri (Y-axis).
3. `<W>` : Kecepatan putar/rotasi sudut sasis (Yaw/Z-axis). Nilai positif untuk putar kanan, negatif untuk putar kiri.

**Perilaku Sistem (Safety Timeout):**
Jika durasi tidak diberikan, ESP32 Slave akan otomatis memberikan durasi pergerakan *default* selama **2000 ms (2 detik)**. Jika dalam 2 detik tidak ada paket serial tambahan dari Master, robot akan otomatis menghentikan semua motor untuk menghindari robot berjalan liar akibat hilangnya koneksi. 

Oleh karena itu, Master harus mengirimkan paket pembaruan (termasuk `0 0 0\n` untuk berhenti) minimal kurang dari 2 detik.

**Contoh:**
```text
100 0 0\n       (Maju dengan kecepatan 100)
0 -100 0\n      (Geser ke kiri dengan kecepatan 100)
100 50 20\n     (Bergerak diagonal sambil memutar sasis)
0 0 0\n         (Berhenti)
```

### B. Format Spesifik (Dengan Durasi Kustom)
Digunakan jika Master ingin mendelegasikan pergerakan presisi selama waktu tertentu tanpa perlu terus-menerus mengirim data.

**Struktur Pesan:**
`<Vx> <Vy> <W> <Durasi>\n`

**Parameter:**
4. `<Durasi>`: Waktu pergerakan dalam satuan milidetik (ms).

**Contoh:**
```text
100 0 0 5000\n  (Maju dengan kecepatan 100 selama persis 5 detik, lalu berhenti otomatis)
```

---

## 3. MEKANISME PARSING DI SISI SLAVE

Di dalam file `serial_commands.ino`, parser berjalan secara *non-blocking* menggunakan *State-Machine*:
1. `Serial1.available()` terus dipantau pada fungsi `serialCommandsTick()`.
2. Fungsi `Serial1.readBytesUntil('\n')` mengambil data dari buffer hingga menemui enter.
3. Fungsi `sscanf()` digunakan untuk mengekstrak 3 atau 4 variabel tipe *integer* sekaligus dengan sangat cepat.
4. Slave memberhentikan aktivitas motor yang mungkin sedang berjalan dari terminal Serial USB komputer (`serialContinuousStop()`).
5. Terakhir, Slave memanggil fungsi `serialTestFieldCentric(Vx, Vy, W, dur)` untuk mengeksekusi kinematik Mecanum dengan referensi arah Yaw lapangan global dari IMU (MPU6050/MPU9250).

*(Catatan: Mode kinematik bawaannya adalah `Field-Centric`. Anda dapat mengubah pemanggilannya di `serial_commands.ino` menjadi `serialTestRobotCentric` jika diinginkan).*
