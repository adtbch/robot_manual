# Analisis Alur Perintah GOTO & KN (Master ⇄ Slave1)

Dokumen ini menjelaskan alur pengiriman data pergerakan (`GOTO` dan `KN`) dari Master ke Slave1, fungsi-fungsi yang terlibat, serta analisis mengapa perintah `GOTO` berbasis teks menyebabkan lag/stuttering dibandingkan dengan `KN` yang lancar.

---

## 1. Alur Perintah Kinematik (KN)

Mode Kinematik (`modeKinematics = true`) mengirimkan kecepatan langsung ($v_x, v_y, \text{yaw}$) dari joystick/DPAD ke Slave1.

### A. Sisi Master
1. **Pemicu**: `loop()` di `master.ino` memanggil `motionControlTick(gLastRxPacket)` setiap iterasi.
2. **Pengolahan Stick**: Di `motion_control.ino`, stick joystick di-map ke RPM target:
   - `vx = mapJoystickToRpm(ly, gTargetSpeedRpm);`
   - `vy = mapJoystickToRpm(lx, gTargetSpeedRpm);`
3. **Pengiriman**: Jika `modeKinematics` aktif:
   - `sendKnCommand(vx, vy, gYawTarget)` dipanggil.
   - Fungsi `sendKnCommand` di `serial_command.ino` mengirim data teks via UART1:
     ```cpp
     slave1Serial.printf("kn %d %d %d\n", vx, vy, yawTarget);
     ```
     *Catatan: Pengiriman biner `sendBinaryMotionCmd` dinonaktifkan (di-comment).*

### B. Sisi Slave1
1. **Penerimaan**: `loop()` di `Slave1motion.ino` memanggil `serialCommandTick()`.
2. **Buffer Uart**: Di `serial.ino`, `readSerialMasterBinary()` mendeteksi data masuk. Karena tidak diawali header biner `0xAA`, data masuk ke buffer teks `masterBuf`.
3. **Parsing Teks**: Saat mendeteksi newline (`\n`), fungsi `parseAndExecuteCommand(masterBuf, Serial1, true)` dipanggil:
   - String dipisahkan menggunakan `strtok` dengan token `"kn"`.
   - Mengambil parameter: `vx`, `vy`, `yaw`.
4. **Eksekusi**: Memanggil `driveFieldCentricWithYawCorrection(vx, vy, yaw)` di `kinematik.ino` untuk menghitung PWM masing-masing roda berdasarkan kinematika mekanum.

---

## 2. Alur Perintah GOTO (Target Koordinat)

Mode GOTO (`modeKinematics = false`) mengirimkan target koordinat absolut ($x, y$ dalam cm, dan $\text{yaw}$ dalam derajat) ke Slave1.

### A. Sisi Master
1. **Pemicu**: `motionControlTick()` dipanggil secara terus-menerus di `master.ino`.
2. **Target Incrementor**:
   - Jika stick joystick didorong, koordinat target master (`gTargetX_cm`, `gTargetY_cm`) di-increment setiap `gotoStepMs` (5ms / 20ms / 100ms):
     ```cpp
     if (vx != 0) gTargetX_cm += (vx > 0) ? GOTO_STEP_CM : -GOTO_STEP_CM;
     if (vy != 0) gTargetY_cm += (vy > 0) ? GOTO_STEP_CM : -GOTO_STEP_CM;
     ```
3. **Pengiriman**:
   - Setiap `GOTO_SEND_INTERVAL_MS` (20ms), Master memanggil `sendGotoCommand`:
     ```cpp
     sendGotoCommand((int16_t)lroundf(gTargetX_cm), (int16_t)lroundf(gTargetY_cm), gYawTarget);
     ```
   - Fungsi `sendGotoCommand` di `serial_command.ino` mengirim teks ke UART1:
     ```cpp
     slave1Serial.printf("goto %d %d %d\n", x_cm, y_cm, yaw_deg);
     ```

### B. Sisi Slave1
1. **Penerimaan**: Sama seperti KN, `readSerialMasterBinary()` mengumpulkan karakter ke `masterBuf`.
2. **Parsing**: Di `serial.ino`, `parseAndExecuteCommand` mencocokkan token `"goto"`:
   - Mengekstrak `x_cm`, `y_cm`, `yaw_deg`, dan `speed_rpm`.
3. **Guard RUNNING (Penyebab Utama Lag)**:
   ```cpp
   // ponytail: skip startWaypoint saat RUNNING — master spam goto tidak reset progress
   if (getWaypointState() != WaypointState::RUNNING) {
       testYawMode = false;
       startWaypoint(x_cm, y_cm, yaw, speed);
   }
   ```
   - Jika Slave1 sedang bergerak menuju waypoint sebelumnya (`RUNNING`), **target baru dari Master diabaikan sepenuhnya**!
   - Nilai target target di Slave1 (`wpTargetX_m`, `wpTargetY_m`) tidak pernah diperbarui selama statusnya masih `RUNNING`.
4. **Eksekusi Loop**:
   - Di `Slave1motion.ino`, jika `wpState != WaypointState::IDLE`, fungsi `waypointTick()` dijalankan untuk menghitung error posisi dan menggerakkan robot menggunakan kontroler P posisi.
5. **Umpan Balik REACHED**:
   - Ketika robot berada di dalam toleransi target (`isWithinWaypointTol`), status berubah menjadi `REACHED`.
   - Slave1 mengirim sinyal teks `"WP: REACHED"` ke Master via `wpNotifyReachedToMaster()`.
   - Master menangkap sinyal ini melalui `parseSlave1Status()` dan mematikan bendera pergerakan otomatis jika ada.
   - Karena status Slave1 sekarang sudah `REACHED` (bukan `RUNNING`), maka pada perintah `goto` berikutnya dari Master, Slave1 baru akan menerima koordinat target yang baru melalui `startWaypoint()`.

---

## 3. Analisis Penyebab Lag dan Freeze pada GOTO

Ada dua masalah utama yang menyebabkan sistem GOTO terasa sangat lag/freeze dibandingkan dengan KN:

### A. Target Wind-up & Penolakan Target (Stuttering)
1. **Master Terus Berjalan**: Ketika joystick ditekan, Master terus menambah target `gTargetX_cm` setiap 20ms tanpa peduli apakah robot sudah bergerak secara fisik atau belum.
2. **Slave1 Menolak Update**: Karena status Slave1 masih `RUNNING` untuk mencapai koordinat pertama (misal `0.85 cm`), Slave1 mengabaikan semua target intermediate dari Master (`1.70`, `2.55`, `3.40`, dst.).
3. **Target Melompat Jauh (Jerky Motion)**: 
   - Ketika Slave1 akhirnya mencapai target pertama (`0.85 cm`), statusnya menjadi `REACHED`.
   - Detik berikutnya, Master mengirim target terbarunya (yang sudah terakumulasi jauh, misal `15.30 cm`).
   - Karena status Slave1 sedang tidak `RUNNING`, Slave1 akhirnya menerima target `15.30 cm` tersebut.
   - Selisih jarak yang besar ini membuat kontroler PID memberikan daya motor maksimal (ngegas mendadak), lalu melambat secara drastis saat mendekati target (ngerem), kemudian menunggu status `REACHED` untuk mengambil target berikutnya.
   - Hasilnya: Robot bergerak tersendat-sendat (ngegas-ngerem secara ekstrem).

### B. Overhead Parsing Teks Berkecepatan Tinggi (Freeze/Lag UART)
1. **Kecepatan Pengiriman Tinggi (20ms)**: Master terus-menerus membanjiri UART1 dengan string `"goto X Y W\n"`.
2. **String Tokenization (`strtok`) & `atof`**:
   - Fungsi string dinamis seperti `strtok` dan `atof` cukup berat untuk dijalankan setiap 20ms di mikrokontroler.
   - Terjadi penumpukan karakter di buffer serial Slave1 jika pemrosesan loop lebih lambat daripada laju penerimaan data.
3. **Spam Log Print**:
   - Ketika target tercapai, Slave1 memanggil `Serial.printf("[WP] Reached! ...")` dan `Serial1.println("WP: REACHED")`.
   - Panggilan `Serial.print` ke USB PC dalam loop frekuensi tinggi dapat memblokir eksekusi CPU (blocking IO) jika buffer hardware penuh. Hal ini menyebabkan mikrokontroler terasa "freeze".

---

## 4. Solusi Terencana: Binary Handshake Target Increment

Untuk mengatasi lag dan stuttering ini, kita akan menerapkan **Synchronous Binary Handshake**:

1. **Komunikasi Biner Penuh (Header `0xAA 0xCC`)**:
   - Slave1 tidak lagi mengirim teks `"WP: REACHED"`.
   - Slave1 mengirim paket biner 4-byte: `[0xAA, 0xCC, WP_STATE, CHECKSUM]`.
   - Parsing di Master menggunakan mesin state biner non-blocking (bebas `strtok`).

2. **Master Menunggu Konfirmasi Slave1 (Event-Driven Increment)**:
   - Master tidak boleh menambah target `gTargetX_cm` secara buta berdasarkan timer (`gJedaGotoStep`).
   - Master hanya boleh melakukan increment target (`+0.85 cm`) jika Slave1 telah melaporkan bahwa ia telah menerima target sebelumnya atau sedang berada dalam state siap menerima target (`REACHED` / `IDLE`).
   - Ini menyelaraskan (sinkronisasi) keinginan Master dengan kemampuan fisik robot Slave1, mencegah akumulasi lag (wind-up).
