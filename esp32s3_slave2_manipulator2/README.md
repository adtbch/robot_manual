# Sistem Arm Box - Manipulator 2 (Sumbu Z, Y, W)

Sistem kontrol robot arm manipulator receiver untuk mengambil dan memindahkan box berbasis ESP32-S3 menggunakan komunikasi USB Serial & UART.

---

## 📋 Struktur File

Proyek ini menggunakan multi-file compilation Arduino IDE (main file digabungkan dengan file lain secara alfabetis):

1. **`esp32s3_slave2_manipulator2.ino`** - Entry point utama (setup dan loop orkestrasi).
2. **`armbox_config.h`** - Konfigurasi terpusat (pinout, konstanta PWM, servo, & tipe data).
3. **`motor.ino`** - Driver motor DC dengan array `std::vector` & fungsi `pwmMotor()`.
4. **`servo.ino`** - Driver servo gripper.
5. **`encoder.ino`** - Pembacaan encoder via `attachInterruptArg` single ISR.
6. **`relay.ino`** - Driver relay output.
7. **`limit_switch.ino`** - Setup input limit switch.
8. **`arm.ino`** - Logic pergerakan sumbu (`setMotorTarget`, `updateMotorPositioning`, dan `setHoming`).
9. **`serial.ino`** - Parser perintah Serial (USB & UART) menggunakan parser `strtok` & `strcmp`.

---

## 🔌 Konfigurasi Pin (ESP32-S3)

### Sumbu W (Motor Putar / Rotasi)
- RPWM: Pin `16` (LEDC Channel 0)
- LPWM: Pin `15` (LEDC Channel 1)
- Encoder A: Pin `40`
- Encoder B: Pin `39`
- Limit Switch: Pin `10`

### Sumbu Z (Motor Naik Turun / Vertikal)
- RPWM: Pin `7` (LEDC Channel 2)
- LPWM: Pin `6` (LEDC Channel 3)
- Encoder A: Pin `41`
- Encoder B: Pin `42`
- Limit Switch: Pin `11`

### Sumbu Y (Motor Maju Mundur / Horizontal)
- RPWM: Pin `5` (LEDC Channel 4)
- LPWM: Pin `4` (LEDC Channel 5)
- Encoder A: Pin `1`
- Encoder B: Pin `2`
- Limit Switch: Pin `3`

### Servo & Relay
- Servo Pin: Pin `38` (LEDC Channel 6)
- Relay 1: Pin `12`
- Relay 2: Pin `13`

### UART (Komunikasi ke Controller Utama)
- RX: Pin `38`
- TX: Pin `21`
- Baud Rate: `115200`

---

## 📡 Protokol Perintah Serial & UART

Baudrate Serial: **`115200`**. Setiap command diakhiri dengan newline (`\n` atau `\r`). Parameter dipisahkan dengan spasi.

### 1. Perintah Pergerakan Target (Positioning)

| Perintah | Deskripsi | Contoh |
|----------|-----------|--------|
| `motorW <pos>` | Gerakkan sumbu W ke posisi target encoder | `motorW 1500` |
| `motorZ <pos>` | Gerakkan sumbu Z ke posisi target encoder | `motorZ 800` |
| `motorY <pos>` | Gerakkan sumbu Y ke posisi target encoder | `motorY 1200` |
| `stopW` | Hentikan pergerakan sumbu W secara individu | `stopW` |
| `stopZ` | Hentikan pergerakan sumbu Z secara individu | `stopZ` |
| `stopY` | Hentikan pergerakan sumbu Y secara individu | `stopY` |

### 2. Perintah Arah Manual (Cepat / Presets)

| Perintah | Fungsi | Sumbu | Arah Motor |
|----------|--------|-------|------------|
| `w_kanan` / `putar_kanan` | Putar kanan | Sumbu W | RPWM ON |
| `w_kiri` / `putar_kiri` | Putar kiri | Sumbu W | LPWM ON |
| `z_naik` / `naik` / `naik_turun_naik` | Naik | Sumbu Z | RPWM ON |
| `z_turun` / `turun` / `naik_turun_turun` | Turun | Sumbu Z | LPWM ON |
| `y_maju` / `maju` / `maju_mundur_maju` | Maju | Sumbu Y | RPWM ON |
| `y_mundur` / `mundur` / `maju_mundur_mundur` | Mundur | Sumbu Y | LPWM ON |

### 3. Perintah Servo Gripper

- `servo0 <angle>` - Atur sudut servo ke `0` - `180` derajat (Contoh: `servo0 90`)
- `buka` - Preset membuka gripper (Servo ke `180`°)
- `tutup` - Preset menutup gripper (Servo ke `0`°)
- `home` - Preset servo kembali ke tengah (Servo ke `90`°)

### 4. Perintah Relay

- `relay0 <0/1>` - Mengatur Relay 1 (`0` = OFF, `1` = ON)
- `relay1 <0/1>` - Mengatur Relay 2 (`0` = OFF, `1` = ON)
- `relay0_on` / `relay0_off` - Nyalakan / Matikan Relay 1
- `relay1_on` / `relay1_off` - Nyalakan / Matikan Relay 2

### 5. Perintah Sistem Utama

- `homing` - Memulai proses homing berurutan pada semua motor sumbu W, Z, Y (blocking).
- `status` - Menampilkan status koordinat posisi encoder semua sumbu, sudut servo, dan kondisi relay.
- `encoders` - Membaca data mentah (raw counts) semua encoder.
- `stop` - Hentikan paksa seluruh motor dan target pergerakan (Emergency Stop).

---

## 🚀 Contoh Integrasi Komunikasi Python

```python
import serial
import time

# Hubungkan ke COM port ESP32-S3
ser = serial.Serial('COM5', 115200, timeout=1)
time.sleep(2)  # Tunggu inisialisasi boot

# 1. Jalankan Homing
ser.write(b'homing\n')
print(ser.readline().decode().strip())

# 2. Gerakkan sumbu Z ke posisi 800
ser.write(b'motorZ 800\n')
time.sleep(1)

# 3. Buka Gripper
ser.write(b'buka\n')

# 4. Baca Status Akhir
ser.write(b'status\n')
print(ser.readline().decode().strip())

ser.close()
```

---

Copyright © 2026 Robot Manual Team
