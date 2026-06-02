# Sistem Arm Box - Ambil Box

Sistem kontrol robot arm untuk mengambil dan memindahkan box menggunakan Arduino.

## 📋 Daftar File

1. **ambil_box.ino** - File master (main program)
2. **armbox_config.h** - Konfigurasi pin dan konstanta
3. **arm_putar.ino** - Kontrol motor putar (rotasi)
4. **arm_naik_turun.ino** - Kontrol motor naik turun (vertikal)
5. **arm_maju_mundur.ino** - Kontrol motor maju mundur (horizontal)
6. **arm_servo.ino** - Kontrol servo gripper
7. **arm_relay.ino** - Kontrol relay
8. **uart_communication.ino** - Komunikasi UART

## 🔌 Konfigurasi Pin

### Motor Putar (Rotasi)
- RPWM: Pin 15
- LPWM: Pin 16
- Encoder A: Pin 40
- Encoder B: Pin 39
- Limit Switch: Pin 3

### Motor Naik Turun (Vertikal)
- RPWM: Pin 6
- LPWM: Pin 7
- Encoder A: Pin 41
- Encoder B: Pin 42
- Limit Switch: Pin 11

### Motor Maju Mundur (Horizontal)
- RPWM: Pin 4
- LPWM: Pin 5
- Encoder A: Pin 1
- Encoder B: Pin 2
- Limit Switch: Pin 10

### Relay
- Relay 1: Pin 12
- Relay 2: Pin 13

### Servo
- Servo Pin: Pin 38

### UART Communication
- RX: Pin 38
- TX: Pin 21
- Baud Rate: 115200

## 📡 Protokol Komunikasi UART

### Format Perintah
Setiap perintah diakhiri dengan newline character (`\n`)

### Perintah Motor Putar
- `PUTAR_KANAN` - Putar ke kanan
- `PUTAR_KIRI` - Putar ke kiri
- `PUTAR_STOP` - Stop motor putar
- `PUTAR_POS_<nilai>` - Putar ke posisi encoder tertentu (contoh: PUTAR_POS_500)
- `PUTAR_HOME` - Homing motor putar

### Perintah Motor Naik Turun
- `NAIK_TURUN_NAIK` - Naik
- `NAIK_TURUN_TURUN` - Turun
- `NAIK_TURUN_STOP` - Stop motor naik turun
- `NAIK_TURUN_POS_<nilai>` - Ke posisi tertentu (contoh: NAIK_TURUN_POS_300)
- `NAIK_TURUN_HOME` - Homing motor naik turun

### Perintah Motor Maju Mundur
- `MAJU_MUNDUR_MAJU` - Maju
- `MAJU_MUNDUR_MUNDUR` - Mundur
- `MAJU_MUNDUR_STOP` - Stop motor maju mundur
- `MAJU_MUNDUR_POS_<nilai>` - Ke posisi tertentu (contoh: MAJU_MUNDUR_POS_800)
- `MAJU_MUNDUR_HOME` - Homing motor maju mundur

### Perintah Servo
- `SERVO_BUKA` - Buka gripper
- `SERVO_TUTUP` - Tutup gripper
- `SERVO_HOME` - Servo ke posisi home (90°)
- `SERVO_SETENGAH` - Gripper setengah buka
- `SERVO_ANGLE_<nilai>` - Set sudut servo (contoh: SERVO_ANGLE_45)

### Perintah Relay
- `RELAY_1_ON` - Nyalakan relay 1
- `RELAY_1_OFF` - Matikan relay 1
- `RELAY_2_ON` - Nyalakan relay 2
- `RELAY_2_OFF` - Matikan relay 2
- `RELAY_ALL_ON` - Nyalakan semua relay
- `RELAY_ALL_OFF` - Matikan semua relay

### Perintah Sistem
- `HOMING` - Homing semua motor
- `STATUS` - Request status sistem
- `STOP` - Emergency stop semua motor

### Response Format
- `OK:<perintah>` - Perintah berhasil dieksekusi
- `ERROR:<pesan>` - Terjadi error
- `INFO:<pesan>` - Informasi
- `STATUS:<data>` - Response status sistem

## 🚀 Cara Menggunakan

### 1. Upload ke Arduino
1. Buka file `ambil_box.ino` di Arduino IDE
2. Pastikan semua file `.ino` dan `.h` ada di folder yang sama
3. Pilih board: Arduino Mega atau ESP32
4. Upload program

### 2. Testing Komponen
Program memiliki fungsi `testAllComponents()` untuk testing semua komponen secara otomatis.

### 3. Komunikasi UART
Contoh penggunaan dari Python:
```python
import serial

ser = serial.Serial('COM3', 115200, timeout=1)

# Homing
ser.write(b'HOMING\n')
response = ser.readline().decode().strip()
print(response)

# Putar ke posisi 500
ser.write(b'PUTAR_POS_500\n')
response = ser.readline().decode().strip()
print(response)

# Buka gripper
ser.write(b'SERVO_BUKA\n')
response = ser.readline().decode().strip()
print(response)

ser.close()
```

## ⚠️ Catatan Penting

1. **Limit Switch**: Pastikan semua limit switch terpasang dengan benar
2. **Homing**: Selalu lakukan homing sebelum operasi otomatis
3. **Power Supply**: Pastikan supply daya cukup untuk semua motor
4. **Encoder**: Sesuaikan nilai `ENCODER_PPR` di config sesuai encoder Anda
5. **Pin Conflict**: Pin UART RX (38) sama dengan pin Servo (38), pertimbangkan untuk mengubah salah satu

## 🔧 Troubleshooting

### Motor tidak bergerak
- Cek koneksi motor driver
- Cek supply daya
- Cek pin PWM

### Encoder tidak akurat
- Cek koneksi encoder
- Sesuaikan nilai ENCODER_PPR
- Pastikan interrupt bekerja

### Limit switch tidak berfungsi
- Cek koneksi kabel
- Cek dengan multimeter
- Pastikan menggunakan INPUT_PULLUP

### UART tidak merespon
- Cek koneksi TX/RX
- Pastikan baud rate sama (115200)
- Cek dengan Serial Monitor

## 📝 Lisensi

Copyright © 2026 Robot Manual Team

## 👥 Author

Robot Manual Team - 2 Juni 2026
