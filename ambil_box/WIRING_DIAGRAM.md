# Diagram Koneksi - Sistem Arm Box

## 🔌 Koneksi Hardware

### Arduino Board
Disarankan menggunakan **Arduino Mega 2560** atau **ESP32** karena memiliki banyak pin.

---

## 📊 Tabel Koneksi

| Komponen | Pin Signal | Pin Arduino | Keterangan |
|----------|-----------|-------------|------------|
| **MOTOR PUTAR** | | | |
| Motor Driver | RPWM | 15 | PWM untuk arah kanan/CW |
| Motor Driver | LPWM | 16 | PWM untuk arah kiri/CCW |
| Encoder | A | 40 | Encoder channel A (interrupt) |
| Encoder | B | 39 | Encoder channel B |
| Limit Switch | Signal | 3 | Limit switch (INPUT_PULLUP) |
| **MOTOR NAIK TURUN** | | | |
| Motor Driver | RPWM | 6 | PWM untuk naik |
| Motor Driver | LPWM | 7 | PWM untuk turun |
| Encoder | A | 41 | Encoder channel A (interrupt) |
| Encoder | B | 42 | Encoder channel B |
| Limit Switch | Signal | 11 | Limit switch (INPUT_PULLUP) |
| **MOTOR MAJU MUNDUR** | | | |
| Motor Driver | RPWM | 4 | PWM untuk maju |
| Motor Driver | LPWM | 5 | PWM untuk mundur |
| Encoder | A | 1 | Encoder channel A (interrupt) |
| Encoder | B | 2 | Encoder channel B |
| Limit Switch | Signal | 10 | Limit switch (INPUT_PULLUP) |
| **RELAY** | | | |
| Relay Module | IN1 | 12 | Relay 1 control |
| Relay Module | IN2 | 13 | Relay 2 control |
| **SERVO** | | | |
| Servo Motor | Signal | 38 | Servo PWM signal |
| **UART** | | | |
| UART Device | RX | 38 | Receive data |
| UART Device | TX | 21 | Transmit data |

---

## ⚠️ CATATAN PENTING - PIN CONFLICT!

### Konflik Pin 38
**Pin 38 digunakan untuk 2 fungsi:**
- Servo Signal
- UART RX

**Solusi:**
1. **Opsi 1 (Disarankan)**: Pindahkan Servo ke pin lain (misalnya pin 9 atau 10)
   ```cpp
   #define SERVO_PIN      9  // Ganti dari 38 ke 9
   ```

2. **Opsi 2**: Pindahkan UART RX ke pin lain (misalnya pin 19)
   ```cpp
   #define UART_RX_PIN    19  // Ganti dari 38 ke 19
   ```

3. **Opsi 3**: Gunakan Serial hardware berbeda jika board mendukung

---

## 🔧 Detail Koneksi per Komponen

### 1. Motor Driver (BTS7960 atau IBT-2)
```
Motor Driver <---> Arduino
VCC         <---> 5V
GND         <---> GND
RPWM        <---> Pin PWM
LPWM        <---> Pin PWM
R_EN        <---> 5V (enable)
L_EN        <---> 5V (enable)
```

### 2. Encoder (Rotary Encoder)
```
Encoder     <---> Arduino
VCC         <---> 5V
GND         <---> GND
A           <---> Pin Interrupt
B           <---> Pin Digital
```

### 3. Limit Switch
```
Limit Switch <---> Arduino
NO/NC        <---> Pin Digital (INPUT_PULLUP)
COM          <---> GND
```

**Catatan:** Gunakan INPUT_PULLUP, sehingga:
- Switch terbuka = HIGH
- Switch tertutup/tertekan = LOW

### 4. Relay Module
```
Relay Module <---> Arduino
VCC          <---> 5V
GND          <---> GND
IN1          <---> Pin 12
IN2          <---> Pin 13
```

### 5. Servo Motor
```
Servo       <---> Arduino
VCC (Red)   <---> 5V atau External Power
GND (Brown) <---> GND
Signal      <---> Pin 38 (atau pin lain)
```

**⚠️ Peringatan:** Servo berbeban berat membutuhkan power supply eksternal!

### 6. UART Communication
```
UART Device <---> Arduino
TX          <---> RX (Pin 38)
RX          <---> TX (Pin 21)
GND         <---> GND
```

---

## 🔋 Power Supply

### Rekomendasi Power
- **Arduino**: 7-12V DC atau via USB
- **Motor Driver**: 12-24V DC (sesuai spesifikasi motor)
- **Logic (5V)**: Dari Arduino atau power supply terpisah
- **Servo**: 5-6V DC (gunakan power supply terpisah untuk servo besar)

### Skema Power
```
Power Supply 12V -----> Motor Driver VCC
              |
              +-------> DC-DC Converter (12V to 5V)
                        |
                        +-----> Arduino VIN/5V
                        +-----> Servo VCC (jika kecil)
                        +-----> Logic VCC (Relay, Encoder)
```

**⚠️ PENTING:**
1. Gunakan ground bersama (common ground) untuk semua komponen
2. Pisahkan power motor dan logic jika terjadi noise
3. Tambahkan kapasitor (100μF - 1000μF) di power motor driver
4. Gunakan dioda flyback pada relay coil

---

## 🔍 Pin Interrupt

Untuk encoder, gunakan pin yang mendukung interrupt:

**Arduino Mega:**
- Pin 2, 3, 18, 19, 20, 21 (interrupt)
- Pin 40, 41 tidak support interrupt (perlu dipindah!)

**Rekomendasi untuk Mega:**
```cpp
// Pindahkan encoder A ke pin interrupt
#define MOTOR_PUTAR_ENCODER_A        2  // Ganti dari 40
#define MOTOR_NAIK_TURUN_ENCODER_A   3  // Ganti dari 41
#define MOTOR_MAJU_MUNDUR_ENCODER_A  18 // Ganti dari 1
```

**ESP32:**
- Semua GPIO dapat digunakan sebagai interrupt

---

## 📐 Layout Fisik (Contoh)

```
                        [ARDUINO MEGA]
                             |
        +--------------------+--------------------+
        |                    |                    |
    [Motor 1]           [Motor 2]            [Motor 3]
    Driver              Driver               Driver
    + Encoder           + Encoder            + Encoder
    + Limit SW          + Limit SW           + Limit SW
        |                    |                    |
    [Motor DC]          [Motor DC]           [Motor DC]
    Putar               Naik-Turun           Maju-Mundur
    
    
    [Servo] -----> [Gripper]
    
    [Relay 1] ----> [Device 1]
    [Relay 2] ----> [Device 2]
    
    [UART] <----> [PC/Microcontroller]
```

---

## ✅ Checklist Instalasi

- [ ] Semua pin terhubung sesuai tabel
- [ ] Ground bersama untuk semua komponen
- [ ] Power supply terpisah untuk motor dan logic
- [ ] Kapasitor terpasang di motor driver
- [ ] Limit switch terpasang dengan benar
- [ ] Encoder A pada pin interrupt
- [ ] Tidak ada konflik pin (terutama pin 38)
- [ ] Kabel tertata rapi dan tidak terjepit
- [ ] Test kontinuitas dengan multimeter

---

## 🧪 Testing Koneksi

### Test Motor Driver
```cpp
// Upload test code
digitalWrite(RPWM, 128); // Half speed forward
digitalWrite(LPWM, 0);
delay(2000);
digitalWrite(RPWM, 0);   // Stop
```

### Test Encoder
```cpp
// Print encoder value
Serial.println(encoderPutar);
// Putar manual, nilai harus berubah
```

### Test Limit Switch
```cpp
// Print limit switch state
Serial.println(digitalRead(MOTOR_PUTAR_LIMIT));
// Tekan switch, nilai harus berubah HIGH <-> LOW
```

### Test Relay
```cpp
digitalWrite(RELAY_1_PIN, HIGH);
delay(1000);
digitalWrite(RELAY_1_PIN, LOW);
// Dengar "click" dari relay
```

### Test Servo
```cpp
servo.write(0);   delay(1000);
servo.write(90);  delay(1000);
servo.write(180); delay(1000);
// Servo harus bergerak smooth
```

---

## 📞 Support

Jika ada masalah dengan koneksi, cek:
1. Kontinuitas kabel dengan multimeter
2. Tegangan pada setiap komponen
3. Pin assignment di code sesuai dengan wiring
4. Ground terhubung dengan baik

---

**Dibuat:** 2 Juni 2026  
**Versi:** 1.0  
**Author:** Robot Manual Team
