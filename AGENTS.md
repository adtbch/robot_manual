# KRAI 2026 Robot Manual — Agent Guide

## Proyek

Firmware multi-board ESP32-S3 untuk robot kompetisi KRAI 2026. Setiap board punya fungsi spesifik.

## Struktur Direktori

| Direktori | Board | Fungsi |
|---|---|---|
| `esp32controller/` | ESP32 biasa | PS4 joystick, transmit ESP-NOW ke master |
| `esp32s3_master/` | ESP32-S3 | Reference lama — relay, mecanum + arm manipulator |
| `esp32s3_slave1_localizer_motion/` | ESP32-S3 | Reference lama — IMU MPU9250 yaw, odometri |
| `esp32s3_slave2_manipulator2/` | ESP32-S3 | Reference lama — encoder motor arm, PID |
| `KRAI2026Manual/master/` | ESP32-S3 | **Master board** — ESP-NOW receiver, relay, mecanum + arm |
| `KRAI2026Manual/Slave2arm/` | ESP32-S3 | **Slave2 arm** — 4 motor, encoder, limit, pneumatic |
| `KRAI2026Manual/Slave1motion/` | ESP32-S3 | **Slave1 motion** — mecanum, MPU6050 yaw, PID |
| `KRAI2026Manual/webserver/` | ESP32 | **Web configurator** — WiFi AP, edit PID/mapping via browser |
| `KRAI2026Manual/web/` | — | Web UI HTML (single-file, embedded di webserver) |

## ESP32 Core Version

**ESP32 Arduino Core 3.1.1** — sudah terinstal di `C:\Users\NITRO 5\AppData\Local\Arduino15\packages\esp32\`.

### Breaking Changes (Core 3.x vs 2.x)

| Lama (Core 2.x) | Baru (Core 3.x) |
|---|---|
| `ledcSetup(ch, freq, res)` + `ledcAttachPin(pin, ch)` | `ledcAttach(pin, freq, res)` |
| `ledcWrite(ch, duty)` | `ledcWrite(pin, duty)` |
| ESP-NOW callback: `void(const uint8_t*, const uint8_t*, int)` | `void(const esp_now_recv_info*, const uint8_t*, int)` — MAC via `info->src_addr` |
| `ledc_channel` di MotorConfig/ServoConfig | Dihapus — `ledcAttach()` auto-assign channel |

### Arduino CLI

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 <folder>
arduino-cli upload -p COM3 --fqbn esp32:esp32:esp32s3 <folder>
arduino-cli monitor -p COM3 -b 115200
```

Path: `C:\Program Files\Arduino CLI\arduino-cli.exe`
Port: cek dengan `arduino-cli board list`.

---

## KRAI2026Manual/master — Master Board

Board master untuk KRAI 2026. Flat .ino approach. ESP-NOW receiver dari controller, relay ke slave1 (UART1) dan slave2 (UART2).

### Struktur Folder

```
KRAI2026Manual/master/
├── master.ino                  ← main entry: setup() + loop()
│
├── config.h                    ← shared types: ControlPacket, BTN_*, Jeda
├── espnow.h                    ← ESP-NOW config: MAC whitelist, channel, magic, function declarations
├── motor.h                     ← Motor: pin, PWM, MotorConfig struct
├── encoder.h                   ← Encoder: pin, ESP32Encoder library
├── limit_switch.h              ← Limit switch: pin
├── servo.h                     ← Servo: pin, PWM 50Hz, ServoConfig struct
├── relay.h                     ← Relay: pin
├── proximity.h                 ← Proximity: pin
├── serial.h                    ← Serial: UART1→slave1, UART2→slave2, function declarations
│
├── espnow.ino                  ← ESP-NOW: init, peer, callback, fetchPacket, isLinkAlive
├── motor.ino                   ← SetupMotors(), pwmMotor(), motorStopAll()
├── encoder.ino                 ← setupEncoders(), getEncoderCount(), resetEncoderCount()
├── limit_switch.ino            ← setupLimits(), readLimitSwitch() — double-read anti-noise
├── servo.ino                   ← setupServos(), setServoAngle()
├── relay.ino                   ← setupRelay(), relayOn(), relayOff(), relayToggle()
├── proximity.ino               ← setupProximity(), readProximity()
├── serial.ino                  ← Unified command handler: PC + slave1 + slave2
├── serial_command.ino          ← Helper: sendRpmCommand() ke slave1
│
├── gripper.ino                 ← Auto gripper: proximity → close servo d → straighten servo b
├── gripper_control.ino         ← Mapping tombol → gripper (R1/L1/R2/L2/Segitiga)
├── motion_control.ino          ← Mapping joystick/dpad → mecanum ke slave1 (SHARE toggle)
└── armbox_control.ino          ← kontrol slave2 (nanti)
```

### Serial Architecture — Unified Command Handler

```
                ┌──────────────────────────┐
                │    parseAndExecute()      │
                │    (Print& out)           │
                └──────────┬───────────────┘
           ┌───────────────┼───────────────┐
           ▼               ▼               ▼
    ┌──────────────┐ ┌──────────────┐ ┌──────────────┐
    │ Serial (USB) │ │ slave1Serial │ │ slave2Serial │
    │    pcBuf     │ │  slave1Buf   │ │  slave2Buf   │
    └──────────────┘ └──────────────┘ └──────────────┘
     PC Monitor       UART1 → slave1    UART2 → slave2
```

- **`HardwareSerial slave1Serial(1)`** — RX=45, TX=48 → slave1
- **`HardwareSerial slave2Serial(2)`** — RX=47, TX=21 → slave2
- **3 buffer terpisah** — PC, slave1, slave2 tidak saling ganggu
- **Response balik ke sumber** — `parseAndExecuteCommand(cmd, Print& out)` tulis ke `out`
- **Command sama** dari mana pun — `motor x 500`, `servo d 90`, `gripper reset`, `status`, dll

### Serial Commands

```
motor <id> <pwm>         — Set motor PWM (contoh: motor x 500)
motorstop                — Stop semua motor
servo <id> <angle>       — Set servo sudut (contoh: servo d 90)
relay <on|off|t>         — Relay on/off/toggle
enc                      — Baca encoder
encreset                 — Reset encoder
limit                    — Baca limit switch
prox                     — Baca proximity
gripper <reset|homing>   — Reset atau homing gripper
status                   — Tampilkan semua status
stop                     — Stop semua motor + servo
help                     — Tampilkan daftar command
```

### Controller Mapping — Gripper (`gripper_control.ino`)

| Tombol | Aksi |
|--------|------|
| R1 | Tutup gripper (servo d = 90) |
| L1 | Buka gripper (servo d = 0) |
| R2 | Homing (buka + lengan awal) |
| L2 | Reset state gripper |
| Segitiga | Siap stab (`gripperReadytoStab()`) |

- **Edge detection** — trigger sekali saat tombol ditekan, tidak trigger saat dihold
- **Mapping gampang** — edit konstanta `BTN_GRIPPER_*` di atas file

### Controller Mapping — Motion (`motion_control.ino`)

| Input | Aksi |
|-------|------|
| DPAD/Analog kiri | Maju/mundur/geser (mecanum) |
| SHARE | Toggle input mode (DPAD ↔ ANALOG) |
| R1 hold | Fast mode (1.5x speed) |
| L1 hold | Slow mode (0.5x speed) |

- **Default mode: DPAD** — tekan SHARE untuk switch ke analog
- **Deadzone 20** — filter noise joystick
- **Safety stop** — PS4 disconnect / ESP-NOW putus → motor stop otomatis

### Serial Command ke Slave1 (`serial_command.ino`)

```
rpm <fr> <fl> <br> <bl>    — 4 motor mecanum RPM
contoh: rpm 500 -500 500 -500
```

### Kinematik Mecanum

```
FR = LY + LX
FL = LY - LX
BR = LY - LX
BL = LY + LX
```

### Auto Gripper (`gripper.ino`)

```
┌──────────┐
│   IDLE   │ ← proximity tidak aktif
└────┬─────┘
     │ proximity = detected
     ▼
┌──────────────┐
│  CLOSING     │ ← servo d tutup (timeout 1s)
└────┬─────────┘
     │ timeout 1s
     ▼
┌──────────────┐
│  STRAIGHTEN  │ ← servo b lurus (no timeout)
└──────────────┘
```

- **`gripperZone1()`** — panggil di loop(), non-blocking
- **`setServoHoming()`** — buka gripper + lengan ke posisi awal
- **`gripperReset()`** — reset state ke IDLE
- **`gripperReadytoStab()`** — lengan ke posisi siap stab

### Build & Upload

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 KRAI2026Manual/master
arduino-cli upload -p COM3 --fqbn esp32:esp32:esp32s3 KRAI2026Manual/master
```

Flash: ~892KB (68%), RAM: ~44KB (13%)

---

## KRAI2026Manual/Slave2arm — Slave2 Arm Manipulator

Board slave2 untuk arm manipulator KRAI 2026. Flat .ino approach, 4 motor + 2 encoder + 4 limit switch + 2 proximity + 4 pneumatic + unified serial.

### Struktur Folder

```
KRAI2026Manual/Slave2arm/
├── Slave2arm.ino         ← main entry: setup() + loop()
│
├── config.h              ← shared: Jeda
├── motor.h               ← Motor: 4 motor pin, PWM, MotorConfig struct
├── encoder.h             ← Encoder: 2 encoder pin, ESP32Encoder library
├── limit_switch.h        ← Limit switch: 4 pin
├── proximity.h           ← Proximity: 2 pin
├── serial.h              ← Serial: UART1→master (MASTER_RX/TX)
├── pneumatic.h           ← Pneumatic: 4 valve pin
│
├── motor.ino             ← SetupMotors(), pwmMotor(), motorStopAll()
├── encoder.ino           ← setupEncoders(), getEncoderCount(), resetEncoderCount()
├── limit_switch.ino      ← setupLimits(), updateLimitSwitches(), readLimitSwitch()
├── proximity.ino         ← setupProximity(), readProximity()
├── serial.ino            ← Unified command handler: PC + master
├── pneumatic.ino         ← setupPneumatic(), pneumaticOn(), pneumaticOff(), pneumaticToggle()
```

### Serial Architecture — Unified Command Handler

```
                ┌──────────────────────────┐
                │    parseAndExecute()      │
                │    (Print& out)           │
                └──────────┬───────────────┘
           ┌───────────────┴───────────────┐
           ▼                               ▼
    ┌──────────────┐               ┌──────────────┐
    │ Serial (USB) │               │ masterSerial │
    │    pcBuf     │               │  masterBuf   │
    └──────────────┘               └──────────────┘
     PC Monitor                     UART1 → master
```

- **`HardwareSerial masterSerial(1)`** — RX=36, TX=35 → master
- **2 buffer terpisah** — PC dan master tidak saling ganggu
- **Response balik ke sumber** — `parseAndExecuteCommand(cmd, Print& out)` tulis ke `out`
- **Command sama** dari mana pun — `motor1 500`, `pne1 on`, `enc`, `status`, dll

### Pin Summary

| Modul | Pin |
|-------|-----|
| Motor1 | 4, 7 |
| Motor2 | 6, 5 |
| Motor3 | 17, 18 |
| Motor4 | 8, 3 |
| Encoder1 | 41, 42 |
| Encoder2 | 1, 2 |
| Limit 1-4 | 40, 39, 38, 37 |
| Proximity 1-2 | 15, 16 |
| Master (UART1) | RX=36, TX=35 |
| Pneumatic 1-4 | 49, 9, 10, 11 |

### Build & Upload

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 KRAI2026Manual/Slave2arm
arduino-cli upload -p COM3 --fqbn esp32:esp32:esp32s3 KRAI2026Manual/Slave2arm
```

Flash: ~339KB (25%), RAM: ~20KB (6%)

---

## KRAI2026Manual/Slave1motion — Slave1 Motion (Mecanum)

Board slave1 untuk motion control KRAI 2026. Flat .ino approach, 4 motor mecanum + MPU6050 yaw + PID + serial.

### Struktur Folder

```
KRAI2026Manual/Slave1motion/
├── Slave1motion.ino       ← main entry: setup() + loop()
│
├── config.h               ← shared: Jeda, ControlPacket, MotorConfig, EncoderConfig, PIDState
├── motor.h                ← Motor: 4 motor pin, PWM config
├── encoder.h              ← Encoder: 4 encoder pin, ESP32Encoder library, RPM
├── mpu.h                  ← MPU6050: I2C pin, interrupt, yaw functions
├── kinematik.h            ← Kinematik: mecanum drive functions
├── pid.h                  ← PID: controller, NVS, rpmMotor
├── serial.h               ← Serial: UART1 (master), UART2 (WSN-31)
│
├── motor.ino              ← SetupMotors(), pwmMotor(), motorStopAll()
├── encoder.ino            ← setupEncoders(), convertEncoderToRPM(), velocity getters
├── mpu.ino                ← setupMPU(), updateYaw(), calibrateGyro(), NVS
├── kinematik.ino          ← driveRobotCentric(), driveFieldCentric(), yaw correction
├── pid.ino                ← pidCompute(), pidComputeYaw(), rpmMotor(), NVS load/save
├── serial.ino             ← setupSerial(), serialRelayTick()
```

### Status

⚠️ **BLOCKED** — MPU6050 library v1.4.4 (`I2cdevlib`) tidak kompatibel dengan ESP32 Core 3.x.
Error: `undefined reference to TwoWire::requestFrom(unsigned char, unsigned char)`.
Perlu update ke Adafruit MPU6050 atau patch `I2Cdev.cpp`.

### Pin Summary

| Modul | Pin |
|-------|-----|
| Motor FR | 6, 7 |
| Motor FL | 3, 8 |
| Motor BR | 15, 16 |
| Motor BL | 18, 17 |
| Encoder FR | 40, 39 |
| Encoder FL | 4, 5 |
| Encoder BR | 1, 2 |
| Encoder BL | 41, 42 |
| MPU SDA | 13 |
| MPU SCL | 14 |
| MPU Interrupt | 46 |
| Serial Master | RX=21, TX=20 |
| Serial WSN | RX=12, TX=11 |
| WSN SET | 19 |

### Libraries

| Library | Fungsi |
|---------|--------|
| ESP32Encoder | Quadrature encoder read |
| MPU6050 (i2cdevlib) | DMP yaw (6-axis) — **perlu patch untuk core 3.x** |
| Preferences | NVS PID storage |

---

## KRAI2026Manual/webserver — Web Configurator

ESP32 dedicated untuk WiFi AP + HTTP server. Edit PID, button mapping, servo presets via browser. Kirim config ke master via ESP-NOW.

### Struktur Folder

```
KRAI2026Manual/webserver/
├── webserver.ino           ← setup + loop: WiFi AP, ESP-NOW, HTTP server
├── config.h                ← constants: AP SSID/password, ESP-NOW channel, magic
├── webpage.h               ← PROGMEM: single-file HTML/CSS/JS
├── espnow_sender.h/.ino    ← ESP-NOW init, send config packet ke master
├── http_server.h/.ino      ← HTTP endpoints: GET /, GET /api/config, POST /api/config
├── config_manager.h/.ino   ← NVS load/save, JSON serialization
```

### ESP-NOW Config Protocol

- **Magic**: `0xC0DE` (berbeda dari joystick `0xA5B4`)
- **Header**: magic(2) + index(1) + total(1) + type(1) = 5 bytes
- **Payload max**: 245 bytes/packet (MTU 250 - header 5)
- **Fragmentasi**: JSON > 245 bytes dipecah jadi beberapa packet
- **Master role**: receiver — handle joystick (`0xA5B4`) DAN config (`0xC0DE`)

### Web UI Features

- Single-file HTML inline (CSS + JS) — `PROGMEM`
- WiFi AP mode — hotspot sendiri, tidak perlu router
- Per-card save buttons — setiap section (mapping_grip, mapping_arm, presets, pid, yaw_pid, motor_pos) save independen
- Section routing: mapping_grip/arm/presets → NVS master; pid/yaw_pid → slave1 via UART; motor_pos → slave2 via UART

### Build & Upload

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 KRAI2026Manual/webserver
arduino-cli upload -p COM3 --fqbn esp32:esp32:esp32 KRAI2026Manual/webserver
```

---

## Skill Wajib

**LOAD `arduino-code-generator` skill** untuk setiap task yang membuat/memodifikasi kode Arduino/embedded.

## Konvensi Flat .ino

- Main `.ino` (misal `master.ino`) harus jadi pertama secara alfabet.
- Semua `.ino` di satu folder dikompilasi bersama-sama.
- Per-modul `.h` (`motor.h`, `encoder.h`, dst) berisi konfigurasi spesifik modul itu.
- Setiap `.ino` `#include` modul `.h` miliknya.
- Internal functions pakai **anonymous namespace** (bukan `static`).
- Global variable: **definition** di `.ino` modul, **`extern` declaration** di `config.h`.
- **`constexpr` untuk pin configs** — compile-time, tidak bisa diubah via web UI.
- **Dynamic configs** (PID, button mapping, servo presets) — NVS-stored, editable via web UI.

## Konvensi Clean Code

- **Hapus dead code** — fungsi yang tidak dipanggil harus dihapus, bukan di-comment.
- **Nama deskriptif** — bukan `pwm`, bukan `val`, gunakan `motorTargetRpm`, `yawCorrection`.
- **Satuan di nama** — `_deg` untuk derajat, `_rpm` untuk RPM, `_ms` untuk ms.
- **Satu tanggung jawab** — fungsi `pidComputeYaw` hanya hitung PID yaw.
- **`static constexpr`** untuk konstanta: `static constexpr const char* NVS_NS = "yaw_pid";`
- **Constrain output** — setiap PID return `int` PWM, di-constrain `±maxPwm`.
- **Wrapping angle** — wrap error dan derivative diff ke `[-180, 180]` sebelum PID.

## Serial Command Pattern (Unified)

Semua board (master, slave1, slave2) pakai pattern yang sama — `parseAndExecuteCommand(cmd, Print& out)`:
- PC (USB Serial) → buffer terpisah → response ke `Serial`
- UART ke board lain → buffer terpisah → response ke `HardwareSerial`
- Command sama dari mana pun, response balik ke sumber

```cpp
void parseAndExecuteCommand(char* cmd, Print& out) {
    char* token = strtok(cmd, " ");
    if (strcmp(token, "motor1") == 0) {
        char* val = strtok(nullptr, " ");
        if (val != nullptr) {
            int pwm = constrain(atoi(val), -PWM_MAX, PWM_MAX);
            pwmMotor(0, pwm);
            out.printf("Motor1 PWM: %d\n", pwm);
        }
    }
    // ... command lainnya
}
```

## Critical Safety Rules

- **Limit switch harus double-read** dengan 2ms delay — motor noise cause false trigger.
- **Encoder adalah single source of truth** — pakai global vars dari encoder.ino.
- **Port serial bisa terputus** jika ESP32-S3 crash I2C — reset fisik diperlukan.
- **Port serial tidak always mounted** — cek dengan `arduino-cli board list` sebelum upload.

### Limit Switch Double-Read

**Masalah:** Motor DC menghasilkan electrical noise yang bikin pin GPIO berubah-ubah.

**Solusi:** Baca 2 kali dengan jeda 2ms. Kalau kedua bacaan sama → valid. Kalau beda → ditolak.

**Implementasi (non-blocking):**
- `updateLimitSwitches()` dipanggil di setiap `loop()`
- State machine per switch: baca pertama → tunggu 2ms via `Jeda` → baca kedua → bandingkan
- Hasil disimpan di `limitState[]`, dibaca via `readLimitSwitch(idx)`

**Ref:** `limit_switch.ino` — `updateLimitSwitches()`, `readLimitSwitch()`

## PID Pattern (canonical)

Setiap PID controller punya:
1. **NVS load** (`initXxx()`) di `setup()` — baca dari Preferences, default jika kosong.
2. **NVS save** (`saveXxx()`) — tulis ke Preferences, dipanggil saat serial command.
3. **NVS key** pakai namespace unik: motor=`"pid_tuning"`, yaw=`"yaw_pid"`.
4. **Reset state** saat load: `pidState.reset()`, set `lastTarget = 0.0f`.
5. **Modular compute** — generic `int pidCompute(PIDState&, target, current, dt)` return `int` PWM.
6. **Overload untuk motor** — `int pidCompute(int motorIdx, float target, float dt)` auto-ambil current dari encoder.

## MPU9250 Yaw Conventions (slave1)

- **Boot tidak auto-calibrate** — pakai default atau hardcoded bias.
- **Kalibrasi hanya via serial** `CALIB_GYRO`.
- **Yaw relatif dari posisi awal** — tidak absolute heading magnetik.
- **Startup convergence 10 detik** blocking di `setupMPU()`.
- Loop rate 40ms (25 Hz).
- `getYaw()` return degrees, rentang -180..180.

## Referensi Dokumentasi

- `esp32s3_slave1_localizer_motion/documentasi_project.md` — arsitektur, kinematik, yaw
- `esp32s3_slave2_manipulator2/WIRING_DIAGRAM.md` — wiring harness
- `autotune.md` — auto-tuner behaviour & score function
