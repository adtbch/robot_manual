# KRAI 2026 Robot Manual — Agent Guide

## Proyek

Firmware multi-board ESP32/ESP32-S3 untuk robot kompetisi KRAI 2026. Setiap board punya fungsi spesifik.

## Struktur Direktori

| Direktori | Board | Fungsi |
|---|---|---|
| `esp32controller/` | ESP32 biasa | PS4 joystick, transmit ESP-NOW ke master |
| `esp32s3_master/` | ESP32-S3 | Relay, mecanum + arm manipulator, serial ke slave |
| `esp32s3_slave1_localizer_motion/` | ESP32-S3 | IMU MPU9250 yaw, odometri, kinematik |
| `esp32s3_slave2_manipulator2/` | ESP32-S3 | Encoder motor arm, limit switch, PID position hold |
| `KRAI2026Manual/master/` | ESP32-S3 | **NEW** — Master board KRAI 2026, flat .ino build |
| `KRAI2026Manual/Slave2arm/` | ESP32-S3 | **NEW** — Slave2 arm manipulator, 4 motor, flat .ino build |

## KRAI2026Manual/master — Arsitektur Flat .ino

Board master baru untuk KRAI 2026. Menggunakan **flat approach**: semua `.ino` dalam satu folder, dikompilasi otomatis oleh Arduino IDE. Tinggal buka `master.ino`, klik Upload.

### Struktur Folder

```
KRAI2026Manual/master/
├── master.ino                  ← main entry: setup() + loop() — alphabetical first
│
├── config.h                    ← shared types: ControlPacket, BTN_*, Jeda (SEMUA modul)
├── espnow.h                    ← ESP-NOW config: MAC whitelist, channel, magic, timing
├── motor.h                     ← Motor config: pin, PWM, MotorConfig struct
├── encoder.h                   ← Encoder config: pin, ESP32Encoder library
├── limit_switch.h              ← Limit switch config: pin
├── servo.h                     ← Servo config: pin, PWM 50Hz, ServoConfig struct
├── relay.h                     ← Relay config: pin
├── proximity.h                 ← Proximity sensor config: pin
├── serial.h                    ← Serial config: UART1 (RX45/TX48), UART2 (RX47/TX21)
│
├── espnow.ino                  ← ESP-NOW: init, WiFi channel, peer, callback, fetchPacket, stats
├── motor.ino                   ← SetupMotors(), pwmMotor(), motorStopAll()
├── encoder.ino                 ← setupEncoders(), getEncoderCount(), resetEncoderCount()
├── limit_switch.ino            ← setupLimits(), readLimitSwitch() — double-read anti-noise
├── servo.ino                   ← setupServos(), setServoAngle()
├── relay.ino                   ← setupRelay(), relayOn(), relayOff(), relayToggle(), relayState()
├── proximity.ino               ← setupProximity(), readProximity()
├── serial.ino                  ← setupSerial() — UART1 + UART2 init
├── arm_control.ino             ← setHoming(), moveToCenter(), updateMotorPositioning()
│
├── mecanum_control.ino         ← kinematik mecanum ke slave1
├── gripper_control.ino         ← toggle servo + motor jog (MODE_GRIPPING)
├── armbox_control.ino          ← kontrol slave2 (MODE_ARM_BOX)
│
├── serial_command.ino          ← serial command parser (motorX, servo, stop)
├── motion_serial.ino           ← UART1 binary protocol ke slave1
├── manipulator_serial.ino      ← UART2 ke slave2
│
├── jeda.ino                    ← placeholder (struct Jeda sudah di config.h)
│
└── mode_control.ino            ← GRIPPING/ARM_BOX toggle, NVS save
```

### Mapping File Lama → Baru

| File lama (`esp32s3_master/`) | File baru (`KRAI2026Manual/master/`) |
|---|---|
| `robot_config.h` | `config.h` + per-module `.h` (`espnow.h`, `motor.h`, dst) |
| `espnow_control.ino` | `espnow.ino` |
| `motor.ino` | `motor.ino` |
| `encoder.ino` | `encoder.ino` |
| `servo.ino` | `servo.ino` |
| `limit_switch.ino` | `limit_switch.ino` |
| `arm.ino` | `arm_control.ino` |
| `mecanum_control.ino` | `mecanum_control.ino` |
| `gripper_control.ino` | `gripper_control.ino` |
| `armbox_control.ino` | `armbox_control.ino` |
| `serial.ino` | `serial_command.ino` |
| `motion_serial.ino` | `motion_serial.ino` |
| `manipulator_serial.ino` | `manipulator_serial.ino` |
| `mode_control.ino` | `mode_control.ino` |

### Build & Upload

Langsung via Arduino IDE — buka `master.ino`, klik Upload.

Atau via `arduino-cli`:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 KRAI2026Manual/master
arduino-cli upload -p COM3 --fqbn esp32:esp32:esp32s3 KRAI2026Manual/master
```

Port aktual: cek dengan `arduino-cli board list`.

### Konvensi Flat .ino

- `master.ino` harus jadi pertama secara alfabet — nama `master` sebelum semua modul.
- `config.h` berisi shared types, konstanta, `extern` declarations, dan function prototypes cross-file.
- Per-modul `.h` (`espnow.h`, `motor.h`, dst) berisi konfigurasi spesifik modul itu.
- Setiap `.ino` `#include` modul `.h` miliknya (misal `motor.ino` → `#include "motor.h"`).
- `master.ino` `#include` semua modul `.h` yang dipakai.
- Internal functions pakai **anonymous namespace** (bukan `static`).
- Global variable: **definition** di `.ino` modul, **`extern` declaration** di `config.h`.
- ESP-NOW `ControlPacket` struct di `config.h` — **identik** dengan `esp_receiver/config.h`.

## KRAI2026Manual/Slave2arm — Slave2 Arm Manipulator

Board slave2 untuk arm manipulator KRAI 2026. Flat .ino approach, 4 motor + 2 encoder + 4 limit switch + 2 proximity + serial.

### Struktur Folder

```
KRAI2026Manual/Slave2arm/
├── Slave2arm.ino         ← main entry: setup() + loop() — alphabetical first
│
├── config.h              ← shared: Jeda
├── motor.h               ← Motor: 4 motor pin, PWM, MotorConfig struct
├── encoder.h             ← Encoder: 2 encoder pin, ESP32Encoder library
├── limit_switch.h        ← Limit switch: 4 pin
├── proximity.h           ← Proximity: 2 pin
├── serial.h              ← Serial: UART1 (RX36/TX35)
├── pneumatic.h           ← Pneumatic: 4 valve pin
│
├── motor.ino             ← SetupMotors(), pwmMotor(), motorStopAll()
├── encoder.ino           ← setupEncoders(), getEncoderCount(), resetEncoderCount()
├── limit_switch.ino      ← setupLimits(), updateLimitSwitches(), readLimitSwitch()
├── proximity.ino         ← setupProximity(), readProximity()
├── serial.ino            ← setupSerial()
├── pneumatic.ino         ← setupPneumatic(), pneumaticOn(), pneumaticOff(), pneumaticToggle(), pneumaticAllOff()
```

### Build & Upload

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 KRAI2026Manual/Slave2arm
arduino-cli upload -p COM3 --fqbn esp32:esp32:esp32s3 KRAI2026Manual/Slave2arm
```

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
| Serial1 | RX=36, TX=35 |
| Pneumatic 1-4 | 49, 9, 10, 11 |

## Skill Wajib

**LOAD `arduino-code-generator` skill** untuk setiap task yang membuat/memodifikasi kode Arduino/embedded. Ini adalah requirement dari user.

## Konvensi Clean Code

- **Hapus dead code** — fungsi yang tidak dipanggil di mana pun harus dihapus, bukan di-comment. Contoh: `pidKinematikYawFieldCentric()` yang tidak pernah dipanggil → hapus.
- **Nama deskriptif** — bukan `pwm`, bukan `val`, gunakan `motorTargetRpm`, `yawCorrection`, `limitSwitchState`.
- **Satuan di nama** — jika nilai dalam derajat, gunakan suffix `_deg`. Jika RPM, `_rpm`. Jika ms, `_ms`. Jika detik, `_sec`.
- ** satu tanggung jawab** — fungsi `pidComputeYaw` hanya hitung PID yaw. Fungsi `driveFieldCentric` hanya rotate koordinat. Jangan campur.
- **Gunakan `static constexpr`** untuk konstanta (namespace, defaults): `static constexpr const char* NVS_NS = "yaw_pid";`
- **Constrain output** — setiap fungsi PID return `int` PWM, di-constrain `±maxPwm`.
- **Wrapping angle untuk PID yaw** — selalu wrap error dan derivative diff ke `[-180, 180]` sebelum kalkulasi PID.

## Build & Upload

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 esp32s3_slave1_localizer_motion
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3 esp32s3_slave1_localizer_motion
arduino-cli monitor -p /dev/ttyACM0 -b 115200
```

Port bisa berbeda (`/dev/ttyACM0`, `/dev/ttyUSB0`). Cek dengan `arduino-cli board list`.

## Arduino IDE Multi-File Compilation Rules

- Main `.ino` (misal `esp32s3_slave1_localizer_motion.ino`) harus ada dan diurutkan first secara alfabet.
- Semua `.ino` di satu folder dikompilasi bersama-sama.
- `#include "robot_config.h"` di setiap `.ino` yang butuh konfigurasi.
- Declarations yang dibutuhkan cross-file taruh di `robot_config.h` (bukan `.ino`).
- Jika pakai `extern` untuk variabel global: definition di `.ino`, `extern` declaration di `robot_config.h`.

## Library Penting

| Library | Board | Fungsi |
|---|---|---|
| `MPU9250` (hideakitai) | slave1 | Yaw quaternion-fusion. API: `mpu.setup()`, `mpu.update()`, `mpu.getYaw()` |
| `Preferences` (ESP32) | semua | NVS flash storage untuk persistensi |
| `ESP-NOW` (native) | controller, master | Wireless komunikasi |
| `ESP32Encoder` (lucky68t) | master | Quadrature encoder read. API: `attachHalfQuad()`, `getCount()`, `clearCount()` |

## MPU9250 Yaw Conventions (slave1)

- **Boot tidak auto-calibrate** — pakai default atau hardcoded bias.
- **Kalibrasi hanya via serial** `CALIB_GYRO`.
- **Yaw relatif dari posisi awal** — tidak absolute heading magnetik.
- **Startup convergence 10 detik** blocking di `setupMPU()` — robot diam saat boot.
- Loop rate 40ms (25 Hz).
- `getYaw()` return degrees, rentang -180..180.
- **Rate-limit OLED ke 200ms** — I2C bus contention dengan MPU9250.

## PID Pattern (canonical)

Setiap PID controller punya:
1. **NVS load** (`initXxx()`) di `setup()` — baca dari Preferences, default jika kosong.
2. **NVS save** (`saveXxx()`) — tulis ke Preferences, dipanggil saat serial command.
3. **NVS key** pakai namespace unik: motor=`"pid_tuning"`, yaw=`"yaw_pid"`.
4. **Reset state** saat load: `pidState.reset()`, set `lastTarget = 0.0f`.
5. **Modular compute** — generic `int pidCompute(PIDState&, target, current, dt)` return `int` PWM.
6. **Overload untuk motor** — `int pidCompute(int motorIdx, float target, float dt)` auto-ambil current dari encoder.

## Serial Command Pattern

```cpp
if (strncmp(cmd, "COMMAND_NAME", N) == 0) {
    // parse dengan sscanf atau strtok
    float kp, ki, kd;
    if (sscanf(cmd + N, "%f %f %f", &kp, &ki, &kd) == 3) {
        // update + save
    } else {
        Serial.println("Usage: COMMAND_NAME <kp> <ki> <kd>");
    }
    return;
}
```

## Critical Safety Rules

- **Limit switch harus double-read** dengan 2ms delay — motor noise cause false trigger.
- **Encoder adalah single source of truth** — semua fungsi pakai global vars (`encoderMotorW/Z/Y`), bukan baca langsung dari interrupt.
- **Port serial bisa terputus** jika ESP32-S3 crash I2C — reset fisik diperlukan.
- **Port `/dev/ttyACM0`** tidak always mounted — cek sebelum upload.

### Limit Switch Double-Read — Penjelasan

**Masalah:** Motor DC menghasilkan electrical noise yang bikin pin GPIO berubah-ubah tanpa sebab.

```
Sinyal asli:      ──────────LOW──────────HIGH─────────
Sinyal + noise:   ──LOW──HIGH──LOW──LOW──HIGH──LOW──HIGH──
                    ↑     ↑     ↑
                    noise noise noise
```

**Solusi:** Baca 2 kali dengan jeda 2ms. Kalau kedua bacaan sama → valid. Kalau beda → ditolak (noise).

**Implementasi (non-blocking):**
- `updateLimitSwitches()` dipanggil di setiap `loop()`
- State machine per switch: baca pertama → tunggu 2ms via `Jeda` → baca kedua → bandingkan
- Hasil disimpan di `limitState[]`, dibaca via `readLimitSwitch(idx)`

**Mengapa 2ms?**
- Terlalu singkat (<1ms) → noise mungkin masih ada
- Terlalu lama (>10ms) → robot sudah gerak terlalu jauh
- 2ms → sweet spot untuk motor noise di robot KRAI

**Ref:** `limit_switch.ino` — `updateLimitSwitches()`, `readLimitSwitch()`

## Referensi Dokumentasi

- `esp32s3_slave1_localizer_motion/documentasi_project.md` — arsitektur, kinematik, yaw
- `esp32s3_slave1_localizer_motion/workflow_slave_localizer.md` — development workflow
- `esp32s3_slave2_manipulator2/WIRING_DIAGRAM.md` — wiring harness
- `autotune.md` — auto-tuner behaviour & score function