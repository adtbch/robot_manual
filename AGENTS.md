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
| `Encoder` (Encoder.h) | slave2 | Quadrature encoder read |

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

## Referensi Dokumentasi

- `esp32s3_slave1_localizer_motion/documentasi_project.md` — arsitektur, kinematik, yaw
- `esp32s3_slave1_localizer_motion/workflow_slave_localizer.md` — development workflow
- `esp32s3_slave2_manipulator2/WIRING_DIAGRAM.md` — wiring harness
- `autotune.md` — auto-tuner behaviour & score function