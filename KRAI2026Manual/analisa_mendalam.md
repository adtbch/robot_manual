# Analisis Mendalam Arsitektur KRAI 2026 Robot Manual
*Checkpoint lengkap — Agustus 2024*

## Status Analisis
- [x] Master Board — Lengkap
- [x] Slave1 Motion Board — Lengkap
- [x] Slave2 Arm Board — Lengkap
- [x] Webserver — Lengkap

---

## 1. Aliran Data Utama
```
[PS4 Controller] ──USB OTG──→ [s3controllerespnow] ──ESP-NOW (0xA5B4)──→ [Master ESP32-S3]
                                                                              │
                                 ┌────────────────────────────────────────────┤
                                 │                     │                      │
                          UART1 (921600)          UART2 (921600)         ESP-NOW (0xC0DE)
                                 │                     │                      │
                                 ▼                     ▼                      ▼
                     [Slave1 Motion]          [Slave2 Arm Board]       [Web Configurator]
                     Core0: I2C(MPU+OLED)     Core0: WebServer HTTP   ESP32 WiFi AP
                     Core1: PID RPM+Kinematik Core1: UART+Sensor+     Panel GUI Tuning PID,
                             +UART Master             Motor+Limit           Mapping Tombol
```

## 2. Komunikasi Antar Board
| Jalur | Arah | Format | Contoh |
|-------|------|--------|--------|
| Master → Slave1 | UART1 (921600) | Teks `kn vx vy yaw`, `goto x y yaw speed`, `wp cancel` | `kn 75 0 -90` |
| Slave1 → Master | UART1 (921600) | Teks `WP: REACHED`, `odomToMaster x y yaw` | `odomToMaster 1.25 -0.50 180.0` |
| Master → Slave2 | UART2 (921600) | Teks `motor id pwm`, `pne r on`, `motortarget enc` | `motor x 500` |
| Slave2 → Master | UART2 (921600) | Teks `prox r 0`, `limit d 1`, `enc x 12345`, `pne r 1` | `prox r 1` |
| Web → Master | ESP-NOW (0xC0DE) | JSON terfragmentasi | `{"section":"pid","data":{...}}` |

## 3. Analisis Bug Teridentifikasi (Riwayat)
| Bug | Root Cause | Fix |
|-----|-----------|-----|
| **Robot Freeze 1** | I2C blocking (MPU+OLED) di Core 1 bareng PID | Pindah I2C Task ke Core 0 |
| **Robot Freeze 2** | dt=0 di PID Yaw karena `driveField...` dipanggil 2x dalam ms yang sama | Guard `(nowMs <= lastCallMs)` → biarkan dalam proper bounds |
| **Robot Muter-muter** | `wpState != RUNNING` guard dikomentari → Waypoint override `kn` terus | Uncomment guard; hanya eksekusi PID waypoint jika state `RUNNING` |
| **Master ↔ Slave1 Backpressure** | UART buffer penuh karena terlalu banyak print | Guard `availableForWrite()` |
| **ESP-NOW send callback** | Core 3.x pakai `wifi_tx_info_t*` bukan `uint8_t*` | `#if ESP_ARDUINO_VERSION >= 3` + recompile library EspUsbHost v1.0.1 |
| **L2 trigger tidak terdeteksi** | `BTN_L2` hanya aktif saat trigger penuh (digital), pakai bit analog | Ganti ke `l2Value > 64` |
| `modeKinematics = false` | File revert ke versi usang setelah refactor | Set ke `true` |

## 4. Ringkasan Per-Board

### A. Master Board
- **Input controller**: ESP-NOW penerima packet `ControlPacket`
- **Fungsi utama**: Gripper state machine (`IDLE→CLOSING→UP→STRAIGHTEN→READY_TO_STAB`), ArmBox state machine (`ARMBOX_WAIT→GRAB→DONE→BACK`), Forest navigation state machine (`COMPUTE→APPROACH→YAW→FOREST_MOVE→ARM_Y→DEPLOY`), Motion control, **relay command ke Slave1 & Slave2**
- **Pin penting**: Encoder X/Y, PWM motor X/Y, Servo D/T/B, Limit X/Y/K, Proximity 1, Flash relay
- **Safety**: Limit switch X/Y untuk homing/reset enc; limit K depan/belakang untuk motor K Slave2 yang di-stop via serial; anti-gravity hold duty untuk motor Y vertikal

### B. Slave1 Motion Board
- **Dual-core allocation**:
  - **Core 0**: FreeRTOS task `I2cTask` — MPU6050 update yaw + OLED display (mutex via `I2cBus`)
  - **Core 1**: Loop utama — serial (UART Master), PID RPM 4 motor, odometry, waypoint tick
- **PID RPM**: PID compute dengan anti-windup, derivative on measurement, feedforward `Kf`, gravity compensation `Kg·sin(roll)`, deadband threshold
- **AutoTuner**: 3 fase (deadband, Kf open-loop, PI closed-loop + load injection), scoring function dengan weight overshoot/burst/rise
- **Encoder**: Internal ISR (4000 RPM, 270 PPR, EMA filtered) + External ESP32Encoder untuk odometry (1600 PPR)
- **I2C Bus**: Mutex token-based untuk mencegah tabrakan MPU6050 vs OLED → release setelah selesai, settle 80µs, + I2C bus recovery (9 clock pulse) jika NACK

### C. Slave2 Arm Board
- **Dual-core allocation**:
  - **Core 0**: FreeRTOS task `webTask` — HTTP Web Server
  - **Core 1**: Loop — antre proxy serial, parse command, motor run tick, limit safety
- **Motor**: Arm Box kanan (X, Y) & Arm Box kiri (K) — 20kHz PWM, 10-bit
- **Limit Safety**: Depan (pin 40), Belakang (pin 39), Turun (pin 38) → motor stop + reset enc.
- **Komunikasi**: UART1 (921600) dengan Master; laporan status sensor `prox`, `limit`, `enc`, `pne`

### D. Web Configurator
- **ESP32 dedicated**, WiFi AP `KRAI2026_Config`, IP `192.168.4.1`
- **ESP-NOW** ke Master dengan magic `0xC0DE`, paket ke-1 dari fragmentasi
- **GUI**: Single-page HTML/CSS/JS inline di PROGMEM (~38KB)
  - Tab: Button Mapping, PID Tuning, Waypoint Tuning, Console
- **Per-section save** via POST → forward ke Master via ESP-NOW
- **Serial relay**: Input console → packet `CONFIG_TYPE_SERIAL` → Master jalankan

## 5. Issues Terbuka (Belum Diverifikasi)
| Issue | Deskripsi | Prioritas |
|-------|-----------|-----------|
| Forest auto-nav non-fungsional | `gMotionWaypointMode` masih diset true, tapi setelah KN-only refactor, tidak ada handler yang mengirim KN berdasarkan target | Medium |
| Binary GOTO di-skip | Master tidak kirim `goto` ke slave1 → Waypoint hanya via USB serial | Medium |
| dt di PID Yaw | Guard pencegahan dt=0 sudah ditambahkan, tapi masih mungkin PID Yaw terlalu agresif kalau core 1 dan core 0 baca MPU bersamaan | High |
| `Linear regression` pada state `STRAIGHTEN` | Gripper otomotif menggunakan `motorXPosSet(0)` yang mungkin override target `waypoint` tiba-tiba | Low |

## 6. Catatan Kode Penting Saat Ini
| File | Konteks | Lokasi Baris |
|------|---------|--------------|
| `master/kinematik.ino` | BUG: `driveFieldCentricRpm(vx, -vy, correctionYaw)` — `vy` ditiadakan, mungkin membalik gerak lateral | 73 |
| `master/motion_control.ino` | `modeKinematics = true` — KN-only | 67 |
| `master/motion_control.ino` | L2 threshold pakai `pkt.l2Value > 64` — fix trigger | 189-191 |
| `Slave1motion/kinematik.ino` | `dt` guard untuk mencegah ∞ output PID | 69 |
| `Slave1motion/serial.ino` | KN idle skip saat waypoint RUNNING | ±baris handler `kn` |
| `Slave1motion/waypoint.ino` | Guard `if (wpState != RUNNING) return;` | 167 |
| `Slave1motion/Slave1motion.ino` | `I2cTask` di Core 0 via lambda | close setup |
