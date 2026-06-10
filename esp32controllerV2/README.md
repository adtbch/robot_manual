# ESP32 Controller — Dual Channel Transmitter

![Platform](https://img.shields.io/badge/platform-ESP32-blue)
![Protocol](https://img.shields.io/badge/protocol-WSN--31%20%7C%20ESP--NOW-orange)
![Input](https://img.shields.io/badge/input-PS4%20DualShock%204-brightgreen)

Firmware ESP32 sebagai controller utama robot. Membaca input PS4 DualShock 4 via Bluetooth, lalu mengirimkan data ke ESP32-S3 menggunakan dua jalur komunikasi redundan yang bisa di-switch secara manual.

---

## Arsitektur Sistem

```
[PS4 DualShock 4]
      | Bluetooth Classic (BR/EDR)
      v
[ESP32 Controller]  <-- tombol BOOT GPIO0 = toggle jalur
      |
      +-- JALUR A (Primary)  --> UART TX26 --> WSN-31 ~~radio~~ WSN-31 --> [ESP32-S3]
      |
      +-- JALUR B (Backup)   --> WiFi ESP-NOW 2.4GHz langsung -----------> [ESP32-S3]
```

Tombol BOOT bawaan ESP32 digunakan untuk toggle jalur aktif kapan saja tanpa restart.

---

## Struktur File

```
esp32controller/
├── esp32controller.ino      Entry point: setup() dan loop() (orkestrasi)
├── config.h                 Konstanta, pin, enum, extern — tidak ada logic
├── packet.ino               ControlPacket struct + checksum + paket stop
├── ps4_bluetooth.ino        Baca input PS4 DualShock 4 via Bluetooth
├── wsn_serial.ino           Kirim data ke WSN-31 via UART binary frame
├── espnow_transmitter.ino   Kirim data ke ESP32-S3 via ESP-NOW WiFi
├── tombol_toggle.ino        Baca tombol BOOT, toggle jalur aktif
└── debug_stats.ino          Output Serial Monitor: boot info + statistik
```

### Tanggung Jawab Setiap File

| File | Tanggung Jawab | Fungsi Publik |
|------|----------------|---------------|
| `config.h` | Konfigurasi terpusat | — |
| `packet.ino` | Format data | `hitung_checksum()`, `buat_paket_stop()` |
| `ps4_bluetooth.ino` | Input PS4 | `ps4_init()`, `ps4_baca_paket()`, `ps4_is_aktif()` |
| `wsn_serial.ino` | Jalur A radio | `wsn_serial_init()`, `kirim_via_wsn31()` |
| `espnow_transmitter.ino` | Jalur B WiFi | `espnow_init()`, `kirim_via_espnow()` |
| `tombol_toggle.ino` | Switch jalur | `tombol_init()`, `tombol_update()` |
| `debug_stats.ino` | Logging | `debug_init()`, `debug_cetak_info_boot()`, `debug_cetak_statistik()` |
| `esp32controller.ino` | Orkestrasi | `setup()`, `loop()` |

---

## Hardware yang Dibutuhkan

| Komponen | Jumlah | Keterangan |
|----------|--------|------------|
| ESP32 Dev Module | 1 | Board utama controller |
| PS4 DualShock 4 | 1 | Input joystick/gyro/tombol |
| Modul WSN-31 | 2 | Sepasang — satu di controller, satu di sisi robot |
| ESP32-S3 | 1 | Board penerima di sisi robot |
| Kabel jumper | 3 | TX, RX, GND untuk koneksi ke WSN-31 |

---

## Wiring

### ESP32 Controller → WSN-31 #1

```
ESP32 Pin 26 (TX)  -->  WSN-31 RXD
ESP32 Pin 25 (RX)  -->  WSN-31 TXD   (untuk future ACK)
ESP32 GND          -->  WSN-31 GND   (WAJIB common ground)
3.3V atau 5V       -->  WSN-31 VCC   (sesuaikan spek modul)
```

### WSN-31 #2 → ESP32-S3 (sisi robot)

```
WSN-31 TXD  -->  ESP32-S3 RX pin
WSN-31 RXD  -->  ESP32-S3 TX pin
WSN-31 GND  -->  ESP32-S3 GND
```

### Tombol BOOT (sudah built-in di ESP32 Dev Module)

```
GPIO0  -->  GND saat ditekan (internal pull-up, tidak perlu resistor)
```

---

## Format Paket Data (ControlPacket)

Struct ini dikirim sebagai binary frame dan **harus identik** di sisi pengirim (ESP32 Controller) dan penerima (ESP32-S3).

```cpp
struct __attribute__((packed)) ControlPacket {
    uint16_t magic;      // 0xA5B4 — validasi paket
    int16_t  x;          // Gerakan lateral  (analog kiri X × 8)
    int16_t  y;          // Gerakan maju/mundur (analog kiri Y × 8, dibalik)
    int16_t  w;          // Rotasi           (analog kanan X × 8)
    int8_t   lx, ly;     // Stik kiri raw (-128..127)
    int8_t   rx, ry;     // Stik kanan raw (-128..127)
    uint8_t  l2Value;    // Trigger L2 (0..255)
    uint8_t  r2Value;    // Trigger R2 (0..255)
    int16_t  gyrX, gyrY, gyrZ;  // Gyro IMU PS4
    uint32_t buttons;    // Bitmask semua tombol
    uint16_t seq;        // Nomor urut paket
    uint8_t  connected;  // 1 = PS4 aktif, 0 = stop
};
// sizeof(ControlPacket) = 29 bytes
```

### Bitmask `buttons`

| Bit | Tombol | Bit | Tombol |
|-----|--------|-----|--------|
| 0 | Cross (X) | 9 | R3 |
| 1 | Circle (O) | 10 | D-pad Up |
| 2 | Triangle | 11 | D-pad Down |
| 3 | Square | 12 | D-pad Left |
| 4 | L1 | 13 | D-pad Right |
| 5 | R1 | 14 | Share |
| 6 | L2 digital | 15 | Options |
| 7 | R2 digital | 16 | PS Button |
| 8 | L3 | 17 | Touchpad |

### Format Frame UART ke WSN-31

```
[0xAA] [0x55] [LEN_L] [LEN_H] [...30 bytes ControlPacket...] [XOR_CHECKSUM]
  ^      ^       ^       ^                                          ^
start1 start2  length (little-endian)                       integritas data
```

Total frame: 4 (header) + 29 (payload) + 1 (checksum) = **34 bytes per paket**

---

## Software Dependencies

### Library yang Diperlukan

| Library | Versi | Install via |
|---------|-------|-------------|
| PS4Controller by aed3 | latest | Arduino Library Manager |
| esp32 board package | >= 2.0 | Boards Manager |

### Install Library PS4Controller

1. Buka Arduino IDE
2. **Sketch → Include Library → Manage Libraries**
3. Cari: `PS4Controller`
4. Pilih yang by **aed3** → klik **Install**

---

## Konfigurasi Sebelum Upload

Buka `config.h` dan isi dua bagian ini:

### 1. MAC Address PS4

```cpp
// Cara cari MAC PS4:
// Flash sketch GetBDAddress (dari contoh library PS4Controller),
// baca output Serial Monitor → salin MAC yang tercetak
static const char kPs4BluetoothMac[] = "xx:xx:xx:xx:xx:xx"; // GANTI
```

### 2. MAC Address ESP32-S3 Target

```cpp
// Cara cari MAC ESP32-S3:
// Buka Serial Monitor ESP32-S3 saat boot → cari baris "MAC STA"
constexpr uint8_t kEspNowTargetMac[6] = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX}; // GANTI
```

### Konfigurasi Opsional

```cpp
constexpr uint8_t  kEspNowChannel   = 1;    // channel WiFi ESP-NOW
constexpr long     kWsnBaudrate     = 115200; // baudrate WSN-31
constexpr int      kWsnTxPin        = 26;   // TX ESP32 ke WSN-31
constexpr int      kWsnRxPin        = 25;   // RX ESP32 dari WSN-31
constexpr uint32_t kSendIntervalMs  = 25;   // interval kirim (ms)
constexpr uint32_t kPs4TimeoutMs    = 500;  // timeout PS4 disconnect (ms)
```

---

## Cara Upload

1. **Pilih board**: `ESP32 Dev Module`
2. **Flash Mode**: Default
3. **Upload Speed**: 921600
4. **Port**: sesuai COM port ESP32

Jika upload gagal: tahan tombol BOOT saat klik Upload, lepas saat muncul `Connecting...`

---

## Penggunaan

### Saat Pertama Menyala

Serial Monitor (115200 baud) akan mencetak:

```
=========================================
     ESP32 CONTROLLER — DUAL CHANNEL
=========================================
PS4 Bluetooth MAC target : a1:b2:c3:d4:e5:f6
ESP-NOW target MAC       : AA:BB:CC:DD:EE:FF
WSN-31 UART              : TX=26 RX=25 @115200 bps
ESP-NOW channel          : 1
ESP-NOW init             : OK
Interval kirim           : 25 ms (40 Hz)
-----------------------------------------
Jalur default            : A — WSN-31 (radio UART)
Tekan tombol BOOT (GPIO0): toggle jalur A <-> B
=========================================
```

### Toggle Jalur

Tekan tombol BOOT ESP32 (sekali) → jalur berpindah dan status tercetak:

```
[TOMBOL] >>> Pindah ke JALUR B — ESP-NOW (WiFi langsung) <<<
[TOMBOL] >>> Pindah ke JALUR A — WSN-31 (radio UART) <<<
```

### Statistik Periodik (tiap 3 detik)

```
[STAT] jalur=A-WSN31   ps4=OK    wsn_tx=480 espnow_tx=0   espnow_err=0 seq=480
[STAT] jalur=B-ESPNOW  ps4=OK    wsn_tx=480 espnow_tx=120 espnow_err=2 seq=600
```

---

## Troubleshooting

<details>
<summary><b>PS4 tidak terhubung</b></summary>

1. Pastikan `kPs4BluetoothMac` di `config.h` sudah diisi dengan MAC PS4 yang benar
2. Flash sketch `GetBDAddress` terlebih dahulu untuk mendapatkan MAC PS4
3. Mode pairing PS4: tahan tombol PS + Share sampai lampu berkedip cepat
4. Pastikan tidak ada device lain yang sudah paired dengan PS4 ini

</details>

<details>
<summary><b>ESP-NOW init GAGAL</b></summary>

1. Pastikan `kEspNowTargetMac` di `config.h` sudah diisi MAC ESP32-S3 yang benar
2. Channel harus sama antara pengirim dan penerima (`kEspNowChannel`)
3. ESP-NOW tidak bisa dipakai bersamaan dengan WiFi.begin() (koneksi ke router)

</details>

<details>
<summary><b>WSN-31 tidak mengirim data</b></summary>

1. Cek wiring: TX ESP32 (pin 26) harus ke RXD WSN-31, bukan TXD
2. Pastikan GND ESP32 dan WSN-31 terhubung (common ground)
3. Pastikan baudrate WSN-31 = 115200 sesuai `kWsnBaudrate`
4. Cek tegangan VCC WSN-31 (3.3V atau 5V sesuai spek modul)

</details>

<details>
<summary><b>Data tidak diterima di ESP32-S3</b></summary>

1. Verifikasi `struct ControlPacket` identik di kedua sisi
2. Cek `kPacketMagic` sama di pengirim dan penerima (0xA5B4)
3. Untuk jalur WSN-31: pastikan pair WSN-31 sudah dikonfigurasi pada channel yang sama
4. Untuk jalur ESP-NOW: pastikan ESP32-S3 sudah daftarkan ESP32 Controller sebagai peer

</details>

---

## Catatan Teknis

### Kenapa ESP-NOW dan Bluetooth bisa bersamaan?

PS4Controller menggunakan **Bluetooth Classic (BR/EDR)**, sedangkan ESP-NOW menggunakan **WiFi 2.4GHz stack**. Keduanya adalah hardware radio yang berbeda di ESP32 sehingga bisa berjalan paralel tanpa interferensi.

> Penting: `espnow_init()` harus dipanggil **sebelum** `ps4_init()` agar stack WiFi selesai inisialisasi sebelum stack Bluetooth dimulai.

### Kenapa format binary bukan JSON/String?

- Binary frame 35 byte jauh lebih kecil dari JSON setara (~200 byte)
- Parsing lebih cepat di sisi penerima (tidak ada alokasi String)
- Sesuai aturan clean code embedded: **Zero Dynamic Allocation**

### Nomor Urut Paket (`seq`)

Field `seq` bertambah setiap paket dikirim. Sisi penerima bisa hitung packet loss:
```
packet_loss = (seq_diterima - seq_sebelumnya) - 1
```

command for compile
```
arduino-cli compile --fqbn "esp32:esp32:esp32:PartitionScheme=huge_app" "C:\Users\NITRO 5\Documents\GitHub\robot_manual\esp32controller"
```

command for upload
```
arduino-cli upload -p COM14 --fqbn "esp32:esp32:esp32:PartitionScheme=huge_app" "C:\Users\NITRO 5\Documents\GitHub\robot_manual\esp32controller"
```

MAC STICK = B8:1E:A4:97:26:D8