# ESP32-S3 Master Manipulator 1 - ESP-NOW Receiver

## Deskripsi

Kode ini adalah **receiver ESP-NOW** untuk ESP32-S3 Master Manipulator 1 yang menerima data kontrol dari controller (joystick/gamepad) secara wireless menggunakan protokol ESP-NOW.

## Fitur

- ✅ **Komunikasi 1 Arah** - Receiver menerima data dari controller
- ✅ **ESP-NOW Protocol** - Komunikasi wireless tanpa WiFi router
- ✅ **MAC Whitelist** - Keamanan dengan validasi MAC address
- ✅ **Magic Number Validation** - Validasi paket dengan magic number
- ✅ **Sequence Number** - Deteksi packet loss dan duplikasi
- ✅ **Link Alive Detection** - Deteksi timeout koneksi
- ✅ **Statistics Monitoring** - Monitor paket diterima, ditolak, dll

## Struktur File

```
esp32s3_master_manipulator1/
├── esp32s3_master_manipulator1.ino  # File utama
├── espnow_control.ino               # Implementasi ESP-NOW receiver
├── robot_config.h                   # Konfigurasi dan struktur data
└── README.md                        # Dokumentasi ini
```

## Cara Setup

### 1. Dapatkan MAC Address Controller

Upload sketch ke controller dan buka Serial Monitor untuk mendapatkan MAC address.

### 2. Edit Konfigurasi MAC Address

Edit file `robot_config.h` baris 14-15:

```cpp
const uint8_t espNowAllowedTransmitterStaMac[6] = {0x58, 0xBF, 0x25, 0x8B, 0xDB, 0x18};
const uint8_t espNowAllowedTransmitterApMac[6] = {0x58, 0xBF, 0x25, 0x8B, 0xDB, 0x19};
```

Ganti dengan MAC address controller Anda.

### 3. Upload ke ESP32-S3

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 esp32s3_master_manipulator1
arduino-cli upload -p COM3 --fqbn esp32:esp32:esp32s3 esp32s3_master_manipulator1
```

### 4. Monitor Serial Output

```bash
arduino-cli monitor -p COM3 -c baudrate=115200
```

## Struktur Data Paket

Paket yang diterima dari controller memiliki struktur `EspNowControlPacket`:

```cpp
typedef struct {
    uint16_t magic;       // Magic number (0xA5B4)
    int16_t x;            // Joystick kiri X (-127 to 127)
    int16_t y;            // Joystick kiri Y (-127 to 127)
    int16_t w;            // Joystick kanan X untuk rotasi (-127 to 127)
    int8_t lx;            // Joystick kiri X (analog)
    int8_t ly;            // Joystick kiri Y (analog)
    int8_t rx;            // Joystick kanan X (analog)
    int8_t ry;            // Joystick kanan Y (analog)
    uint8_t l2Value;      // Trigger L2 (0-255)
    uint8_t r2Value;      // Trigger R2 (0-255)
    int16_t gyrX;         // Gyroscope X
    int16_t gyrY;         // Gyroscope Y
    int16_t gyrZ;         // Gyroscope Z
    uint32_t buttons;     // Button states (bitmask)
    uint16_t seq;         // Sequence number
    uint8_t connected;    // Status koneksi (0=disconnect, 1=connected)
} EspNowControlPacket;
```

## Cara Menggunakan di Kode

### Membaca Paket dari Controller

```cpp
EspNowControlPacket packet = {};

void loop() {
    // Baca paket terbaru
    if (espNowControlReadPacket(packet)) {
        // Ada paket baru
        Serial.printf("X=%d Y=%d W=%d\n", packet.x, packet.y, packet.w);
    }
    
    // Update statistik
    espNowControlTick();
}
```

### Cek Button yang Ditekan

```cpp
if (espNowControlReadPacket(packet)) {
    // Cek button X/Cross
    if (packet.buttons & BTN_CROSS) {
        Serial.println("Button X ditekan!");
    }
    
    // Cek button Circle
    if (packet.buttons & BTN_CIRCLE) {
        Serial.println("Button O ditekan!");
    }
    
    // Cek button L1
    if (packet.buttons & BTN_L1) {
        Serial.println("Button L1 ditekan!");
    }
}
```

### Cek Status Koneksi

```cpp
void loop() {
    // Cek apakah link masih hidup
    if (espNowControlIsLinkAlive()) {
        Serial.println("Controller terhubung");
    } else {
        Serial.println("Controller timeout/disconnect");
    }
}
```

## Button Definitions

Gunakan konstanta ini untuk cek button:

- `BTN_CROSS` - Button X / Cross
- `BTN_CIRCLE` - Button O / Circle
- `BTN_SQUARE` - Button Square
- `BTN_TRIANGLE` - Button Triangle
- `BTN_L1` - Button L1
- `BTN_R1` - Button R1
- `BTN_L2` - Button L2
- `BTN_R2` - Button R2
- `BTN_SHARE` - Button Share
- `BTN_OPTIONS` - Button Options
- `BTN_L3` - Joystick kiri tekan
- `BTN_R3` - Joystick kanan tekan
- `BTN_PS` - Button PS
- `BTN_TOUCHPAD` - Touchpad tekan
- `BTN_UP` - D-Pad Up
- `BTN_DOWN` - D-Pad Down
- `BTN_LEFT` - D-Pad Left
- `BTN_RIGHT` - D-Pad Right

## Troubleshooting

### Tidak Ada Data Diterima

1. Cek MAC address di `robot_config.h` sudah benar
2. Pastikan controller sudah upload dan running
3. Cek channel WiFi sama (default: channel 1)
4. Jarak maksimal 100-200 meter

### Packet Loss Tinggi

1. Kurangi jarak antara controller dan receiver
2. Hindari obstacle metal/beton tebal
3. Cek interferensi WiFi lain di sekitar

### Link Timeout Terus

1. Cek `espNowLinkAliveMs` di `robot_config.h` (default: 180ms)
2. Pastikan controller mengirim paket secara kontinyu
3. Cek Serial Monitor untuk statistik RX

## Serial Monitor Output

Contoh output normal:

```
Receiver STA MAC: A4:CF:12:34:56:78
Allowed TX STA : 58:BF:25:8B:DB:18
Allowed TX AP  : 58:BF:25:8B:DB:19
ESP-NOW ready=true channel=1
RX seq=1 x=0 y=0 w=0 connected=1
RX seq=2 x=10 y=20 w=0 connected=1
ESPNOW RX any=100 ok=98 rejMac=0 rejLen=0 rejMagic=0 rejSeq=2 link=OK
```

## Konfigurasi Lanjutan

Edit `robot_config.h` untuk konfigurasi:

```cpp
// Timeout link (ms)
const unsigned long espNowLinkAliveMs = 180;

// Interval print statistik (ms)
const unsigned long espNowStatsIntervalMs = 1000;

// Channel WiFi (1-13)
const uint8_t espNowChannel = 1;

// Enable/disable MAC whitelist
const bool espNowEnableMacWhitelist = true;
```

## Referensi

- Workflow lengkap: `workflow.md`
- Contoh receiver lain: `Esp32Receiver/Esp32Receiver.ino`
- ESP-NOW Documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
