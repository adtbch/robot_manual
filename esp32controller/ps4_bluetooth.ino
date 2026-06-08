/*
 * =====================================================================
 * FILE    : ps4_bluetooth.ino
 * PERAN   : Segala sesuatu yang berhubungan dengan PS4 DualShock 4
 *           via Bluetooth Classic.
 *
 * ISI FILE INI:
 *   - ps4_init()         Inisialisasi Bluetooth PS4 (dipanggil di setup())
 *   - ps4_baca_paket()   Baca semua input PS4 → isi ControlPacket
 *   - ps4_is_aktif()     Cek apakah koneksi PS4 masih hidup (dengan timeout)
 *
 * LIBRARY:
 *   PS4Controller by aed3
 *   Install: Arduino IDE → Library Manager → cari "PS4Controller"
 *
 * CATATAN:
 *   PS4Controller menggunakan Bluetooth Classic (BR/EDR),
 *   bukan BLE. Ini berbeda stack dengan WiFi/ESP-NOW sehingga
 *   keduanya bisa berjalan bersamaan di ESP32.
 * =====================================================================
 */

#include "config.h"
#include <PS4Controller.h>

// =====================================================================
//  VARIABEL LOKAL (hanya dipakai di file ini)
// =====================================================================

static bool     ps4_status_sebelumnya = false; // untuk deteksi perubahan koneksi
static bool     led_state             = false; // status LED saat ini (ON/OFF)
static uint32_t waktu_led_terakhir    = 0;     // untuk timing kedip LED

// --- State untuk toggle warna LED PS4 via Options ---
static bool     options_prev          = false; // edge detection: state sebelumnya
static uint8_t  led_color_index       = 0;     // indeks warna aktif

// Palet warna untuk LED PS4 (R, G, B) — kontras tinggi
static const uint8_t led_palet[][3] = {
    {0,   0,   255},  // 0: Biru
    {255, 0,   0  },  // 1: Merah
};
static const uint8_t kLedPaletCount = sizeof(led_palet) / sizeof(led_palet[0]);

// --- State untuk LED warna saat Share held ---
static bool     share_held_prev       = false;
static bool     led_share_mode        = false;

// =====================================================================
//  FUNGSI: INISIALISASI BLUETOOTH PS4
// =====================================================================

/**
 * Mulai koneksi Bluetooth ke PS4 DualShock 4.
 * Dipanggil SEBELUM inisialisasi WiFi/ESP-NOW di setup().
 */
void ps4_init() {
    // PS4.begin() membutuhkan MAC address ESP32 SENDIRI sebagai host,
    // bukan MAC PS4 controller. Pakai MAC dari config.h.
    PS4.begin(kPs4BluetoothMac);
    Serial.printf("[PS4] Bluetooth init — host MAC: %s\n", kPs4BluetoothMac);
    Serial.println("[PS4] Menunggu koneksi DualShock 4...");
    Serial.println("[PS4] Pair: tahan tombol Share + PS bersamaan");
}

// =====================================================================
//  FUNGSI: INISIALISASI LED STATUS PS4
// =====================================================================

/**
 * Setup pin LED sebagai output.
 * Dipanggil satu kali di setup().
 */
void led_ps4_init() {
    pinMode(kLedPs4Pin, OUTPUT);
    digitalWrite(kLedPs4Pin, LOW); // mulai dalam keadaan mati
}

// =====================================================================
//  FUNGSI: UPDATE LED STATUS PS4 (non-blocking)
// =====================================================================

/**
 * Kontrol LED berdasarkan status koneksi PS4:
 *   - PS4 terhubung   → LED menyala terus (steady ON)
 *   - PS4 terputus    → LED berkedip (blink)
 *
 * Dipanggil setiap iterasi loop().
 *
 * @param ps4_aktif true jika PS4 sedang aktif/terhubung
 */
void led_ps4_update(bool ps4_aktif) {
    if (ps4_aktif) {
        // PS4 aktif → LED menyala terus
        digitalWrite(kLedPs4Pin, HIGH);
        led_state = true;
    } else {
        // PS4 terputus → LED berkedip
        // Ambil nilai millis dari loop utama
        const uint32_t sekarang = millis();
        if (sekarang - waktu_led_terakhir >= kBlinkIntervalMs) {
            waktu_led_terakhir = sekarang;
            led_state = !led_state;
            digitalWrite(kLedPs4Pin, led_state ? HIGH : LOW);
        }
    }
}

// =====================================================================
//  FUNGSI: CEK STATUS KONEKSI PS4 (dengan timeout)
// =====================================================================

/**
 * Kembalikan true jika PS4 terhubung DAN paket terakhir belum timeout.
 * Update variabel waktu_ps4_terakhir jika PS4 masih aktif.
 *
 * @param sekarang   nilai millis() dari loop utama
 * @return true      jika PS4 aktif dan dalam batas timeout
 */
bool ps4_is_aktif(uint32_t sekarang) {
    if (PS4.isConnected()) {
        waktu_ps4_terakhir = sekarang;

        // Cetak info saat baru terhubung
        if (!ps4_status_sebelumnya) {
            ps4_status_sebelumnya = true;
            Serial.println("[PS4] Terhubung!");
            // Set LED ke warna terakhir saat pertama kali connect
            uint8_t r = led_palet[led_color_index][0];
            uint8_t g = led_palet[led_color_index][1];
            uint8_t b = led_palet[led_color_index][2];
            PS4.setLed(r, g, b);
            PS4.sendToController();
            Serial.printf("[PS4] LED diset ke R=%u G=%u B=%u\n", r, g, b);
        }
        return true;
    }

    // Cetak info saat baru terputus
    if (ps4_status_sebelumnya) {
        ps4_status_sebelumnya = false;
        Serial.println("[PS4] Terputus.");
    }

    // Masih dalam batas timeout sejak terakhir ada data
    return (sekarang - waktu_ps4_terakhir) <= kPs4TimeoutMs;
}

// =====================================================================
//  FUNGSI: BACA INPUT PS4 → ISI ControlPacket
// =====================================================================

/**
 * Baca semua data dari PS4Controller dan isi struct ControlPacket.
 * Dipanggil hanya jika ps4_is_aktif() == true.
 *
 * Mapping:
 *   Analog kiri  X/Y → x/y (perintah gerak lateral & maju/mundur)
 *   Analog kanan X   → w   (perintah rotasi)
 *   Semua tombol     → bitmask buttons
 *   Gyro IMU PS4     → gyrX/Y/Z
 *
 * @param paket Referensi paket yang akan diisi
 */
void ps4_baca_paket(ControlPacket &paket) {
    paket.magic = kPacketMagic;

    // --- Analog stik mentah (-128..127) — int8_t, jangan discale ---
    paket.lx = PS4.LStickX();
    paket.ly = PS4.LStickY();
    paket.rx = PS4.RStickX();
    paket.ry = PS4.RStickY();

    // --- Trigger analog (0..255) ---
    paket.l2Value = PS4.L2Value();
    paket.r2Value = PS4.R2Value();

    // --- Gyro IMU dari PS4 ---
    paket.gyrX = PS4.GyrX();
    paket.gyrY = PS4.GyrY();
    paket.gyrZ = PS4.GyrZ();

    // --- Bitmask tombol ---
    uint32_t btn = 0;
    if (PS4.Cross())    btn |= (1u << 0);
    if (PS4.Circle())   btn |= (1u << 1);
    if (PS4.Triangle()) btn |= (1u << 2);
    if (PS4.Square())   btn |= (1u << 3);
    if (PS4.L1())       btn |= (1u << 4);
    if (PS4.R1())       btn |= (1u << 5);
    if (PS4.L2())       btn |= (1u << 6);
    if (PS4.R2())       btn |= (1u << 7);
    if (PS4.L3())       btn |= (1u << 8);
    if (PS4.R3())       btn |= (1u << 9);
    if (PS4.Up())       btn |= (1u << 10);
    if (PS4.Down())     btn |= (1u << 11);
    if (PS4.Left())     btn |= (1u << 12);
    if (PS4.Right())    btn |= (1u << 13);
    if (PS4.Share())    btn |= (1u << 14);
    if (PS4.Options())  btn |= (1u << 15);
    if (PS4.PSButton()) btn |= (1u << 16);
    if (PS4.Touchpad()) btn |= (1u << 17);
    paket.buttons = btn;

    // --- Deteksi edge tombol Options → ganti warna LED PS4 ---
    bool options_sekarang = PS4.Options();
    if (options_sekarang && !options_prev) {
        led_color_index = (led_color_index + 1) % kLedPaletCount;
        uint8_t r = led_palet[led_color_index][0];
        uint8_t g = led_palet[led_color_index][1];
        uint8_t b = led_palet[led_color_index][2];
        PS4.setLed(r, g, b);
        PS4.sendToController();
    }
    options_prev = options_sekarang;

    // --- Set command byte + LED feedback ---
    bool share_now = PS4.Share();
    paket.command = CMD_NONE;  // default

    // Share dilepas → kembalikan ke warna normal
    if (!share_now && share_held_prev) {
        if (led_share_mode) {
            uint8_t r = led_palet[led_color_index][0];
            uint8_t g = led_palet[led_color_index][1];
            uint8_t b = led_palet[led_color_index][2];
            PS4.setLed(r, g, b);
            PS4.sendToController();
        }
        led_share_mode = false;
    }
    share_held_prev = share_now;

    if (share_now) {
        bool x_now     = PS4.Cross();
        bool circleNow = PS4.Circle();
        bool squareNow = PS4.Square();
        bool up_now    = PS4.Up();
        bool left_now  = PS4.Left();
        bool right_now = PS4.Right();
        bool down_now  = PS4.Down();

        // Share + Circle → save semi-auto Circle
        if (circleNow) {
            paket.command = CMD_SAVE_CIRCLE;
            PS4.setLed(255, 255, 0);
            PS4.sendToController();
            led_share_mode = true;
        }
        // Share + Square → save semi-auto Square
        else if (squareNow) {
            paket.command = CMD_SAVE_SQUARE;
            PS4.setLed(255, 255, 0);
            PS4.sendToController();
            led_share_mode = true;
        }
        // Share + X + Down hold 2 detik → reset
        else if (x_now && down_now) {
            static uint32_t reset_hold_start = 0;
            static bool reset_hold_prev_local = false;
            bool reset_combo = true;
            if (reset_combo && !reset_hold_prev_local) {
                reset_hold_start = millis();
            }
            if (millis() - reset_hold_start >= 2000) {
                paket.command = CMD_RESET;
                PS4.setLed(0, 255, 0);
                PS4.sendToController();
                led_share_mode = true;
                reset_hold_start = millis();
            }
            reset_hold_prev_local = reset_combo;
        }
        // Share + X + Dpad (Up/Left/Right) → save preset servo
        else if (x_now && (up_now || left_now || right_now)) {
            if (up_now)        paket.command = CMD_SAVE_UP;
            else if (left_now)  paket.command = CMD_SAVE_LEFT;
            else if (right_now) paket.command = CMD_SAVE_RIGHT;
            PS4.setLed(255, 255, 0);
            PS4.sendToController();
            led_share_mode = true;
        }
    }

    // --- Metadata ---
    nomor_urut_paket++;
    paket.seq       = nomor_urut_paket;
    paket.connected = 1;
}
