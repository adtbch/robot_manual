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

// =====================================================================
//  FUNGSI: INISIALISASI BLUETOOTH PS4
// =====================================================================

/**
 * Mulai koneksi Bluetooth ke PS4 DualShock 4.
 * Dipanggil SEBELUM inisialisasi WiFi/ESP-NOW di setup().
 */
void ps4_init() {
    PS4.begin(kPs4BluetoothMac);
    Serial.printf("[PS4] Bluetooth init — target MAC: %s\n", kPs4BluetoothMac);
    Serial.println("[PS4] Menunggu koneksi DualShock 4...");
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

    // --- Analog stik mentah (-128..127) ---
    paket.lx = PS4.LStickX();
    paket.ly = PS4.LStickY();
    paket.rx = PS4.RStickX();
    paket.ry = PS4.RStickY();

    // --- Perintah gerak (dikali 8 → range ~-1016..1016) ---
    // Y dibalik: pada PS4, dorong maju = nilai negatif LStickY
    paket.x = static_cast<int16_t>(PS4.LStickX()) * 8;
    paket.y = static_cast<int16_t>(-PS4.LStickY()) * 8;
    paket.w = static_cast<int16_t>(PS4.RStickX()) * 8;

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

    // --- Metadata ---
    nomor_urut_paket++;
    paket.seq       = nomor_urut_paket;
    paket.connected = 1;
}
