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
static bool     led_state             = false; // status LED onboard saat ini (ON/OFF)
static uint32_t waktu_led_terakhir    = 0;     // untuk timing kedip LED onboard

// --- State untuk toggle warna LED PS4 via Options ---
static bool     options_prev          = false; // edge detection: state sebelumnya
static uint8_t  led_color_index       = 0;     // indeks warna aktif

// Palet warna untuk LED PS4 (R, G, B) — kontras tinggi
static const uint8_t led_palet[][3] = {
    {0,   0,   255},  // 0: Biru
    {255, 0,   0  },  // 1: Merah
};
static const uint8_t kLedPaletCount = sizeof(led_palet) / sizeof(led_palet[0]);

// --- Deferred PS4 LED send (agar tidak blocking di send path) ---
static bool     ps4_led_pending   = false;
static uint8_t  ps4_led_r         = 0;
static uint8_t  ps4_led_g         = 0;
static uint8_t  ps4_led_b         = 0;

// =====================================================================
//  FUNGSI: INISIALISASI BLUETOOTH PS4
// =====================================================================

/**
 * Mulai koneksi Bluetooth ke PS4 DualShock 4.
 * Dipanggil SEBELUM inisialisasi WiFi/ESP-NOW di setup().
 */
void ps4_init() {
    PS4.begin(kPs4BluetoothMac);
    ESP_LOGI("ps4_bt", "Bluetooth init — host MAC: %s", kPs4BluetoothMac);
    ESP_LOGI("ps4_bt", "Menunggu koneksi DualShock 4...");
    ESP_LOGI("ps4_bt", "Pair: tahan tombol Share + PS bersamaan");
}

// =====================================================================
//  FUNGSI: INISIALISASI LED STATUS PS4
// =====================================================================

/**
 * Setup pin LED sebagai output.
 * Dipanggil satu kali di setup().
 */
void led_ps4_init() {
    const gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << kLedPs4Pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)kLedPs4Pin, 0); // mulai mati
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
void led_ps4_update(bool ps4_aktif, uint32_t sekarang) {
    if (ps4_aktif) {
        gpio_set_level((gpio_num_t)kLedPs4Pin, 1);
        led_state = true;
    } else {
        if (sekarang - waktu_led_terakhir >= kBlinkIntervalMs) {
            waktu_led_terakhir = sekarang;
            led_state = !led_state;
            gpio_set_level((gpio_num_t)kLedPs4Pin, led_state ? 1 : 0);
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

        // Edge detection: baru connect → set flag LED pending (deferred send)
        if (!ps4_status_sebelumnya) {
            ps4_status_sebelumnya = true;
            ESP_LOGI("ps4_bt", "Terhubung!");
            ps4_led_pending = true;
            ps4_led_r = led_palet[led_color_index][0];
            ps4_led_g = led_palet[led_color_index][1];
            ps4_led_b = led_palet[led_color_index][2];
        }
        return true;
    }

    if (ps4_status_sebelumnya) {
        ps4_status_sebelumnya = false;
        ESP_LOGW("ps4_bt", "Terputus.");
    }

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

    // --- Deteksi edge tombol Options → ganti warna LED PS4 (deferred) ---
    bool options_sekarang = PS4.Options();
    if (options_sekarang && !options_prev) {
        led_color_index = (led_color_index + 1) % kLedPaletCount;
        ps4_led_pending = true;
        ps4_led_r = led_palet[led_color_index][0];
        ps4_led_g = led_palet[led_color_index][1];
        ps4_led_b = led_palet[led_color_index][2];
        ESP_LOGI("ps4_bt", "Options ditekan → LED pending: R=%u G=%u B=%u",
                 ps4_led_r, ps4_led_g, ps4_led_b);
    }
    options_prev = options_sekarang;

    // --- Metadata ---
    nomor_urut_paket++;
    paket.seq       = nomor_urut_paket;
    paket.connected = 1;
}

// =====================================================================
//  FUNGSI: FLUSH LED PS4 (deferred — non-blocking di send path)
// =====================================================================

/**
 * Kirim perintah LED ke PS4 via Bluetooth.
 * Hanya jika ada pending request (connect atau tombol Options).
 * Dipanggil di control_task SETELAH kirim paket.
 */
void ps4_led_flush() {
    if (!ps4_led_pending) return;
    if (!PS4.isConnected()) return;

    ps4_led_pending = false;
    PS4.setLed(ps4_led_r, ps4_led_g, ps4_led_b);
    PS4.sendToController();
    ESP_LOGI("ps4_bt", "LED flushed: R=%u G=%u B=%u",
             ps4_led_r, ps4_led_g, ps4_led_b);
}
