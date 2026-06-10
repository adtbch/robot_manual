/*
 * =====================================================================
 * FILE    : esp32controller.ino
 * BOARD   : ESP32 (bukan S3)
 * PERAN   : Entry point — setup() init hardware + create FreeRTOS task.
 *           loop() dihapus — semua logic di control_task().
 *
 * ARSITEKTUR (Hybrid Arduino + Native ESP-IDF):
 *
 *   [PS4 DualShock 4]
 *         | Bluetooth Classic (PS4Controller library — Arduino)
 *         v
 *   [ESP32 Controller]
 *         | FreeRTOS control_task (core 1, 5ms cycle)
 *         |   - GPIO native (esp-idf) : button + LED
 *         |   - UART native (esp-idf) : WSN-31
 *         |   - ESP-NOW native        : WiFi langsung
 *         |   - ESP_LOGI native       : logging
 *         |
 *         +-- JALUR A (WSN-31)  --> UART2 --> WSN-31 ~~radio~~ ESP32-S3
 *         +-- JALUR B (ESPNOW)   --> WiFi ESP-NOW langsung --> ESP32-S3
 *
 * URUTAN SETUP:
 *   1. debug_init()
 *   2. wsn_serial_init()     ESP-IDF UART driver
 *   3. ps4_init()            Arduino PS4Controller
 *   4. espnow_init()         ESP-IDF ESP-NOW native
 *   5. tombol_init()         ESP-IDF GPIO
 *   6. led_ps4_init()        ESP-IDF GPIO
 *   7. debug_cetak_info_boot()
 *   8. xTaskCreatePinnedToCore(control_task, ...)
 * =====================================================================
 */

#include "config.h"



// =====================================================================
//  DEFINISI VARIABEL GLOBAL (dideklarasikan extern di config.h)
// =====================================================================

JalurAktif jalur_aktif       = JalurAktif::ESPNOW;
uint16_t   nomor_urut_paket  = 0;

uint32_t stat_kirim_wsn      = 0;
uint32_t stat_kirim_espnow   = 0;
uint32_t stat_espnow_error   = 0;

bool     espnow_siap         = false;
uint32_t waktu_ps4_terakhir  = 0;

// =====================================================================
//  FORWARD DECLARATION
// =====================================================================

static void control_task(void *pvParameters);

// =====================================================================
//  SETUP — inisialisasi semua hardware, lalu create FreeRTOS task
// =====================================================================

void setup() {
    // 1. Serial + ESP-IDF logging
    debug_init();

    // 2. UART ke WSN-31 (native ESP-IDF uart driver)
    wsn_serial_init();

    // 3. Bluetooth PS4 (Arduino PS4Controller library)
    ps4_init();

    // 4. ESP-NOW WiFi (native ESP-IDF)
    espnow_siap = espnow_init();

    // 5. Tombol BOOT (native ESP-IDF gpio)
    tombol_init();

    // 6. LED status PS4 (native ESP-IDF gpio)
    led_ps4_init();

    // 7. Cetak info boot
    debug_cetak_info_boot(espnow_siap);

    // Anchor time untuk PS4 timeout
    waktu_ps4_terakhir = millis();

    // 8. Create FreeRTOS control task — ganti super-loop
    xTaskCreatePinnedToCore(
        control_task,
        "control",
        4096,        // stack words
        NULL,        // parameter
        5,           // priority
        NULL,        // task handle (tidak perlu)
        1            // core 1 (pro CPU)
    );

    ESP_LOGI("main", "Control task created on core 1 — 5ms cycle");
}

// =====================================================================
//  LOOP — idle, semua kerjaan di control_task()
// =====================================================================

void loop() {
    delay(20);
}

// =====================================================================
//  CONTROL TASK — non-blocking
//  - ps4_is_aktif() cache 1×, tidak redundant
//  - PS4.sendToController() deferred via ps4_led_flush()
//  - Tidak ada delay() di critical path
// =====================================================================

static void control_task(void *pvParameters) {
    uint32_t waktu_kirim_terakhir = 0;

    while (1) {
        const uint32_t sekarang = millis();

        // --- 1. Tombol BOOT ---
        tombol_update(sekarang);

        // --- 2. Cek status PS4 — CACHE, call ONCE ---
        const bool ps4_aktif = ps4_is_aktif(sekarang);

        // --- 3. Kirim data — interval dinamis per jalur ---
        const uint32_t interval = (jalur_aktif == JalurAktif::WSN31)
                                ? kSendIntervalMsWsn31
                                : kSendIntervalMsEspnow;

        if (sekarang - waktu_kirim_terakhir >= interval) {
            waktu_kirim_terakhir = sekarang;

            ControlPacket paket = {};
            if (ps4_aktif) {
                ps4_baca_paket(paket);
            } else {
                buat_paket_stop(paket);
            }

            if (jalur_aktif == JalurAktif::WSN31) {
                kirim_via_wsn31(paket);
            } else {
                kirim_via_espnow(paket);
            }
        }

        // --- 4. LED status onboard ---
        led_ps4_update(ps4_aktif, sekarang);

        // --- 5. Flush LED PS4 (deferred) ---
        ps4_led_flush();

        // --- 6. Statistik periodik ---
        debug_cetak_statistik(sekarang, ps4_aktif);

        // --- 7. Dynamic sleep: tidur sampai 3ms sebelum next send ---
        uint32_t next_send = waktu_kirim_terakhir + interval;
        int32_t sisa = (int32_t)(next_send - millis());
        if (sisa > 5) {
            delay((uint32_t)(sisa - 3));
        } else if (sisa > 0) {
            delay(1);
        }
    }
}
