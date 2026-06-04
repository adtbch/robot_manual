/*
 * =====================================================================
 * FILE    : button_map.ino
 * PERAN   : Definisi dan mapping semua tombol PS4 ke ControlPacket bitmask.
 *           File ini memudahkan customisasi mapping tombol tanpa mengubah
 *           kode utama.
 *
 * CARA PENGGUNAAN:
 *   if (pkt.buttons & BTN_R1) { ... }
 *   if (pkt.buttons & BTN_CROSS) { ... }
 *
 * BITMASK REFERENSI:
 *   L1=bit4, R1=bit5, L2=bit6, R2=bit7
 *   Triangle=bit2, Circle=bit1, Cross=bit0, Square=bit3
 *   Up=bit10, Down=bit11, Left=bit12, Right=bit13
 *   Share=bit8, Options=bit9, PS=bit16, Touchpad=bit17
 * =====================================================================
 */

#pragma once

// ============================================================
// SHOULDER & TRIGGER
// ============================================================
#define BTN_L1       (1u << 4)
#define BTN_R1       (1u << 5)
#define BTN_L2       (1u << 6)
#define BTN_R2       (1u << 7)

// ============================================================
// FACE BUTTONS
// ============================================================
#define BTN_CROSS    (1u << 0)
#define BTN_CIRCLE   (1u << 1)
#define BTN_SQUARE   (1u << 2)
#define BTN_TRIANGLE (1u << 3)

// ============================================================
// DPAD — sesuai mapping di esp32controller/ps4_bluetooth.ino
// ============================================================
#define BTN_UP       (1u << 10)
#define BTN_DOWN     (1u << 11)
#define BTN_LEFT     (1u << 12)
#define BTN_RIGHT    (1u << 13)

// ============================================================
// SYSTEM
// ============================================================
#define BTN_SHARE    (1u << 8)
#define BTN_OPTIONS  (1u << 9)
#define BTN_PS       (1u << 10)
#define BTN_TOUCHPAD (1u << 13)

// ============================================================
// SPEED MODE (customize di sini)
// ============================================================
// R1 tahan = cepat, L1 tahan = lambat
#define SPEED_MODE_FAST    300
#define SPEED_MODE_DEFAULT 100
#define SPEED_MODE_SLOW    50
