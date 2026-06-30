/*
 * =====================================================================
 * FILE    : forest.h
 * PERAN   : Forest waypoint lookup table — 12 posisi forest (4 baris × 3 kolom).
 *
 * LAYOUT:
 *        col0   col1   col2
 *   row0:  1      2      3
 *   row1:  4     [5]     6     ← posisi 5 invalid (forest tengah)
 *   row2:  7     [8]     9     ← posisi 8 invalid (forest tengah)
 *   row3: 10     11     12
 *
 * MODE WARNA:
 *   RED  (default) — koordinat asli lapangan; Y positif = maju
 *   BLUE           — Y dinegasi (gTargetY_cm = -gTargetY_cm); robot mulai dari sisi berlawanan
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#ifndef FOREST_H
#define FOREST_H

#include "config.h"

// =====================================================================
//  TIPE DATA
// =====================================================================

struct ForestWaypoint {
    float   x_cm;
    float   y_cm;
    long    height_enc;   // encoder target motor Y — gunakan MOTOR_Y_LEVEL_*
    int16_t speed_rpm;
    bool    valid;
};

// Approach point di koridor — waypoint sebelum masuk area forest.
//
//  [0] kiri       │        │       [1] kanan
//  [2] bot-kiri   │  GRID  │   [3] bot-kanan
//
// [0] — koridor kiri atas  : dipakai forest 4,7,10; transit keluar kiri
// [1] — koridor kanan atas : dipakai forest 6,9;    transit keluar kanan
// [2] — koridor kiri bawah : forest 11,12 via jalur kiri
// [3] — koridor kanan bawah: forest 11,12 via jalur kanan
struct ForestColApproach {
    float   pre_x_cm;
    float   pre_y_cm;
    bool    has_pre;
    int16_t yaw_deg;  // heading robot saat tiba di approach point (-180..180)
    bool    has_yaw;  // false = skip yaw phase
};

// =====================================================================
//  SHARED STATE
// =====================================================================

extern AllianceColor gAllianceColor;  // default RED
extern int8_t        gLastApproachedCol;  // -1 = belum pernah approach
extern int8_t        gLastForestId;       //  0 = belum ada forest terakhir

// =====================================================================
//  API
// =====================================================================

// Gerak ke forest id (1-12) — panggil tiap loop().
// Compute route otomatis (approach + yaw di tiap corridor + goto forest).
// Return true  = masih berjalan (approach / yaw / gerak ke forest).
// Return false = sudah tiba di forest id, atau id tidak valid.
// Panggil dengan id baru kapan saja untuk reset dan recompute route.
bool goForest(uint8_t id);

// Batalkan sequence aktif.
void cancelForestGoto();

// Keluar dari forest — gerak ke approach [2], yaw pasti 180° (kondisi khusus).
// Return true = masih berjalan; false = sudah tiba di [2] menghadap 180°.
bool exitFromForest();

#endif // FOREST_H
