/*
 * =====================================================================
 * FILE    : button.h
 * PERAN   : Driver button sederhana (dengan debounce).
 *           Digunakan untuk tombol boot/mode.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef BUTTON_H
#define BUTTON_H

#include "config.h"

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupButton();
void updateButton();
bool isButtonShortPressed(); // Ditekan singkat (< 3 detik), return true saat dilepas
bool isButtonLongPressed();  // Ditahan ≥ 3 detik, return true saat dilepas
bool isButtonLongHolding();  // True SELAMA masih ditahan ≥ 3 detik (belum dilepas)

#endif // BUTTON_H
