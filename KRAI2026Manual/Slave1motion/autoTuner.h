/*
 * =====================================================================
 * FILE    : autoTuner.h
 * PERAN   : Auto-Tuner PID (Step Response + Scoring).
 *           Mencari Kf, lalu Kp, lalu Ki terbaik secara otomatis.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef AUTOTUNER_H
#define AUTOTUNER_H

#include "config.h"

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void startAutoTune(int motorIdx);
void autoTunerTick();
bool isAutoTunerRunning();

#endif // AUTOTUNER_H
