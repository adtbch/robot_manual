/*
 * =====================================================================
 * FILE    : autoTuner.h
 * PERAN   : Auto-Tuner PID (3-Phase: Deadband -> Kf -> PI).
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef AUTOTUNER_H
#define AUTOTUNER_H

#include "config.h"

void startAutoTune(int motorIdx);
void startAutoTuneAll();
void autoTunerAbort();
bool isAutoTunerRunning();
void autoTunerTick(bool bootPressed);

#endif // AUTOTUNER_H
