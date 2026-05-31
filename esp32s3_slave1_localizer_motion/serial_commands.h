// ============================================================
// SERIAL COMMANDS INTERFACE - Header File
// ============================================================
// Fungsi-fungsi untuk kontrol motor via Serial USB
// Dipanggil dari loop() di file utama

#pragma once

#include "robot_config.h"

// ============================================================
// Function Declarations
// ============================================================

// Print usage help
void printSerialUsage();

// Process serial commands
void processSerialCommands();

// Motor control functions
void serialMoveRpm(int idx, float rpm, unsigned long durationMs);
void serialMovePwm(int idx, int pwm, unsigned long durationMs);
void serialSeqRpm(float rpm, unsigned long durationMs);
void serialSeqPwm(int pwm, unsigned long durationMs);

// Kinematic test functions
void serialTestRobotCentric(int vx, int vy, int vtheta, unsigned long durationMs);
void serialTestFieldCentric(int vx, int vy, int vtheta, unsigned long durationMs);

// Auto-tune commands
void serialAutoTuneSingle(int motorIdx, float initKp = -1.0f, float initKi = -1.0f, float initKd = -1.0f);
void serialAutoTuneAll();

// Non-blocking state machine tick — called from loop()
void serialCommandsTick();
void serialContinuousTick();

// Emergency stop
void serialEmergencyStop();
