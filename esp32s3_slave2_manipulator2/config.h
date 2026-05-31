#ifndef CONFIG_H
#define CONFIG_H

// ==========================================
// KONFIGURASI ESP32-S3 SLAVE 2 (LENGAN 2)
// ==========================================

// --- MODE SIMULASI ---
#define DRY_RUN_MODE            0

// --- DEBUG FLAGS ---
#define DEBUG_LENGAN2           1  // Log posisi aktual dan target Lengan 2
#define DEBUG_UART_RX           0  // Log penerimaan data serial target posisi
#define DEBUG_DIAGNOSTICS       1  // Log RAM FreeRTOS

// --- PARAMETER UART ---
#define UART_BAUD_MASTER        921600
#define UART_BAUD_DEBUG         115200
#define UART_RX_BUFFER_SIZE     1024

// --- KONTROL PID LENGAN 2 SUMBU PUTAR (BASE) ---
#define L2_PUTAR_KP             1.8f
#define L2_PUTAR_KI             0.008f
#define L2_PUTAR_KD             0.12f
#define L2_PUTAR_MAX_PWM        90.0f
#define L2_PUTAR_DEADBAND       8

// --- KONTROL PID LENGAN 2 SUMBU Z (VERTISAL) ---
#define L2_Z_KP                 2.2f
#define L2_Z_KI                 0.015f
#define L2_Z_KD                 0.18f
#define L2_Z_MAX_PWM            95.0f
#define L2_Z_DEADBAND           8

// --- KONTROL PID LENGAN 2 SUMBU Y (HORIZONTAL) ---
#define L2_Y_KP                 2.0f
#define L2_Y_KI                 0.01f
#define L2_Y_KD                 0.15f
#define L2_Y_MAX_PWM            90.0f
#define L2_Y_DEADBAND           8

// --- FREERTOS TASK CONFIG ---
#define TASK_LENGAN2_STACK      4096
#define TASK_LENGAN2_PRIO       4        // Prioritas tinggi
#define TASK_LENGAN2_CORE       1        // Kendali PID motor di Core 1

#define TASK_WATCHDOG_STACK     2048
#define TASK_WATCHDOG_PRIO      3        // Prioritas sedang
#define TASK_WATCHDOG_CORE      0        // Parser UART & Failsafe Watchdog di Core 0

// --- FAILSAFE ---
#define FAILSAFE_TIMEOUT_MS     500      // Timeout komunikasi sebelum rem otomatis

#endif
