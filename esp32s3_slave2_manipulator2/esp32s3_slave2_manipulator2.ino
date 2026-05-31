#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Sertakan modul Slave 2
#include "pinout.h"
#include "config.h"
#include "uart_slave_receiver.h"
#include "motor_arm2_pid_control.h"

// Semaphore untuk memproteksi data setpoint Lengan 2
SemaphoreHandle_t arm2Mutex = NULL;

// Handle diagnostik FreeRTOS
TaskHandle_t TaskWatchdogHandle = NULL;
TaskHandle_t TaskLengan2PIDHandle = NULL;

// ==========================================
// 1. TASK KONTROL PID LENGAN 2 DI CORE 1 (100Hz / Tiap 10ms)
// ==========================================
void TaskLengan2PID(void *pvParameters) {
    (void) pvParameters;

    // Inisialisasi hardware Manipulator 2 (PCNT, MCPWM, LEDC)
    motor_arm2_init();

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Update PID posisi 3 sendi bertenaga DC Motor & Servo Wrist
        // Ambil data setpoint terbaru secara aman
        if (xSemaphoreTake(arm2Mutex, portMAX_DELAY) == pdTRUE) {
            motor_arm2_pid_loop_update();
            xSemaphoreGive(arm2Mutex);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

// ==========================================
// 2. TASK UART PARSER & WATCHDOG DI CORE 0 (Fast Polling)
// ==========================================
void TaskWatchdog(void *pvParameters) {
    (void) pvParameters;

    // Inisialisasi receiver UART @921600 bps
    uart_slave_init();

    for (;;) {
        // Parse UART non-blocking mendengarkan instruksi Master
        uart_slave_parse_incoming_data();

        // Jika ada target baru ter-parse, update shared variables secara aman
        if (new_target_ready) {
            if (xSemaphoreTake(arm2Mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                arm2_target_putar = l2_target_putar;
                arm2_target_z     = l2_target_z;
                arm2_target_y     = l2_target_y;
                arm2_target_wrist = l2_target_wrist;

                new_target_ready = false;
                xSemaphoreGive(arm2Mutex);
            }
        }

        // Jalankan polling super cepat dengan jeda minimal 1ms untuk feed watchdog
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ==========================================
// 3. TASK MONITORING DIAGNOSTIK KESEHATAN STACK RAM
// ==========================================
void TaskDiagnostics(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        UBaseType_t watchdog_stack = uxTaskGetStackHighWaterMark(TaskWatchdogHandle) * 4;
        UBaseType_t arm2_stack     = uxTaskGetStackHighWaterMark(TaskLengan2PIDHandle) * 4;

        #if DEBUG_DIAGNOSTICS
            Serial.println("=========================================");
            Serial.printf("[DIAG] Sisa RAM Task Watchdog : %d bytes\n", watchdog_stack);
            Serial.printf("[DIAG] Sisa RAM Task Lengan 2  : %d bytes\n", arm2_stack);
            Serial.printf("[DIAG] Total Sisa RAM Bebas    : %d bytes\n", ESP.getFreeHeap());
            Serial.println("=========================================");
        #endif

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ==========================================
// 4. ARDUINO SETUP
// ==========================================
void setup() {
    Serial.begin(UART_BAUD_DEBUG);

    // Buat Mutex
    arm2Mutex = xSemaphoreCreateMutex();

    if (arm2Mutex != NULL) {
        // Task Watchdog di CORE 0
        xTaskCreatePinnedToCore(
            TaskWatchdog,
            "TaskWatchdog",
            TASK_WATCHDOG_STACK,
            NULL,
            TASK_WATCHDOG_PRIO,
            &TaskWatchdogHandle,
            TASK_WATCHDOG_CORE
        );

        // Task Lengan 2 PID di CORE 1
        xTaskCreatePinnedToCore(
            TaskLengan2PID,
            "TaskLengan2PID",
            TASK_LENGAN2_STACK,
            NULL,
            TASK_LENGAN2_PRIO,
            &TaskLengan2PIDHandle,
            TASK_LENGAN2_CORE
        );

        // Task Diagnostik di Core 1 (Prioritas Rendah)
        xTaskCreatePinnedToCore(
            TaskDiagnostics,
            "TaskDiag",
            2048,
            NULL,
            1,
            NULL,
            1
        );
    } else {
        Serial.println("[FATAL] Gagal membuat Mutex Slave 2!");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("[SYSTEM] ESP32-S3 Slave 2 Manipulator 2 Booted!");
}

// ==========================================
// 5. LOOP UTAMA ARDUINO (Berjalan di Core 1)
// ==========================================
void loop() {
    // Loop default kosong karena proses utama dijalankan secara multitasking
    delay(1000);
}
