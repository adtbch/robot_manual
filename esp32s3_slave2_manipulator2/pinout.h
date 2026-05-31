#ifndef PINOUT_H
#define PINOUT_H

// ==========================================
// PEMETAAN GPIO ESP32-S3 SLAVE 2 (LENGAN 2)
// ==========================================

// Motor Lengan 2 Sumbu Putar (Driver BTS7960)
#define PIN_L2_PUTAR_RPWM    4
#define PIN_L2_PUTAR_LPWM    5

// Motor Lengan 2 Sumbu Z (Driver BTS7960)
#define PIN_L2_Z_RPWM        8
#define PIN_L2_Z_LPWM        9

// Motor Lengan 2 Sumbu Y (Driver BTS7960)
#define PIN_L2_Y_RPWM        14
#define PIN_L2_Y_LPWM        15

// Encoder Lengan 2 Sumbu Putar
#define PIN_ENC_L2_PUTAR_A   6
#define PIN_ENC_L2_PUTAR_B   7

// Encoder Lengan 2 Sumbu Z
#define PIN_ENC_L2_Z_A       10
#define PIN_ENC_L2_Z_B       11

// Encoder Lengan 2 Sumbu Y
#define PIN_ENC_L2_Y_A       12
#define PIN_ENC_L2_Y_B       13

// Servo Wrist Lengan 2 (LEDC PWM)
#define PIN_SERVO_L2_WRIST   16

// UART Communication (Mendengarkan Master)
#define PIN_UART_MASTER_RX   18  // RX dari ESP Master TX
#define PIN_UART_MASTER_TX   17  // TX ke ESP Master RX

#endif
