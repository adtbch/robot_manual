/*
 * File: armbox_config.h
 * Deskripsi: Konfigurasi pin untuk sistem arm box
 * Tanggal: 2 Juni 2026
 */

#ifndef ARMBOX_CONFIG_H
#define ARMBOX_CONFIG_H

// ==============================
// MOTOR PUTAR (Rotasi)
// ==============================
#define MOTOR_PUTAR_RPWM        16
#define MOTOR_PUTAR_LPWM        15
#define MOTOR_PUTAR_ENCODER_A   40
#define MOTOR_PUTAR_ENCODER_B   39
#define MOTOR_PUTAR_LIMIT       3
#define MOTOR_PUTAR_PWM_PIN     MOTOR_PUTAR_RPWM
#define MOTOR_PUTAR_DIR_PIN     MOTOR_PUTAR_LPWM

// ==============================
// MOTOR NAIK TURUN (Vertikal)
// ==============================
#define MOTOR_NAIK_TURUN_RPWM        6
#define MOTOR_NAIK_TURUN_LPWM        7
#define MOTOR_NAIK_TURUN_ENCODER_A   41
#define MOTOR_NAIK_TURUN_ENCODER_B   42
#define MOTOR_NAIK_TURUN_LIMIT       11
#define MOTOR_NAIK_TURUN_PWM_PIN     MOTOR_NAIK_TURUN_RPWM
#define MOTOR_NAIK_TURUN_DIR_PIN     MOTOR_NAIK_TURUN_LPWM

// ==============================
// MOTOR MAJU MUNDUR (Horizontal)
// ==============================
#define MOTOR_MAJU_MUNDUR_RPWM        4
#define MOTOR_MAJU_MUNDUR_LPWM        5
#define MOTOR_MAJU_MUNDUR_ENCODER_A   1
#define MOTOR_MAJU_MUNDUR_ENCODER_B   2
#define MOTOR_MAJU_MUNDUR_LIMIT       10
#define MOTOR_MAJU_MUNDUR_PWM_PIN     MOTOR_MAJU_MUNDUR_RPWM
#define MOTOR_MAJU_MUNDUR_DIR_PIN     MOTOR_MAJU_MUNDUR_LPWM

// ==============================
// RELAY
// ==============================
#define RELAY_1_PIN    12
#define RELAY_2_PIN    13

// ==============================
// SERVO
// ==============================
#define SERVO_PIN      38

// ==============================
// UART COMMUNICATION
// ==============================
#define UART_RX_PIN    38
#define UART_TX_PIN    21

// ==============================
// KONSTANTA MOTOR
// ==============================
#define PWM_MAX        255
#define PWM_MIN        0
#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_RESOLUTION 8
#define MOTOR_PUTAR_RPWM_CHANNEL 0
#define MOTOR_PUTAR_LPWM_CHANNEL 1
#define MOTOR_NAIK_TURUN_RPWM_CHANNEL 2
#define MOTOR_NAIK_TURUN_LPWM_CHANNEL 3
#define MOTOR_MAJU_MUNDUR_RPWM_CHANNEL 4
#define MOTOR_MAJU_MUNDUR_LPWM_CHANNEL 5

// Encoder PPR untuk setiap motor (sesuaikan dengan encoder Anda)
#define ENCODER_PPR_PUTAR         360   // Pulses per revolution
#define ENCODER_PPR_NAIK_TURUN    360
#define ENCODER_PPR_MAJU_MUNDUR   360

// Lead screw pitch (untuk konversi encoder ke mm)
#define LEAD_SCREW_PITCH    8.0  // mm per putaran

// Toleransi posisi encoder (dalam pulsa)
#define POSITION_TOLERANCE  10   // Toleransi untuk menganggap posisi sudah tercapai

// ==============================
// KONSTANTA SERVO
// ==============================
#define SERVO_MIN_ANGLE    0
#define SERVO_MAX_ANGLE    180
#define SERVO_HOME_ANGLE   90
#define SERVO_PWM_FREQ     50
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#define SERVO_TIMER_BITS   16

// ==============================
// KONSTANTA HOMING
// ==============================
#define HOMING_SPEED       50      // PWM speed untuk homing (0-255)
#define HOMING_TIMEOUT     30000    // Timeout homing dalam ms (30 detik)

#endif // ARMBOX_CONFIG_H
