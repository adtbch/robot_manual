#ifndef MANIPULATOR2_H
#define MANIPULATOR2_H

#include <Arduino.h>

// ==========================================
// MODUL MANIPULATOR 2 - LENGAN KEDUA (SLAVE 2)
// PID Posisi 3 Axis DC Motor (BTS7960 + PCNT)
// Kontrol Servo Wrist Pitch (LEDC PWM 50Hz)
// ==========================================

// Ticks target sendi lengan
extern volatile int16_t arm2_target_putar;
extern volatile int16_t arm2_target_z;
extern volatile int16_t arm2_target_y;
extern volatile uint8_t arm2_target_wrist;

// Public API
void motor_arm2_init();
void motor_arm2_pid_loop_update();
void motor_arm2_stop_emergency();
void servo_arm2_wrist_set_angle(uint8_t degree);

#endif
