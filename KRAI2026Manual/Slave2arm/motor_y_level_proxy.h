/*
 * =====================================================================
 * FILE    : motor_y_level_proxy.h
 * PERAN   : UART proxy motor Y levels ke master (tanpa NVS di Slave2).
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#ifndef MOTOR_Y_LEVEL_PROXY_H
#define MOTOR_Y_LEVEL_PROXY_H

#include <Arduino.h>

bool motorYLevelQueryMaster(long out[6]);
bool motorYLevelSaveToMaster(const long levels[6]);
const char* motorYLevelLastAck();
bool parseMasterMotorLevelLine(char* line);

#endif // MOTOR_Y_LEVEL_PROXY_H
