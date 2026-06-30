/*
 * =====================================================================
 * FILE    : forest_config.h
 * PERAN   : UART proxy forest dest/goto ke master (NVS di master).
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#ifndef FOREST_CONFIG_H
#define FOREST_CONFIG_H

#include "config.h"

uint8_t forestGetDest1();
uint8_t forestGetDest2();
bool forestIsDest1DoneLocal();
const char* forestLastAck();

bool forestQueryDestFromMaster();
bool forestSaveDestinations(uint8_t d1, uint8_t d2);
bool forestSendGoto(uint8_t slot);
void forestSendExit();
void forestSendCancel();

bool parseMasterForestLine(char* line);

#endif // FOREST_CONFIG_H
