#ifndef PS4_INPUT_H
#define PS4_INPUT_H

#include "config.h"

void onPs4Connected();
void onPs4Disconnected();
void resetYaw();
uint32_t buildButtonsMask();
int16_t applyDeadband(int raw);
int16_t mapStickToPwm(int raw);
int batteryPercent(uint8_t rawBattery);
void updateBatteryLed(bool force);
void buildPacketFromPs4(ControlPacket &packet, bool connected);
bool packetChanged(const ControlPacket &a, const ControlPacket &b);

#endif