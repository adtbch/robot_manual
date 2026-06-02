#ifndef ESPNOW_TRANSPORT_H
#define ESPNOW_TRANSPORT_H

#include "config.h"

#include <esp_now.h>

void onEspNowSent(const uint8_t *macAddr, esp_now_send_status_t status);
bool initEspNow();
bool trySendPacket(ControlPacket &packet);

#endif