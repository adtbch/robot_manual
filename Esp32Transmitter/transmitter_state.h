#ifndef TRANSMITTER_STATE_H
#define TRANSMITTER_STATE_H

#include "config.h"

extern bool lastConnected;
extern volatile bool callbackConnected;
extern bool espNowReady;
extern volatile bool sendBusy;
extern volatile uint16_t espNowFailBurst;
extern uint32_t lastSendMs;
extern uint32_t lastStatusMs;
extern uint16_t txSeq;
extern bool espNowInitTried;
extern bool ledStateInitialized;
extern bool lastBatteryLow;
extern uint32_t ps4ConnectedAtMs;
extern uint32_t lastEspNowInitAttemptMs;
extern bool receiverMacValid;

extern ControlPacket txPacket;
extern ControlPacket lastSentPacket;

#endif