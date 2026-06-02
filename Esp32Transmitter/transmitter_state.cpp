#include "transmitter_state.h"

bool lastConnected = false;
volatile bool callbackConnected = false;
bool espNowReady = false;
volatile bool sendBusy = false;
volatile uint16_t espNowFailBurst = 0;
uint32_t lastSendMs = 0;
uint32_t lastStatusMs = 0;
uint16_t txSeq = 0;
bool espNowInitTried = false;
bool ledStateInitialized = false;
bool lastBatteryLow = false;
uint32_t ps4ConnectedAtMs = 0;
uint32_t lastEspNowInitAttemptMs = 0;
bool receiverMacValid = true;

ControlPacket txPacket = {};
ControlPacket lastSentPacket = {};