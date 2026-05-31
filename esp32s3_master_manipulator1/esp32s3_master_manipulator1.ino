static EspNowControlPacket gLastRxPacket = {};

void setup() {
  Serial.begin(115200);

  bool espNowReady = espNowControlInit();
}

void loop() {
  if (espNowControlReadPacket(gLastRxPacket)) {
    Serial.printf("RX seq=%u x=%d y=%d w=%d connected=%u\n",
                  gLastRxPacket.seq,
                  gLastRxPacket.x,
                  gLastRxPacket.y,
                  gLastRxPacket.w,
                  gLastRxPacket.connected);
  }
}