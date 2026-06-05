#include "armbox_config.h"

// Relay config vector (definition)
std::vector<RelayConfig> relays = {
  {RELAY_1_PIN, false},
  {RELAY_2_PIN, false}
};

void setupRelays() {
  for (size_t i = 0; i < relays.size(); i++) {
    pinMode(relays[i].pin, OUTPUT);
    digitalWrite(relays[i].pin, HIGH);
    relays[i].state = false;
  }
  Serial.println("  Relays initialized");
}

void relay(int idRelay, int value) {
  if (idRelay < 0 || (size_t)idRelay >= relays.size()) return;
  // value: 0 = ON (nyala), 1 = OFF (mati)
  bool state = (value == 0);
  digitalWrite(relays[idRelay].pin, state ? LOW : HIGH); // LOW untuk ON, HIGH untuk OFF (sesuai dengan wiring relay)
  relays[idRelay].state = state;
  Serial.printf("Relay %d: %s\n", idRelay, state ? "ON" : "OFF");
}

bool getRelayState(int idRelay) {
  if (idRelay < 0 || (size_t)idRelay >= relays.size()) return false;
  return relays[idRelay].state;
}
