#include "armbox_config.h"

// Relay config vector (definition)
std::vector<RelayConfig> relays = {
  {RELAY_1_PIN, false},
  {RELAY_2_PIN, false}
};

void setupRelays() {
  for (size_t i = 0; i < relays.size(); i++) {
    pinMode(relays[i].pin, OUTPUT);
    digitalWrite(relays[i].pin, LOW);
    relays[i].state = false;
  }
  Serial.println("  Relays initialized");
}

void setRelay(int idRelay, bool state) {
  if (idRelay < 0 || (size_t)idRelay >= relays.size()) return;
  digitalWrite(relays[idRelay].pin, state ? HIGH : LOW);
  relays[idRelay].state = state;
}

bool getRelayState(int idRelay) {
  if (idRelay < 0 || (size_t)idRelay >= relays.size()) return false;
  return relays[idRelay].state;
}
