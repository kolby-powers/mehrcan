#pragma once
#include "main.h"

class LocoStore {
  public:
    // Load a loco from eeprom at given address
    Loco load(int eepromID);
    // Save a loco at given eeprom address
    bool save(Loco loco);
    // Returns loco count based on saved cnt at eeprom
    int getCount();

    uint8_t getNewAddress();
};