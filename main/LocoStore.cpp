#include <Arduino.h>
#include <EEPROM.h>
#include "LocoStore.h"

Loco LocoStore::load(int eepromID) {
  Loco loco;

  // Get eeprom saved loco cnt
  int count = getCount();

  if (eepromID > count) {
    return loco;
  }

  // Pos seems to be valid
  loco.eepromID = eepromID;

  // Store loco
  // First byte of loco is sid
  int addressStart = 100 + (10 * eepromID);

  loco.sid = EEPROM.read(addressStart);

  // Next 4 bytes are uid as HI/LO
  for (int i = 1; i < 5; i++) {
    loco.uid[i - 1] = EEPROM.read(addressStart + i);
  }

  return loco;
};

bool LocoStore::save(Loco loco) {
  // Save loco at given address
  // Check count matches, move eepos otherwise
  int count = getCount();

  if (loco.eepromID > count + 1 || loco.eepromID <= 0) {
    loco.eepromID = ++count;  // Add at new pos
  }

  int addressStart = 100 + (10 * loco.eepromID);

  // Write sid
  EEPROM.update(addressStart, loco.sid);

  // Write uid HI/LO
  for (int i = 1; i < 5; i++) {
    EEPROM.update(addressStart + i, loco.uid[i - 1]);
  }

  // Write new count
  EEPROM.update(100, (uint8_t)count);

  return true;
};

int LocoStore::getCount() {
  int cnt = (int)EEPROM.read(100);
  Serial.println(cnt);

  if (cnt == 255) {
    return 0;
  }

  return cnt;
};

uint8_t LocoStore::getNewAddress() {
  int cnt = getCount();
  uint8_t addr = 0x01;  // First loco address

  if (cnt > 0) {
    return (uint8_t)++cnt;
  }

  return addr;
}