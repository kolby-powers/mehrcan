#pragma once
#include <Arduino.h>

#define DEFAULT_TIMEOUT         500 // Timeout in ms

// Internal command set used for command struct
#define COMMAND_BOOTLOADER_CAN  1
#define COMMAND_PING            18
#define COMMAND_SYSTEM_GO       20
#define COMMAND_SYSTEM_STOP     21
#define COMMAND_SYSTEM_HOLD     22
#define COMMAND_MFX_DISCOVERY   30
#define COMMAND_MFX_BIND        31
#define COMMAND_MFX_VERIFY      32
#define COMMAND_LOCO_SPEED      40
#define COMMAND_LOCO_DIR        41
#define COMMAND_LOCO_FUNC       42

// Märklin command set
// System commands
#define CMD_BOOTLOADER_CAN  0x1B
#define CMD_PING            0x18
#define CMD_SYSTEM          0x00
// MFX Commands
#define CMD_MFX_DISCOVERY   0x01
#define CMD_MFX_BIND        0x02
#define CMD_MFX_VERIFY      0x03
// Loco control
#define CMD_LOCO_SPEED      0x04
#define CMD_LOCO_DIR        0x05
#define CMD_LOCO_FUNC       0x06

#define LOCO_DIR_F          0x01
#define LOCO_DIR_R          0x02
#define LOCO_DIR_SWITCH     0x03

#define CMD_SUB_STOP  0x00
#define CMD_SUB_GO    0x01
#define CMD_SUB_HOLD  0x02

#define PRIO_SYSTEM 0x0

#define STATUS_REQUEST  0
#define STATUS_RESPONSE 1

// Sensors
#define TRACK_SENSOR_DEFAULT_TIMEOUT 500

// Track status
#define TRACK_STATUS_OK           0
#define TRACK_STATUS_STOP         1
#define TRACK_STATUS_SHUTDOWN     2
#define TRACK_STATUS_LOCO_SEARCH  7

struct CANFrame {
  uint32_t id;
  uint8_t prio;
  uint8_t command;
  int status;
  uint16_t address;
  int len;
  uint8_t data[8];
  int buildFB;
  int sendFB;
};

struct Command {
  int intCmd;
  int waitForResponse;
  CANFrame tx;
  CANFrame rx;
};

struct Device {
  uint8_t id[4];
  uint8_t type[2];
};

struct Loco {
  uint8_t uid[4];
  uint8_t sid;
  int eepromID;
};