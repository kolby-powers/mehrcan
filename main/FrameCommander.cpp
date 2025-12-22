#include "FrameCommander.h"

CANFrame FrameCommander::buildFrame(int command) {
  bool broadcast = true;

  // Create new frame
  CANFrame Frame;
  Frame.buildFB = 0;

  switch (command) {
    case COMMAND_BOOTLOADER_CAN:
      Frame.command = CMD_BOOTLOADER_CAN;
      // Send start to all
      Frame.data[4] = 0x11;
      Frame.len = 5;
      Frame.prio = PRIO_SYSTEM;
      Frame.status = STATUS_REQUEST;
      Frame.buildFB = 1;
      break;

    case COMMAND_PING:
      broadcast = false;
      // Ping requires no data in request
      Frame.command = CMD_PING;
      Frame.len = 0;
      Frame.prio = PRIO_SYSTEM;
      Frame.status = STATUS_REQUEST;
      Frame.buildFB = 1;
      break;

    case COMMAND_SYSTEM_GO:
      // Send broadcast go
      Frame.command = CMD_SYSTEM;
      Frame.data[4] = CMD_SUB_GO;
      Frame.len = 5;
      Frame.prio = PRIO_SYSTEM;
      Frame.status = STATUS_REQUEST;
      Frame.buildFB = 1;
      break;

    case COMMAND_SYSTEM_HOLD:
      // Send broadcast shutdown of track
      Frame.command = CMD_SYSTEM;
      Frame.data[4] = CMD_SUB_HOLD;
      Frame.len = 5;
      Frame.prio = PRIO_SYSTEM;
      Frame.status = STATUS_REQUEST;
      Frame.buildFB = 1;
      break;

    case COMMAND_SYSTEM_STOP:
      // Send broadcast stop
      Frame.command = CMD_SYSTEM;
      Frame.data[4] = CMD_SUB_STOP;
      Frame.len = 5;
      Frame.prio = PRIO_SYSTEM;
      Frame.status = STATUS_REQUEST;
      Frame.buildFB = 1;
      break;

    case COMMAND_MFX_DISCOVERY:
      // Start mfx full discovery
      Frame.command = CMD_MFX_DISCOVERY;
      Frame.data[0] = 0x20;
      Frame.len = 1;
      Frame.prio = PRIO_SYSTEM;
      Frame.status = STATUS_REQUEST;
      Frame.buildFB = 1;
      broadcast = false;
      break;

    case COMMAND_LOCO_SPEED:
      Frame.command = CMD_LOCO_SPEED;
      Frame.prio = PRIO_SYSTEM;
      Frame.status = STATUS_REQUEST;
      // Data needs to be set later
      Frame.buildFB = 1;
      broadcast = false;
      break;

    case COMMAND_LOCO_DIR:
      Frame.command = CMD_LOCO_DIR;
      Frame.prio = PRIO_SYSTEM;
      Frame.status = STATUS_REQUEST;
      // Data needs to be set later
      Frame.buildFB = 1;
      broadcast = false;
      break;

    case COMMAND_LOCO_FUNC:
      Frame.command = CMD_LOCO_FUNC;
      Frame.prio = PRIO_SYSTEM;
      Frame.status = STATUS_REQUEST;
      // Data needs to be set later
      Frame.buildFB = 1;
      broadcast = false;
      break;
  }

  // Add broadcast address to data
  if (broadcast) {
    for (int i = 0; i < 4; i++) {
      Frame.data[i] = (uint8_t)0x00;
    }
  }

  return Frame;
};

CANFrame FrameCommander::parseFrame(CANFrame Frame) {
  uint32_t canID = Frame.id;

  // 29bit used
  // Prio 2+2Bits at start
  Frame.prio = (uint8_t)(canID & 0xFF000000) >> 25;

  // Command 8bit (5-12 in id)
  Frame.command = (canID >> 17) & 0xFF;

  // Status 1bit (13 in id)
  Frame.status = (canID >> 16) & 0x1;

  // Address last 16bit (14-29)
  Frame.address = (canID & 0x0000FFFF);

  Frame.buildFB = 1;

  return Frame;
}