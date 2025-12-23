#pragma once
#include "main.h"

class FrameCommander
{
public:
  static const uint8_t broadcastAdr[4] = {0x00, 0x00, 0x00, 0x00}; // Default märklin broadcast address
  static CANFrame buildFrame(int command);
  static CANFrame parseFrame(CANFrame Frame);
};