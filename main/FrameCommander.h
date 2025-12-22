#pragma once
#include "main.h"

class FrameCommander {
  public:
    static CANFrame buildFrame(int command);
    static CANFrame parseFrame(CANFrame Frame);
};