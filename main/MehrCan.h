#pragma once
#include "main.h"
#include "LocoStore.h"
#include "FrameCommander.h"

class MehrCan
{
public:
	static const bool debugMode = true;
	static const bool oldStyle = true;		   // Dont use packet num for now
	static const uint32_t canUID = 0x4711EEF0; // CAN UID of arduino to use (copy of real cs2 start 0x47 with serial at the end)

	unsigned long loopTime;
	unsigned int pkgNum;
	bool initSuccess;
	Command cmd;
	Device devices[1]; // Just one device for now
	LocoStore locoStore;

	bool begin();
	bool initMBox();
	bool runCommand(int command);
	bool runLocoCommand(int locoCommand, Loco loco, uint8_t data[4], int additionalLen);
	void loop();
	CANFrame canSend(CANFrame Frame);
	CANFrame onReceive(int packetSize);
	CANFrame processRxFrame(CANFrame Frame);
	Loco MFXBind(CANFrame Frame);
	Loco *loadLocos();
	static void debug(String text);
	static void dumpData(unsigned long data, int type);
	static void dumpFrame(CANFrame Frame);

private:
	uint16_t prepareUID();
	uint16_t addPkgNum(uint16_t UID);
};

extern MehrCan MCAN;