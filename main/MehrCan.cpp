#include <CAN.h>
#include "MehrCan.h"

bool MehrCan::begin()
{
	// Set clock for AZ-Delivery module
	CAN.setClockFrequency(8E6);

	// Start can with märklin 250k speed
	int tryCnt = 10;

	while (!CAN.begin(250E3) && tryCnt > 0)
	{
		debug("Failed to init CAN! Retry...");
		delay(500);
	}

	if (!tryCnt)
	{
		debug("Failed to init CAN after 10 tries!");
		return false;
	}

	// Init Märklin digital box
	bool initSuccess = initMBox();
	unsigned long loopTime = millis();

	return initSuccess;
};

void MehrCan::loop()
{
	// Check for incoming pkg
	int packetSize = CAN.parsePacket();
	long pkgID = CAN.packetId();

	if (packetSize || pkgID > 0)
	{
		onReceive(packetSize);
	}

	// Command timeout update
	unsigned long diff = (millis() - loopTime);
	loopTime = millis();

	if (cmd.waitForResponse > 0)
	{
		if (cmd.waitForResponse - diff < 0)
		{
			cmd.waitForResponse = 0;
			debug("Pkg timeout...");
			dumpFrame(cmd.tx);
		}
		else
		{
			cmd.waitForResponse -= diff;
		}
	}
};

bool MehrCan::initMBox()
{
	// Init is called at arduino startup, if box is responding, send stop just in case arduino crashed before
	// Send bootloader start cmd
	// Use CANFrame without command because theres no response for this
	CANFrame frame = FrameCommander::buildFrame(COMMAND_BOOTLOADER_CAN);

	if (frame.buildFB)
	{
		frame = canSend(frame);

		if (frame.sendFB)
		{
			// Wait
			delay(1000);

			return runCommand(COMMAND_PING);
		}
	}

	return false;
};

Loco *MehrCan::loadLocos()
{
	int cntAvailable = locoStore.getCount();
	Loco *locos = new Loco[cntAvailable];

	for (int i = 0; i < cntAvailable; i++)
	{
		locos[i] = locoStore.load(i + 1); // Store starts at 1
	}

	return locos;
};

bool MehrCan::runCommand(int command)
{
	Command cmd;
	cmd.intCmd = command;
	cmd.tx = FrameCommander::buildFrame(command);
	cmd.waitForResponse = DEFAULT_TIMEOUT; // Default true

	// Check build
	if (!cmd.tx.buildFB)
	{
		return false;
	}

	// Send command
	cmd.tx = canSend(cmd.tx);

	if (!cmd.tx.sendFB)
	{
		return false;
	}

	return true;
};

bool MehrCan::runLocoCommand(int locoCommand, Loco loco, uint8_t data[4], int additionalLen)
{
	// Create base frame
	CANFrame Frame = FrameCommander::buildFrame(locoCommand);

	if (!Frame.buildFB)
	{
		debug("Failed to build Loco control Frame!");
		return false;
	}

	// Add loco address
	Frame.data[0] = 0x00;
	Frame.data[1] = 0x00;
	Frame.data[2] = 0x40; // MFX address space
	Frame.data[3] = loco.sid;

	if (additionalLen)
	{
		for (int i = 0; i < additionalLen; i++)
		{
			Frame.data[(4 + i)] = data[i];
		}
	}

	Frame.len = (4 + additionalLen);

	// Create command and wait for fb
	Command cmd;
	cmd.waitForResponse = DEFAULT_TIMEOUT;
	cmd.intCmd = locoCommand;
	cmd.tx = canSend(Frame);

	if (cmd.tx.sendFB)
	{
		return true;
	}

	// Skip waiting for response
	cmd.waitForResponse = 0;
	debug("Failed to execute Loco command!");

	return false;
};

CANFrame MehrCan::canSend(CANFrame Frame)
{
	debug("Sending request");

	uint32_t canID = 0;

	canID |= (uint32_t)Frame.prio << 25;	// Prio 2+2Bits at the start
	canID |= (uint32_t)Frame.command << 17; // 8Bit command
	canID |= (uint32_t)Frame.status << 16;	// 1Bit status 0 = request / 1 = answer

	// 00100000110001010100111100001
	// Prio|Command |R|Hash
	// 0010|00001100|0|1010100111100001
	// 4   |8       |1|16

	// Add hash of uid at the end
	canID |= addPkgNum(prepareUID());
	// canID |= random(0x10000) & 0xff7f | 0x0300;

	// Add ID to frame
	Frame.id = canID;

	if (debugMode)
	{
		Serial.println(canID, BIN);
		Serial.println(canID, HEX);
	}

	if (CAN.beginExtendedPacket(Frame.id, Frame.len))
	{
		debug("Package created");
	}

	if (Frame.len > 0)
	{
		for (int i = 0; i < Frame.len; i++)
		{
			CAN.write(Frame.data[i]);

			if (debugMode)
			{
				Serial.println(Frame.data[i], HEX);
			}
		}
	}

	Frame.sendFB = CAN.endPacket();

	if (Frame.sendFB)
	{
		debug("Package sent");
		// pkgNum++;
		//! TODO reset pkg num to avoid overflow later
	}

	return Frame;
};

CANFrame MehrCan::onReceive(int packetSize)
{
	// Parse msg
	if (!CAN.packetExtended())
	{
		debug("Should be extended! Skipping;");
		return;
	}

	if (CAN.packetRtr())
	{
		// Remote transmission request, packet contains no data
		debug("RTR Request! Skipping;");
		return;
	}

	CANFrame Frame;
	Frame.id = CAN.packetId();
	Frame.len = packetSize;

	if (debugMode)
	{
		Serial.print("Got packet with id 0x");
		Serial.print(Frame.id, HEX);

		Serial.print(" and length ");
		Serial.println(Frame.len);
	}

	// Read data parts
	int i = 0;

	while (CAN.available())
	{
		Frame.data[i] = CAN.read();

		if (debugMode)
		{
			Serial.println(Frame.data[i], HEX);
		}

		i++;
	}

	// Frame filled, parse
	Frame = FrameCommander::parseFrame(Frame);

	if (Frame.buildFB)
	{
		// Process incoming frame
		Frame = processRxFrame(Frame);
	}

	return Frame;
};

CANFrame MehrCan::processRxFrame(CANFrame Frame)
{
	Frame.sendFB = 0; // Default false

	switch (Frame.command)
	{
	case CMD_PING:
		if (Frame.status == STATUS_REQUEST)
		{
			// Skip for now, we need to send our uid
			Frame.sendFB = 1;
			break;
		}

		if (Frame.len != 8)
		{
			break;
		}

		// Keep data address and type
		Device devices[0];

		for (int i = 0; i < 4; i++)
		{
			devices[0].id[i] = Frame.data[i];
		}

		devices[0].type[0] = Frame.data[6];
		devices[0].type[1] = Frame.data[7];

		Frame.sendFB = 1;
		break;

	case CMD_MFX_DISCOVERY:
		// Ignore requests
		if (Frame.status == STATUS_REQUEST)
		{
			Frame.sendFB = 1; // Its success
			break;
		}

		// Failed discovery
		if (Frame.len == 0)
		{
			break;
		}

		// Debug part with ask byte
		if (Frame.len > 6)
		{
			break;
		}

		if (Frame.len == 5)
		{
			// Loco found, try to bind
			/*
			Loco loco = MFXBind(Frame);

			if (loco.sid > 0) {
			  locos[0] = loco;  // Add to available locos
			  Frame.sendFB = 1;
			}
			*/
		}
		break;

	case CMD_MFX_BIND:
		// Ignore requests
		if (Frame.status == STATUS_REQUEST)
		{
			Frame.sendFB = 1; // Its success
			break;
		}

		Frame.sendFB = 1;
		break;
	}

	// Check if response for pending request
	if (Frame.status == STATUS_RESPONSE && cmd.waitForResponse && Frame.command == cmd.tx.command)
	{
		cmd.waitForResponse = 0;
		cmd.rx = Frame;
	}

	return Frame;
};

Loco MehrCan::MFXBind(CANFrame Frame)
{
	Command cmd;
	cmd.waitForResponse = DEFAULT_TIMEOUT;

	// Create tx frame, its a bit special with dyn data
	cmd.tx.prio = PRIO_SYSTEM;
	cmd.tx.command = CMD_MFX_BIND;
	cmd.tx.status = STATUS_REQUEST;
	cmd.tx.len = 6;
	cmd.tx.buildFB = 1;

	// Add address from discovery
	Loco loco;

	for (int i = 0; i < 4; i++)
	{
		cmd.tx.data[i] = Frame.data[i];
		loco.uid[i] = Frame.data[i];
	}

	// Add new local loco address
	loco.sid = locoStore.getNewAddress();
	cmd.tx.data[4] = 0x00;
	cmd.tx.data[5] = loco.sid;

	cmd.tx = canSend(cmd.tx);

	if (cmd.tx.sendFB)
	{
		debug("New loco bound");
		return loco;
	}

	// Return empty loco on fail
	loco.sid = 0x00;
	debug("Failed to bind loco!");

	return loco;
};

uint16_t MehrCan::prepareUID()
{
	// UID: 0x4711EEF0
	// High Word: 0x4711
	// Low Word: 0xEEF0
	// Hash-Berechnung (XOR):
	// 0x4711 XOR 0xEEF0 = 0xA9E1

	// Get 16bit High and Low
	uint16_t low = (uint16_t)(canUID & 0xFFFF);
	uint16_t high = (uint16_t)((canUID >> 16) & 0xFFFF);

	// XOR
	uint16_t canID = low ^ high;

	return canID;
};

uint16_t MehrCan::addPkgNum(uint16_t UID)
{
	if (oldStyle)
	{
		// Set 110er MASK to mark pkg
		UID = UID & 0xFF7F | 0x0300;
	}
	else
	{
		// Serial.println(UID, BIN);
		//  Remove area for pkg num
		UID &= ~((uint16_t)0b111 << 7);
		// Serial.println(UID, BIN);
		//  Set num
		UID |= (uint16_t)(pkgNum << 7);
		// Serial.println(UID, BIN);
	}

	return UID;
};

void MehrCan::debug(String text)
{
	Serial.println(text);
};

void MehrCan::dumpData(unsigned long data, int type = 0)
{
	if (!type)
	{
		// Dump both, hex and bin
		Serial.println(data, HEX);
		Serial.println(data, BIN);
		return;
	}

	Serial.println(data, type);
};

void MehrCan::dumpFrame(CANFrame Frame)
{
	dumpData(Frame.id);

	for (int i = 0; i < Frame.len; i++)
	{
		dumpData(Frame.data[i], HEX);
	}
};