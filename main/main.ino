#include "MehrCan.h"

// Data
int trackStatus = 0;

// Loco data
MehrCan mcan;
LocoStore locoStore;
Loco *locos;

void setup()
{
	Serial.begin(9600);

	// Init MehrCan and check success
	if (mcan.begin())
	{
		loadLocos(); // Load all locos from store
		// Initial track handling
		performTrackInit();
	}
	else
	{
		// Stop track
		trackStatus = TRACK_STATUS_STOP;
	}
}

void loop()
{
	// trackStatus = TRACK_STATUS_STOP;

	// Process input from serial
	if (Serial.available())
	{
		processSerialInput();
	}

	// Mehr can loop processing
	mcan.loop();

	// Default actions for rail track
	performTrackLoopAction();
}

/*
 * MAIN TRACK DATA
 */
void loadLocos()
{
	locos = mcan.loadLocos();
}

/*
 * Rail fun here
 * Keep in mind to use internal time for delays to avoid pausing can
 */
// Track data
int trackPosPinA = 2; // Station invisible
int trackPosPinB = 4; // Station village

int step = 0;
Loco trackLoco = locos[0];
unsigned long trackTimer;
unsigned long trackTimeout;
int avgTrackTime = 0;

// Track detection
int trackPosLockA;
int trackPosLockB;
int trackPosLockATimeout = TRACK_SENSOR_DEFAULT_TIMEOUT;
int trackPosLockBTimeout = TRACK_SENSOR_DEFAULT_TIMEOUT;

// Track infos
String trackMsg;

void performTrackInit()
{
	// Set detection pins
	pinMode(trackPosPinA, INPUT_PULLUP);
	pinMode(trackPosPinB, INPUT_PULLUP);

	delay(100);

	// Read inital pos
	trackPosLockA = digitalRead(trackPosPinA);
	trackPosLockB = digitalRead(trackPosPinB);
	mcan.debug("Track pos A: " + (String)trackPosLockA);
	mcan.debug("Track pos B: " + (String)trackPosLockB);

	// Check loco at step 0 pos
	// At default it will stop at village pos and go back to A by loco search
	if (trackPosLockA)
	{
		trackStatus = TRACK_STATUS_LOCO_SEARCH;
	}

	// Power rail
	mcan.runCommand(COMMAND_SYSTEM_GO);

	delay(200);

	// Sending loco stop
	uint8_t locoData[2];
	locoData[0] = 0x00;
	locoData[1] = 0x00;
	mcan.runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2);
}

void performTrackLoopAction()
{
	// Update sensor pos infos
	updateTrackInfos();

	String msg = "";

	// Skip if pending command
	if (mcan.cmd.waitForResponse > 0)
	{
		return;
	}

	if (trackStatus > 0)
	{
		switch (trackStatus)
		{
		case TRACK_STATUS_STOP:
			// ESTOP
			return;

		case TRACK_STATUS_SHUTDOWN:
			// Shutown in progress
			if (shutdownTrack())
			{
				return;
			}
			break;

		case TRACK_STATUS_LOCO_SEARCH:
			// Search loco
			locoSearch();
			return;
		}
	}

	uint8_t locoData[4];

	switch (step)
	{
	case 0:
		msg = "Turn on light.";
		locoData[0] = 0x00;
		locoData[1] = 0x01;
		mcan.runLocoCommand(COMMAND_LOCO_FUNC, trackLoco, locoData, 2);

		msg += " Set loco direction.";
		locoData[0] = LOCO_DIR_F;

		if (mcan.runLocoCommand(COMMAND_LOCO_DIR, trackLoco, locoData, 1))
		{
			step++;
		}
		break;

	case 1:
		msg = "Start moving.";
		locoData[0] = 0x00;
		locoData[1] = 0x30;

		if (mcan.runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2))
		{
			msg += " Starting track timer.";
			trackTimer = millis();
			step++;
		}
		break;

	case 2:
		if ((millis() - trackTimer) < 2000)
		{
			msg += "Keep moving...Slow...";
			break;
		}

		msg += "Speed up to final speed. ";
		locoData[0] = 0x00;
		locoData[1] = 0xFF;

		if (mcan.runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2))
		{
			step++;
		}
		break;

	case 3:
		if (!trackPosLockB)
		{
			// Instant stop step if reached village
			step++;
			break;
		}

		if ((millis() - trackTimer) > 12000)
		{
			// Reached time
			locoData[0] = 0x00;
			locoData[1] = 0x50;
			mcan.runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2);
			step++;
		}
		break;

	case 4:
		if (!trackPosLockB)
		{
			// Stop at village
			locoData[0] = 0x00;
			locoData[1] = 0x00;
			mcan.runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2);
			step++;
			trackTimeout = (millis() + random(5000, 10000));
		}
		break;

	case 5:
		// Wait at pos
		if (millis() < trackTimeout)
		{
			msg += "Waiting at pos B...";
			break;
		}

		msg += "Switch direction backwards ";
		locoData[0] = LOCO_DIR_R;

		if (mcan.runLocoCommand(COMMAND_LOCO_DIR, trackLoco, locoData, 1))
		{
			step++;
		}
		break;

	case 6:
		msg += "Start moving. ";
		locoData[0] = 0x00;
		locoData[1] = 0x30;

		if (mcan.runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2))
		{
			// Set new timeout to speed up
			trackTimeout = (millis() + 2000);
			step++;
		}
		break;

	case 7:
		if (millis() < trackTimeout)
		{
			msg += "Keep moving...Slow...";
			break;
		}

		msg += "Speed up to final speed. ";

		locoData[0] = 0x00;
		locoData[1] = 0xFF;

		if (mcan.runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2))
		{
			// Add timeout for fast move
			trackTimeout = (millis() + 10000);
			step++;
		}
		break;

	case 8:
		if (!trackPosLockA)
		{
			// Reached pos, skip slowdown
			step++;
			break;
		}

		if (millis() < trackTimeout)
		{
			msg += "Keep moving fast...";
			break;
		}

		msg += "Slowdown...";

		locoData[0] = 0x00;
		locoData[1] = 0x50;

		if (mcan.runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2))
		{
			step++;
		}
		break;

	case 9:
		if (!trackPosLockA)
		{
			msg += "Move finished... Stop. ";
			locoData[0] = 0x00;
			locoData[1] = 0x00;

			if (mcan.runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2))
			{
				msg += "Track loop finished. Waiting...";
				trackTimeout = (millis() + random(4000, 12000));
				step++;
			}
		}
		break;

	case 10:
		if (millis() < trackTimeout)
		{
			msg += "Keep waiting...";
			break;
		}

		step = 0;
		break;
	}

	msg += "Current step: " + (String)step;

	// Check and update/display msg
	if (trackMsg != msg)
	{
		mcan.debug(msg);
		trackMsg = msg;
	}
}

void locoSearch()
{
	uint8_t locoData[2];

	switch (step)
	{
	case 0:
		// Set loco dir backwards
		locoData[0] = LOCO_DIR_R;
		mcan.runLocoCommand(COMMAND_LOCO_DIR, trackLoco, locoData, 1);
		step++;
		break;

	case 1:
		// Start moving slow
		locoData[0] = 0x00;
		locoData[1] = 0x10;
		mcan.runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2);
		step++;
		break;

	case 2:
		// Wait and check for pos info
		if (!trackPosLockB)
		{
			// Found at village pos, keep gooing backwards
			break;
		}

		if (!trackPosLockA)
		{
			// Found stop
			locoData[0] = 0x00;
			locoData[1] = 0x00;
			mcan.runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2);

			step = 0;
			trackStatus = TRACK_STATUS_OK;
			break;
		}
	}
}

bool shutdownTrack()
{
	// Do graceful shutdown at step 0
	if (step == 5)
	{
		// Stop loco
		uint8_t locoData[2];
		locoData[0] = 0x00;
		locoData[1] = 0x00;
		mcan.runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2);

		// Turn off light
		mcan.runLocoCommand(COMMAND_LOCO_FUNC, trackLoco, locoData, 2);

		// Shutdown finished
		trackStatus = TRACK_STATUS_STOP;
		return true;
	}

	mcan.debug("Waiting for step 5");
	return false;
}

void updateTrackInfos()
{
	// Use loop timer, will be updatet by loop func
	int diff = (millis() - mcan.loopTime);

	if (trackPosLockATimeout)
	{
		if (trackPosLockATimeout - diff < 0)
		{
			trackPosLockATimeout = 0;
		}
		else
		{
			trackPosLockATimeout -= diff;
		}
	}

	if (trackPosLockBTimeout)
	{
		if (trackPosLockBTimeout - diff < 0)
		{
			trackPosLockBTimeout = 0;
		}
		else
		{
			trackPosLockBTimeout -= diff;
		}
	}

	if (trackPosLockA != digitalRead(trackPosPinA) && (!trackPosLockATimeout || !digitalRead(trackPosPinA)))
	{
		trackPosLockA = digitalRead(trackPosPinA);
		trackPosLockATimeout = TRACK_SENSOR_DEFAULT_TIMEOUT;
		mcan.debug("Updating sensor A pos to: " + (String)trackPosLockA);
	}

	if (trackPosLockB != digitalRead(trackPosPinB) && (!trackPosLockBTimeout || !digitalRead(trackPosPinB)))
	{
		trackPosLockB = digitalRead(trackPosPinB);
		trackPosLockBTimeout = TRACK_SENSOR_DEFAULT_TIMEOUT;
		mcan.debug("Updating sensor B pos to: " + (String)trackPosLockB);
	}
}
/*
 * END
 */

/*
 * Additional Serial input stuff for debug and power off
 */
void processSerialInput()
{
	int command = Serial.parseInt();
	uint8_t data[4];

	switch (command)
	{
	case 0:
		mcan.debug("Emergency STOP!");
		trackStatus = 1;
		mcan.runCommand(COMMAND_SYSTEM_STOP);
		break;

	case 1:
		mcan.debug("System GO");
		trackStatus = TRACK_STATUS_STOP;
		step = 0;
		mcan.runCommand(COMMAND_SYSTEM_GO);
		break;

	case 2:
		mcan.debug("Track shutdown");
		trackStatus = TRACK_STATUS_SHUTDOWN;
		break;
	}

	while (Serial.available())
	{
		Serial.read();
	}
}