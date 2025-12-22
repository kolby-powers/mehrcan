#include <CAN.h>
#include "main.h"
#include "LocoStore.h"
#include "FrameCommander.h"

// Settings
uint32_t canUID = 0x4711EEF0;  // CAN UID of arduino to use (copy of real cs2 start 0x47 with serial at the end)
uint16_t pkgNum = 0b1;         // Counter for package num
bool debugMode = true;
bool oldStyle = true;

// Data
bool initSuccess;
unsigned long loopTime;
int trackStatus = 0;
uint8_t broadcastAdr[4] = { 0x00, 0x00, 0x00, 0x00 };

Command cmd;
Device devices[1];
LocoStore locoStore;
Loco locos[1];

void setup() {
  Serial.begin(9600);

  CAN.setClockFrequency(8E6);

  debug("Can INIT...");

  // Start can with märklin 250k speed
  while (!CAN.begin(250E3)) {
    debug("Failed to init CAN! Retry...");
    delay(500);
  }

  debug("SUCCESS! Init at 250k");

  // Init Märklin digital box
  initSuccess = initMBox();

  if (initSuccess) {
    loadLocos();  // Load all locos from store
    // Initial track handling
    performTrackInit();
  } else {
    // Stop track
    trackStatus = TRACK_STATUS_STOP;
  }

  loopTime = millis();
}

void loop() {
  //trackStatus = TRACK_STATUS_STOP;

  // Process input from serial
  if (Serial.available()) {
    processSerialInput();
  }

  // Check for incoming pkg
  int packetSize = CAN.parsePacket();
  long pkgID = CAN.packetId();

  if (packetSize || pkgID > 0) {
    onReceive(packetSize);
  }

  // Default actions for rail track
  performTrackLoopAction();

  // Command timeout update
  unsigned long diff = (millis() - loopTime);
  loopTime = millis();

  if (cmd.waitForResponse > 0) {
    if (cmd.waitForResponse - diff < 0) {
      cmd.waitForResponse = 0;
      debug("Pkg timeout...");
      dumpFrame(cmd.tx);
    } else {
      cmd.waitForResponse -= diff;
    }
  }
}

/*
 * Rail fun here
 * Keep in mind to use internal time for delays to avoid pausing can
 */
// Track data
int trackPosPinA = 2;  // Station invisible
int trackPosPinB = 4;  // Station village

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

void performTrackInit() {
  // Set detection pins
  pinMode(trackPosPinA, INPUT_PULLUP);
  pinMode(trackPosPinB, INPUT_PULLUP);

  delay(100);

  // Read inital pos
  trackPosLockA = digitalRead(trackPosPinA);
  trackPosLockB = digitalRead(trackPosPinB);
  debug("Track pos A: " + (String)trackPosLockA);
  debug("Track pos B: " + (String)trackPosLockB);

  // Check loco at step 0 pos
  // At default it will stop at village pos and go back to A by loco search
  if (trackPosLockA) {
    trackStatus = TRACK_STATUS_LOCO_SEARCH;
  }

  // Power rail
  runCommand(COMMAND_SYSTEM_GO);

  delay(200);

  // Sending loco stop
  uint8_t locoData[2];
  locoData[0] = 0x00;
  locoData[1] = 0x00;
  runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2);
}

void performTrackLoopAction() {
  // Update sensor pos infos
  updateTrackInfos();

  String msg = "";

  // Skip if pending command
  if (cmd.waitForResponse > 0) {
    return;
  }

  if (trackStatus > 0) {
    switch (trackStatus) {
      case TRACK_STATUS_STOP:
        // ESTOP
        return;

      case TRACK_STATUS_SHUTDOWN:
        // Shutown in progress
        if (shutdownTrack()) {
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

  switch (step) {
    case 0:
      msg = "Turn on light.";
      locoData[0] = 0x00;
      locoData[1] = 0x01;
      runLocoCommand(COMMAND_LOCO_FUNC, trackLoco, locoData, 2);

      msg += " Set loco direction.";
      locoData[0] = LOCO_DIR_F;

      if (runLocoCommand(COMMAND_LOCO_DIR, trackLoco, locoData, 1)) {
        step++;
      }
      break;

    case 1:
      msg = "Start moving.";
      locoData[0] = 0x00;
      locoData[1] = 0x30;

      if (runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2)) {
        msg += " Starting track timer.";
        trackTimer = millis();
        step++;
      }
      break;

    case 2:
      if ((millis() - trackTimer) < 2000) {
        msg += "Keep moving...Slow...";
        break;
      }

      msg += "Speed up to final speed. ";
      locoData[0] = 0x00;
      locoData[1] = 0xFF;

      if (runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2)) {
        step++;
      }
      break;

    case 3:
      if (!trackPosLockB) {
        // Instant stop step if reached village
        step++;
        break;
      }

      if ((millis() - trackTimer) > 12000) {
        // Reached time
        locoData[0] = 0x00;
        locoData[1] = 0x50;
        runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2);
        step++;
      }
      break;

    case 4:
      if (!trackPosLockB) {
        // Stop at village
        locoData[0] = 0x00;
        locoData[1] = 0x00;
        runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2);
        step++;
        trackTimeout = (millis() + random(5000, 10000));
      }
      break;

    case 5:
      // Wait at pos
      if (millis() < trackTimeout) {
        msg += "Waiting at pos B...";
        break;
      }

      msg += "Switch direction backwards ";
      locoData[0] = LOCO_DIR_R;

      if (runLocoCommand(COMMAND_LOCO_DIR, trackLoco, locoData, 1)) {
        step++;
      }
      break;

    case 6:
      msg += "Start moving. ";
      locoData[0] = 0x00;
      locoData[1] = 0x30;

      if (runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2)) {
        // Set new timeout to speed up
        trackTimeout = (millis() + 2000);
        step++;
      }
      break;

    case 7:
      if (millis() < trackTimeout) {
        msg += "Keep moving...Slow...";
        break;
      }

      msg += "Speed up to final speed. ";

      locoData[0] = 0x00;
      locoData[1] = 0xFF;

      if (runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2)) {
        // Add timeout for fast move
        trackTimeout = (millis() + 10000);
        step++;
      }
      break;

    case 8:
      if (!trackPosLockA) {
        // Reached pos, skip slowdown
        step++;
        break;
      }

      if (millis() < trackTimeout) {
        msg += "Keep moving fast...";
        break;
      }

      msg += "Slowdown...";

      locoData[0] = 0x00;
      locoData[1] = 0x50;

      if (runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2)) {
        step++;
      }
      break;

    case 9:
      if (!trackPosLockA) {
        msg += "Move finished... Stop. ";
        locoData[0] = 0x00;
        locoData[1] = 0x00;

        if (runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2)) {
          msg += "Track loop finished. Waiting...";
          trackTimeout = (millis() + random(4000, 12000));
          step++;
        }
      }
      break;

    case 10:
      if (millis() < trackTimeout) {
        msg += "Keep waiting...";
        break;
      }

      step = 0;
      break;
  }

  msg += "Current step: " + (String)step;

  // Check and update/display msg
  if (trackMsg != msg) {
    debug(msg);
    trackMsg = msg;
  }
}

void locoSearch() {
  uint8_t locoData[2];

  switch (step) {
    case 0:
      // Set loco dir backwards
      locoData[0] = LOCO_DIR_R;
      runLocoCommand(COMMAND_LOCO_DIR, trackLoco, locoData, 1);
      step++;
      break;

    case 1:
      // Start moving slow
      locoData[0] = 0x00;
      locoData[1] = 0x10;
      runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2);
      step++;
      break;

    case 2:
      // Wait and check for pos info
      if (!trackPosLockB) {
        // Found at village pos, keep gooing backwards
        break;
      }

      if (!trackPosLockA) {
        // Found stop
        locoData[0] = 0x00;
        locoData[1] = 0x00;
        runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2);

        step = 0;
        trackStatus = TRACK_STATUS_OK;
        break;
      }
  }
}

bool shutdownTrack() {
  // Do graceful shutdown at step 0
  if (step == 5) {
    // Stop loco
    uint8_t locoData[2];
    locoData[0] = 0x00;
    locoData[1] = 0x00;
    runLocoCommand(COMMAND_LOCO_SPEED, trackLoco, locoData, 2);

    // Turn off light
    runLocoCommand(COMMAND_LOCO_FUNC, trackLoco, locoData, 2);

    // Shutdown finished
    trackStatus = TRACK_STATUS_STOP;
    return true;
  }

  debug("Waiting for step 5");
  return false;
}

void updateTrackInfos() {
  // Use loop timer, will be updatet by loop func
  int diff = (millis() - loopTime);

  if (trackPosLockATimeout) {
    if (trackPosLockATimeout - diff < 0) {
      trackPosLockATimeout = 0;
    } else {
      trackPosLockATimeout -= diff;
    }
  }

  if (trackPosLockBTimeout) {
    if (trackPosLockBTimeout - diff < 0) {
      trackPosLockBTimeout = 0;
    } else {
      trackPosLockBTimeout -= diff;
    }
  }

  if (trackPosLockA != digitalRead(trackPosPinA) && (!trackPosLockATimeout || !digitalRead(trackPosPinA))) {
    trackPosLockA = digitalRead(trackPosPinA);
    trackPosLockATimeout = TRACK_SENSOR_DEFAULT_TIMEOUT;
    debug("Updating sensor A pos to: " + (String)trackPosLockA);
  }

  if (trackPosLockB != digitalRead(trackPosPinB) && (!trackPosLockBTimeout || !digitalRead(trackPosPinB))) {
    trackPosLockB = digitalRead(trackPosPinB);
    trackPosLockBTimeout = TRACK_SENSOR_DEFAULT_TIMEOUT;
    debug("Updating sensor B pos to: " + (String)trackPosLockB);
  }
}
/*
 * END
 */

bool initMBox() {
  // Init is called at arduino startup, if box is responding, send stop just in case arduino crashed before
  // Send bootloader start cmd
  // Use CANFrame without command because theres no response for this
  CANFrame frame = FrameCommander::buildFrame(COMMAND_BOOTLOADER_CAN);

  if (frame.buildFB) {
    frame = canSend(frame);

    if (frame.sendFB) {
      // Wait
      delay(1000);

      return runCommand(COMMAND_PING);
    }
  }

  return false;
}

void loadLocos() {
  int cntAvailable = locoStore.getCount();

  for (int i = 0; i < cntAvailable; i++) {
    locos[i] = locoStore.load(i + 1);  // Store starts at 1
  }
}

Loco MFXBind(CANFrame Frame) {
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

  for (int i = 0; i < 4; i++) {
    cmd.tx.data[i] = Frame.data[i];
    loco.uid[i] = Frame.data[i];
  }

  // Add new local loco address
  loco.sid = locoStore.getNewAddress();
  cmd.tx.data[4] = 0x00;
  cmd.tx.data[5] = loco.sid;

  cmd.tx = canSend(cmd.tx);

  if (cmd.tx.sendFB) {
    debug("New loco bound");
    return loco;
  }

  // Return empty loco on fail
  loco.sid = 0x00;
  debug("Failed to bind loco!");

  return loco;
}

bool runCommand(int command) {
  cmd.intCmd = command;
  cmd.tx = FrameCommander::buildFrame(command);
  cmd.waitForResponse = DEFAULT_TIMEOUT;  // Default true

  // Check build
  if (!cmd.tx.buildFB) {
    return false;
  }

  // Send command
  cmd.tx = canSend(cmd.tx);

  if (!cmd.tx.sendFB) {
    return false;
  }

  return true;
}

bool runLocoCommand(int locoCommand, Loco loco, uint8_t data[4], int additionalLen) {
  // Create base frame
  CANFrame Frame = FrameCommander::buildFrame(locoCommand);

  if (!Frame.buildFB) {
    debug("Failed to build Loco control Frame!");
    return false;
  }

  // Add loco address
  Frame.data[0] = 0x00;
  Frame.data[1] = 0x00;
  Frame.data[2] = 0x40;  // MFX address space
  Frame.data[3] = loco.sid;

  if (additionalLen) {
    for (int i = 0; i < additionalLen; i++) {
      Frame.data[(4 + i)] = data[i];
    }
  }

  Frame.len = (4 + additionalLen);

  // Create command and wait for fb
  Command cmd;
  cmd.waitForResponse = DEFAULT_TIMEOUT;
  cmd.intCmd = locoCommand;
  cmd.tx = canSend(Frame);

  if (cmd.tx.sendFB) {
    return true;
  }

  // Skip waiting for response
  cmd.waitForResponse = 0;
  debug("Failed to execute Loco command!");

  return false;
}

CANFrame processRxFrame(CANFrame Frame) {
  Frame.sendFB = 0;  // Default false

  switch (Frame.command) {
    case CMD_PING:
      if (Frame.status == STATUS_REQUEST) {
        // Skip for now, we need to send our uid
        Frame.sendFB = 1;
        break;
      }

      if (Frame.len != 8) {
        break;
      }

      // Keep data address and type
      Device devices[0];

      for (int i = 0; i < 4; i++) {
        devices[0].id[i] = Frame.data[i];
      }

      devices[0].type[0] = Frame.data[6];
      devices[0].type[1] = Frame.data[7];

      Frame.sendFB = 1;
      break;

    case CMD_MFX_DISCOVERY:
      // Ignore requests
      if (Frame.status == STATUS_REQUEST) {
        Frame.sendFB = 1;  // Its success
        break;
      }

      // Failed discovery
      if (Frame.len == 0) {
        break;
      }

      // Debug part with ask byte
      if (Frame.len > 6) {
        break;
      }

      if (Frame.len == 5) {
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
      if (Frame.status == STATUS_REQUEST) {
        Frame.sendFB = 1;  // Its success
        break;
      }

      Frame.sendFB = 1;
      break;
  }

  // Check if response for pending request
  if (Frame.status == STATUS_RESPONSE && cmd.waitForResponse && Frame.command == cmd.tx.command) {
    cmd.waitForResponse = 0;
    cmd.rx = Frame;
  }

  return Frame;
}

CANFrame canSend(CANFrame frame) {
  // If a command waiting for response store frame and wait
  if (cmd.waitForResponse) {
  }

  debug("Sending request");

  uint32_t canID = 0;

  canID |= (uint32_t)frame.prio << 25;     // Prio 2+2Bits at the start
  canID |= (uint32_t)frame.command << 17;  // 8Bit command
  canID |= (uint32_t)frame.status << 16;   // 1Bit status 0 = request / 1 = answer

  // 00100000110001010100111100001
  // Prio|Command |R|Hash
  // 0010|00001100|0|1010100111100001
  // 4   |8       |1|16

  // Add hash of uid at the end
  canID |= addPkgNum(prepareUID());
  //canID |= random(0x10000) & 0xff7f | 0x0300;

  // Add ID to frame
  frame.id = canID;

  if (debugMode) {
    Serial.println(canID, BIN);
    Serial.println(canID, HEX);
  }

  if (CAN.beginExtendedPacket(frame.id, frame.len)) {
    debug("Package created");
  }

  if (frame.len > 0) {
    for (int i = 0; i < frame.len; i++) {
      CAN.write(frame.data[i]);

      if (debugMode) {
        Serial.println(frame.data[i], HEX);
      }
    }
  }

  frame.sendFB = CAN.endPacket();

  if (frame.sendFB) {
    debug("Package sent");
    //pkgNum++;
    //!TODO reset pkg num to avoid overflow later
  }

  return frame;
}

void onReceive(int packetSize) {
  // Parse msg
  if (!CAN.packetExtended()) {
    debug("Should be extended! Skipping;");
    return;
  }

  if (CAN.packetRtr()) {
    // Remote transmission request, packet contains no data
    debug("RTR Request! Skipping;");
    return;
  }

  CANFrame Frame;
  Frame.id = CAN.packetId();
  Frame.len = packetSize;

  if (debugMode) {
    Serial.print("Got packet with id 0x");
    Serial.print(Frame.id, HEX);

    Serial.print(" and length ");
    Serial.println(Frame.len);
  }

  // Read data parts
  int i = 0;

  while (CAN.available()) {
    Frame.data[i] = CAN.read();

    if (debugMode) {
      Serial.println(Frame.data[i], HEX);
    }

    i++;
  }

  // Frame filled, parse
  Frame = FrameCommander::parseFrame(Frame);

  if (Frame.buildFB) {
    // Process incoming frame
    Frame = processRxFrame(Frame);
  }
}

void processSerialInput() {
  int command = Serial.parseInt();
  uint8_t data[4];

  switch (command) {
    case 0:
      debug("Emergency STOP!");
      trackStatus = 1;
      runCommand(COMMAND_SYSTEM_STOP);
      break;

    case 1:
      debug("System GO");
      trackStatus = TRACK_STATUS_STOP;
      step = 0;
      runCommand(COMMAND_SYSTEM_GO);
      break;

    case 2:
      debug("Track shutdown");
      trackStatus = TRACK_STATUS_SHUTDOWN;
      break;
  }

  while (Serial.available()) {
    Serial.read();
  }
}

void debug(String text) {
  if (debugMode) {
    Serial.println(text);
  }
}

void dumpFrame(CANFrame Frame) {
  Serial.println(Frame.id, HEX);
  Serial.println(Frame.id, BIN);

  for (int i = 0; i < Frame.len; i++) {
    Serial.println(Frame.data[i], HEX);
  }
}

uint16_t prepareUID() {
  //UID: 0x4711EEF0
  //High Word: 0x4711
  //Low Word: 0xEEF0
  //Hash-Berechnung (XOR):
  //0x4711 XOR 0xEEF0 = 0xA9E1

  // Get 16bit High and Low
  uint16_t low = (uint16_t)(canUID & 0xFFFF);
  uint16_t high = (uint16_t)((canUID >> 16) & 0xFFFF);

  // XOR
  uint16_t canID = low ^ high;

  return canID;
}

uint16_t addPkgNum(uint16_t UID) {
  if (oldStyle) {
    UID = UID & 0xFF7F | 0x0300;
  } else {
    //Serial.println(UID, BIN);
    // Remove area for pkg num
    UID &= ~((uint16_t)0b111 << 7);
    //Serial.println(UID, BIN);
    // Set num
    UID |= (uint16_t)(pkgNum << 7);
    //Serial.println(UID, BIN);
  }

  return UID;
}