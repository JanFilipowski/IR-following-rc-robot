#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <ctype.h>
#include <string.h>

#include "protocol.h"

const int LEFT_IN1  = 6;
const int LEFT_IN2  = 5;
const int RIGHT_IN1 = 10;
const int RIGHT_IN2 = 9;

const byte CE_PIN  = 7;
const byte CSN_PIN = 8;

RF24 radio(CE_PIN, CSN_PIN);

Packet data;

uint8_t currentMode = MODE_MANUAL;
uint8_t lastMode = 255;

unsigned long lastManualPacketTime = 0;
const unsigned long SIGNAL_TIMEOUT = 300;

bool debugEnabled = false;

char serialBuf[32];
uint8_t serialLen = 0;

static void setLeftTrack(int dir) {
  // dir: -1 = tyl, 0 = stop, +1 = przod
  if (dir > 0) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
  } else if (dir < 0) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
  } else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
  }
}

static void setRightTrack(int dir) {
  if (dir > 0) {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
  } else if (dir < 0) {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
  } else {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
  }
}

static void stopMotors() {
  setLeftTrack(0);
  setRightTrack(0);
}

static void runManualMode(const Packet &p) {
  const bool leftFwd  = (p.buttons_bitmask & BTN_MASK_LEFT_FWD)  != 0;
  const bool leftRev  = (p.buttons_bitmask & BTN_MASK_LEFT_REV)  != 0;
  const bool rightFwd = (p.buttons_bitmask & BTN_MASK_RIGHT_FWD) != 0;
  const bool rightRev = (p.buttons_bitmask & BTN_MASK_RIGHT_REV) != 0;

  int leftDir = 0;
  int rightDir = 0;

  if (leftFwd && !leftRev)      leftDir = 1;
  else if (!leftFwd && leftRev) leftDir = -1;
  else                          leftDir = 0;

  if (rightFwd && !rightRev)      rightDir = 1;
  else if (!rightFwd && rightRev) rightDir = -1;
  else                            rightDir = 0;

  setLeftTrack(leftDir);
  setRightTrack(rightDir);

  if (debugEnabled) {
    Serial.print("MANUAL BUTTONS=0x");
    if (p.buttons_bitmask < 16) Serial.print('0');
    Serial.print(p.buttons_bitmask, HEX);
    Serial.print(" LEFT=");
    Serial.print(leftDir);
    Serial.print(" RIGHT=");
    Serial.println(rightDir);
  }
}

static void runAutoPlaceholder() {
  stopMotors();
}

static void printStatus() {
  Serial.print("STATUS MODE=");
  Serial.print(currentMode == MODE_MANUAL ? "MANUAL" : "AUTO");
  Serial.print(" LAST_BUTTONS=0x");
  if (data.buttons_bitmask < 16) Serial.print('0');
  Serial.println(data.buttons_bitmask, HEX);
}

static void toUpperInPlace(char *s) {
  while (*s) {
    *s = toupper((unsigned char)*s);
    ++s;
  }
}

static void processCommand(char *cmd) {
  toUpperInPlace(cmd);

  if (strcmp(cmd, "DEBUG ON") == 0) {
    debugEnabled = true;
    Serial.println("DEBUG=ON");
  } else if (strcmp(cmd, "DEBUG OFF") == 0) {
    debugEnabled = false;
    Serial.println("DEBUG=OFF");
  } else if (strcmp(cmd, "DEBUG") == 0) {
    debugEnabled = !debugEnabled;
    Serial.print("DEBUG=");
    Serial.println(debugEnabled ? "ON" : "OFF");
  } else if (strcmp(cmd, "STATUS") == 0) {
    printStatus();
  } else if (strcmp(cmd, "HELP") == 0) {
    Serial.println("CMDS: DEBUG ON | DEBUG OFF | DEBUG | STATUS | HELP");
  } else {
    Serial.print("UNKNOWN CMD: ");
    Serial.println(cmd);
  }
}

static void handleSerialCommands() {
  while (Serial.available()) {
    const char c = Serial.read();

    if (c == '\r' || c == '\n') {
      if (serialLen > 0) {
        serialBuf[serialLen] = '\0';
        processCommand(serialBuf);
        serialLen = 0;
      }
    } else if (serialLen < sizeof(serialBuf) - 1) {
      serialBuf[serialLen++] = c;
    }
  }
}

void setup() {
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  stopMotors();

  Serial.begin(115200);

  if (!radio.begin()) {
    Serial.println("Blad: radio nie odpowiada");
    while (true) {}
  }

  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(RADIO_CHANNEL);
  radio.openReadingPipe(1, RADIO_ADDRESS);
  radio.startListening();

  data.mode = MODE_MANUAL;
  data.buttons_bitmask = 0;

  Serial.println("RX gotowy");
  Serial.println("DEBUG domyslnie OFF");
}

void loop() {
  handleSerialCommands();

  if (radio.available()) {
    radio.read(&data, sizeof(data));

    currentMode = data.mode;

    if (currentMode != lastMode) {
      lastMode = currentMode;
      if (debugEnabled) {
        Serial.print("TRYB -> ");
        Serial.println(currentMode == MODE_MANUAL ? "MANUAL" : "AUTO");
      }
    }

    if (currentMode == MODE_MANUAL) {
      lastManualPacketTime = millis();
      runManualMode(data);
    }
  }

  if (currentMode == MODE_AUTO) {
    runAutoPlaceholder();
  } else {
    if (millis() - lastManualPacketTime > SIGNAL_TIMEOUT) {
      stopMotors();
    }
  }
}