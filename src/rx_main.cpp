#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <ctype.h>
#include <string.h>

#include "protocol.h"

// TX pinout for button-only controller
static constexpr uint8_t PIN_LEFT_FWD  = 2;
static constexpr uint8_t PIN_LEFT_REV  = 3;
static constexpr uint8_t PIN_RIGHT_REV = 4;
static constexpr uint8_t PIN_RIGHT_FWD = 5;
static constexpr uint8_t PIN_MODE      = 6;   // toggle MANUAL/AUTO
static constexpr uint8_t PIN_BTN_A1    = 7;
static constexpr uint8_t PIN_BTN_A2    = 8;

static constexpr uint8_t PIN_RADIO_CE  = 9;
static constexpr uint8_t PIN_RADIO_CSN = 10;

static constexpr unsigned long DEBOUNCE_MS        = 25;
static constexpr unsigned long MODE_RETRY_MS      = 300;
static constexpr unsigned long STATE_RETRY_MS     = 120;
static constexpr unsigned long DRIVE_HEARTBEAT_MS = 180;

struct DebouncedButton {
  uint8_t pin;
  bool lastReading;
  bool stableState;
  unsigned long lastChangeMs;
};

RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);

DebouncedButton modeBtn      = {PIN_MODE,      HIGH, HIGH, 0};
DebouncedButton leftFwdBtn   = {PIN_LEFT_FWD,  HIGH, HIGH, 0};
DebouncedButton leftRevBtn   = {PIN_LEFT_REV,  HIGH, HIGH, 0};
DebouncedButton rightFwdBtn  = {PIN_RIGHT_FWD, HIGH, HIGH, 0};
DebouncedButton rightRevBtn  = {PIN_RIGHT_REV, HIGH, HIGH, 0};
DebouncedButton a1Btn        = {PIN_BTN_A1,    HIGH, HIGH, 0};
DebouncedButton a2Btn        = {PIN_BTN_A2,    HIGH, HIGH, 0};

uint8_t currentMode = MODE_MANUAL;
bool debugEnabled = false;

Packet desiredPacket{MODE_MANUAL, 0};
Packet lastAckedPacket{MODE_MANUAL, 0};
Packet lastAttemptedPacket{MODE_MANUAL, 0};

bool haveAckedPacket = false;
bool haveAttemptedPacket = false;

unsigned long lastAttemptMs = 0;
unsigned long lastAckMs = 0;

char serialBuf[32];
uint8_t serialLen = 0;

static void updateDebouncedButton(DebouncedButton &btn) {
  const bool reading = digitalRead(btn.pin);

  if (reading != btn.lastReading) {
    btn.lastReading = reading;
    btn.lastChangeMs = millis();
  }

  if ((millis() - btn.lastChangeMs) > DEBOUNCE_MS) {
    btn.stableState = reading;
  }
}

static bool debouncedPressed(DebouncedButton &btn) {
  const bool previousStable = btn.stableState;
  updateDebouncedButton(btn);
  return (previousStable == HIGH && btn.stableState == LOW);
}

static void updateStateButtons() {
  updateDebouncedButton(leftFwdBtn);
  updateDebouncedButton(leftRevBtn);
  updateDebouncedButton(rightFwdBtn);
  updateDebouncedButton(rightRevBtn);
  updateDebouncedButton(a1Btn);
  updateDebouncedButton(a2Btn);
}

static uint8_t buildButtonsBitmask() {
  uint8_t mask = 0;

  if (leftFwdBtn.stableState == LOW)  mask |= BTN_MASK_LEFT_FWD;
  if (leftRevBtn.stableState == LOW)  mask |= BTN_MASK_LEFT_REV;
  if (rightFwdBtn.stableState == LOW) mask |= BTN_MASK_RIGHT_FWD;
  if (rightRevBtn.stableState == LOW) mask |= BTN_MASK_RIGHT_REV;
  if (a1Btn.stableState == LOW)       mask |= BTN_MASK_A1;
  if (a2Btn.stableState == LOW)       mask |= BTN_MASK_A2;

  return mask;
}

static void buildDesiredPacket() {
  desiredPacket.mode = currentMode;
  desiredPacket.buttons_bitmask = buildButtonsBitmask();
}

static void printPacket(const char *prefix, const Packet &p, bool ok, const char *reason) {
  Serial.print(prefix);
  Serial.print(" MODE=");
  Serial.print(p.mode == MODE_MANUAL ? "MANUAL" : "AUTO");
  Serial.print(" BUTTONS=0x");
  if (p.buttons_bitmask < 16) Serial.print('0');
  Serial.print(p.buttons_bitmask, HEX);
  Serial.print(" TX=");
  Serial.print(ok ? "OK" : "FAIL");
  Serial.print(" REASON=");
  Serial.println(reason);
}

static void printStatus() {
  Packet snapshot = desiredPacket;

  Serial.print("STATUS MODE=");
  Serial.print(snapshot.mode == MODE_MANUAL ? "MANUAL" : "AUTO");
  Serial.print(" BUTTONS=0x");
  if (snapshot.buttons_bitmask < 16) Serial.print('0');
  Serial.print(snapshot.buttons_bitmask, HEX);

  Serial.print(" ACKED=");
  Serial.print(haveAckedPacket ? "YES" : "NO");

  if (haveAckedPacket) {
    Serial.print(" LAST_ACK_MODE=");
    Serial.print(lastAckedPacket.mode == MODE_MANUAL ? "MANUAL" : "AUTO");
    Serial.print(" LAST_ACK_BUTTONS=0x");
    if (lastAckedPacket.buttons_bitmask < 16) Serial.print('0');
    Serial.print(lastAckedPacket.buttons_bitmask, HEX);
  }

  Serial.println();
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
  pinMode(PIN_MODE, INPUT_PULLUP);
  pinMode(PIN_LEFT_FWD, INPUT_PULLUP);
  pinMode(PIN_LEFT_REV, INPUT_PULLUP);
  pinMode(PIN_RIGHT_FWD, INPUT_PULLUP);
  pinMode(PIN_RIGHT_REV, INPUT_PULLUP);
  pinMode(PIN_BTN_A1, INPUT_PULLUP);
  pinMode(PIN_BTN_A2, INPUT_PULLUP);

  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  Serial.begin(115200);

  if (!radio.begin()) {
    Serial.println("Blad: radio nie odpowiada");
    while (true) {}
  }

  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(RADIO_CHANNEL);
  radio.openWritingPipe(RADIO_ADDRESS);
  radio.stopListening();

  updateStateButtons();
  buildDesiredPacket();

  const bool ok = radio.write(&desiredPacket, sizeof(desiredPacket));
  haveAttemptedPacket = true;
  lastAttemptedPacket = desiredPacket;
  lastAttemptMs = millis();

  if (ok) {
    haveAckedPacket = true;
    lastAckedPacket = desiredPacket;
    lastAckMs = lastAttemptMs;
  }

  Serial.println("TX gotowy");
  Serial.print("INIT TX=");
  Serial.println(ok ? "OK" : "FAIL");
  Serial.println("DEBUG domyslnie OFF");
}

void loop() {
  handleSerialCommands();

  if (debouncedPressed(modeBtn)) {
    currentMode = (currentMode == MODE_MANUAL) ? MODE_AUTO : MODE_MANUAL;
    if (debugEnabled) {
      Serial.print("TRYB -> ");
      Serial.println(currentMode == MODE_MANUAL ? "MANUAL" : "AUTO");
    }
  }

  updateStateButtons();
  buildDesiredPacket();

  const bool modePending = !haveAckedPacket || (desiredPacket.mode != lastAckedPacket.mode);
  const bool buttonsPending = !haveAckedPacket || (desiredPacket.buttons_bitmask != lastAckedPacket.buttons_bitmask);

  bool shouldSend = false;
  const char *reason = "";

  if (modePending || buttonsPending) {
    const bool newDesiredPacket = !haveAttemptedPacket || !packetEquals(desiredPacket, lastAttemptedPacket);
    const unsigned long retryMs = modePending ? MODE_RETRY_MS : STATE_RETRY_MS;

    if (newDesiredPacket || (millis() - lastAttemptMs >= retryMs)) {
      shouldSend = true;
      if (modePending && buttonsPending) {
        reason = "MODE+CHANGE";
      } else if (modePending) {
        reason = "MODE";
      } else {
        reason = "CHANGE";
      }
    }
  } else if (currentMode == MODE_MANUAL && anyDriveButtons(desiredPacket.buttons_bitmask)) {
    if (millis() - lastAckMs >= DRIVE_HEARTBEAT_MS) {
      shouldSend = true;
      reason = "HEARTBEAT";
    }
  }

  if (shouldSend) {
    const bool ok = radio.write(&desiredPacket, sizeof(desiredPacket));
    haveAttemptedPacket = true;
    lastAttemptedPacket = desiredPacket;
    lastAttemptMs = millis();

    if (ok) {
      haveAckedPacket = true;
      lastAckedPacket = desiredPacket;
      lastAckMs = lastAttemptMs;
    }

    if (debugEnabled) {
      printPacket("TX", desiredPacket, ok, reason);
    }
  }
}