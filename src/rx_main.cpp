#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <ctype.h>
#include <string.h>

#include "protocol.h"

// ===== L298N =====
// Zgodnie z Twoim schematem:
// IN1, IN2, IN3, IN4 -> Arduino 7, 6, 5, 4
const int LEFT_IN1  = 5;
const int LEFT_IN2  = 4;
const int RIGHT_IN1 = 3;
const int RIGHT_IN2 = 2;

// ===== RADIO =====
// nRF24L01
const byte CE_PIN  = 9;
const byte CSN_PIN = 10;

// ===== BUZZER =====
const int BUZZER = 14;

// ===== RADIO =====
RF24 radio(CE_PIN, CSN_PIN);
Packet data;

// ===== TRYB =====
uint8_t currentMode = MODE_MANUAL;
uint8_t lastMode = 255;

// ===== FAILSAFE =====
unsigned long lastPacketTime = 0;
const unsigned long SIGNAL_TIMEOUT = 300;

// ===== DEBUG SERIAL =====
bool debugEnabled = false;
unsigned long lastNoPacketDebugTime = 0;

char serialBuf[32];
uint8_t serialLen = 0;

// ===== STAN GĄSIENIC =====
// -1 = tył, 0 = stop, 1 = przód
int currentLeftDir = 0;
int currentRightDir = 0;

// ===== PRZYCISKI =====
uint8_t lastButtonsMask = 0;

// ===== NUTY DO VALKYRIE / WAGNER =====
#define NOTE_FS3 185
#define NOTE_A3  220
#define NOTE_B3  247
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_FS4 370
#define NOTE_A4  440
#define NOTE_CS5 554

const int wagnerMelody[] = {
  NOTE_B3, NOTE_FS3, NOTE_B3, NOTE_D4, NOTE_B3, NOTE_D4, NOTE_B3, NOTE_D4, NOTE_FS4,
  NOTE_D4, NOTE_FS4, NOTE_D4, NOTE_FS4, NOTE_A4, NOTE_A3, NOTE_D4, NOTE_A3, NOTE_D4,
  NOTE_FS4, NOTE_B3, NOTE_D4, NOTE_B3, NOTE_D4, NOTE_FS4, NOTE_D4, NOTE_FS4, NOTE_D4,
  NOTE_FS4, NOTE_A4, NOTE_FS4, NOTE_A4, NOTE_FS4, NOTE_A4, NOTE_CS5, NOTE_CS4, NOTE_FS4,
  NOTE_CS4, NOTE_FS4, NOTE_A4
};

const float wagnerDurations[] = {
  4, 8, 4, 1.34, 1.34, 4, 8, 4, 1.34, 1.34,
  4, 8, 4, 1.34, 1.34, 4, 8, 4, 0.83, 4,
  4, 8, 4, 1.34, 1.34, 4, 8, 4, 1.34, 1.34,
  4, 8, 4, 1.34, 1.34, 4, 8, 4, 0.30
};

const int WAGNER_BPM = 84;
const int WAGNER_LEN = sizeof(wagnerMelody) / sizeof(wagnerMelody[0]);

bool wagnerEnabled = false;
bool wagnerPlaying = false;
int wagnerIndex = 0;
bool wagnerTonePhase = false;
unsigned long wagnerStepStart = 0;

// ===== BEEP COFANIA =====
unsigned long lastBeepTime = 0;
bool beepOn = false;

// =====================================================
// SILNIKI
// =====================================================

// Zgodnie ze schematem binarnym:
// 01 = przód
// 10 = tył
// 00 = stop
static void setTrackPins(int pin1, int pin2, int dir) {
  if (dir > 0) {
    // 01 = przód
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  } else if (dir < 0) {
    // 10 = tył
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  } else {
    // 00 = stop
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
}

static void applyMotors() {
  setTrackPins(LEFT_IN1, LEFT_IN2, currentLeftDir);
  setTrackPins(RIGHT_IN1, RIGHT_IN2, currentRightDir);
}

static void stopMotors() {
  currentLeftDir = 0;
  currentRightDir = 0;
  applyMotors();
}

// =====================================================
// VALKYRIE / WAGNER
// =====================================================

unsigned long getWagnerFullDurationMs(int index) {
  float full = (60000.0 / WAGNER_BPM) / wagnerDurations[index];

  if (full < 1.0) {
    full = 1.0;
  }

  return (unsigned long)full;
}

void stopWagner() {
  if (wagnerPlaying) {
    noTone(BUZZER);
  }

  wagnerPlaying = false;
  wagnerIndex = 0;
  wagnerTonePhase = false;
}

void startWagner() {
  if (wagnerPlaying) return;

  wagnerPlaying = true;
  wagnerIndex = 0;
  wagnerTonePhase = true;
  wagnerStepStart = millis();

  tone(BUZZER, wagnerMelody[wagnerIndex]);
}

void updateWagner() {
  if (!wagnerPlaying) return;

  unsigned long now = millis();
  unsigned long fullDuration = getWagnerFullDurationMs(wagnerIndex);

  unsigned long toneDuration = (fullDuration * 88UL) / 100UL;
  unsigned long gapDuration = fullDuration - toneDuration;

  if (wagnerTonePhase) {
    if (now - wagnerStepStart >= toneDuration) {
      noTone(BUZZER);
      wagnerTonePhase = false;
      wagnerStepStart = now;
    }
  } else {
    if (now - wagnerStepStart >= gapDuration) {
      wagnerIndex++;

      if (wagnerIndex >= WAGNER_LEN) {
        wagnerIndex = 0;
      }

      tone(BUZZER, wagnerMelody[wagnerIndex]);
      wagnerTonePhase = true;
      wagnerStepStart = now;
    }
  }
}

// =====================================================
// BEEP COFANIA
// =====================================================

void stopReverseBeep() {
  if (beepOn) {
    noTone(BUZZER);
    beepOn = false;
  }
}

void updateReverseBeep() {
  unsigned long now = millis();

  if (!beepOn && now - lastBeepTime >= 500) {
    tone(BUZZER, 500);
    beepOn = true;
    lastBeepTime = now;
  }
  else if (beepOn && now - lastBeepTime >= 500) {
    noTone(BUZZER);
    beepOn = false;
    lastBeepTime = now;
  }
}

void updateSound() {
  if (wagnerEnabled) {
    stopReverseBeep();

    if (!wagnerPlaying) {
      startWagner();
    }

    updateWagner();
    return;
  }

  if (wagnerPlaying) {
    stopWagner();
  }

  bool reversing = currentLeftDir < 0 && currentRightDir < 0;

  if (reversing) {
    updateReverseBeep();
  } else {
    stopReverseBeep();
  }
}

// =====================================================
// PRZYCISKI A1 / A2
// =====================================================

static void handleAuxButtons(uint8_t newMask) {
  bool a1Now = (newMask & BTN_MASK_A1) != 0;
  bool a1Prev = (lastButtonsMask & BTN_MASK_A1) != 0;

  if (a1Now && !a1Prev) {
    wagnerEnabled = !wagnerEnabled;

    Serial.print("VALKYRIE=");
    Serial.println(wagnerEnabled ? "ON" : "OFF");

    if (!wagnerEnabled) {
      stopWagner();
    }
  }

  lastButtonsMask = newMask;
}

// =====================================================
// MANUAL
// =====================================================

static void runManualMode(const Packet &p) {
  const bool leftFwd  = (p.buttons_bitmask & BTN_MASK_LEFT_FWD) != 0;
  const bool leftRev  = (p.buttons_bitmask & BTN_MASK_LEFT_REV) != 0;
  const bool rightFwd = (p.buttons_bitmask & BTN_MASK_RIGHT_FWD) != 0;
  const bool rightRev = (p.buttons_bitmask & BTN_MASK_RIGHT_REV) != 0;

  int leftDir = 0;
  int rightDir = 0;

  if (leftFwd && !leftRev) {
    leftDir = 1;
  } else if (!leftFwd && leftRev) {
    leftDir = -1;
  }

  if (rightFwd && !rightRev) {
    rightDir = 1;
  } else if (!rightFwd && rightRev) {
    rightDir = -1;
  }

  currentLeftDir = leftDir;
  currentRightDir = rightDir;

  applyMotors();

  if (debugEnabled) {
    Serial.print("MANUAL BUTTONS=0x");

    if (p.buttons_bitmask < 16) {
      Serial.print('0');
    }

    Serial.print(p.buttons_bitmask, HEX);
    Serial.print(" LEFT=");
    Serial.print(leftDir);
    Serial.print(" RIGHT=");
    Serial.println(rightDir);
  }
}

// =====================================================
// AUTO — NA RAZIE WYŁĄCZONE
// =====================================================

static void runAutoMode() {
  stopMotors();

  if (debugEnabled) {
    Serial.println("AUTO ignored - IR disabled");
  }
}

// =====================================================
// SERIAL DEBUG
// =====================================================

static void printStatus() {
  Serial.print("STATUS MODE=");
  Serial.print(currentMode == MODE_MANUAL ? "MANUAL" : "AUTO");

  Serial.print(" BUTTONS=0x");
  if (data.buttons_bitmask < 16) {
    Serial.print('0');
  }
  Serial.print(data.buttons_bitmask, HEX);

  Serial.print(" LEFT=");
  Serial.print(currentLeftDir);

  Serial.print(" RIGHT=");
  Serial.print(currentRightDir);

  Serial.print(" VALKYRIE=");
  Serial.println(wagnerEnabled ? "ON" : "OFF");

  Serial.print(" RADIO_CONNECTED=");
  Serial.println(radio.isChipConnected() ? "YES" : "NO");
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
  }
  else if (strcmp(cmd, "DEBUG OFF") == 0) {
    debugEnabled = false;
    Serial.println("DEBUG=OFF");
  }
  else if (strcmp(cmd, "DEBUG") == 0) {
    debugEnabled = !debugEnabled;
    Serial.print("DEBUG=");
    Serial.println(debugEnabled ? "ON" : "OFF");
  }
  else if (strcmp(cmd, "STATUS") == 0) {
    printStatus();
  }
  else if (strcmp(cmd, "HELP") == 0) {
    Serial.println("CMDS: DEBUG ON | DEBUG OFF | DEBUG | STATUS | HELP");
  }
  else {
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
    }
    else if (serialLen < sizeof(serialBuf) - 1) {
      serialBuf[serialLen++] = c;
    }
  }
}

// =====================================================
// SETUP / LOOP
// =====================================================

void setup() {
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(BUZZER, OUTPUT);

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

  lastPacketTime = millis();

  Serial.println("RX gotowy");
  Serial.println("Radio CE=9 CSN=10");
  Serial.println("Listening pipe=1 addr=CTRL1 ch=76 rate=250KBPS pa=MIN");
  Serial.println("Motors IN1=5 IN2=4 IN3=3 IN4=2");
  Serial.println("A1 toggles VALKYRIE!!!");
}

void loop() {
  handleSerialCommands();

  if (radio.available()) {
    radio.read(&data, sizeof(data));

    currentMode = data.mode;
    lastPacketTime = millis();

    handleAuxButtons(data.buttons_bitmask);

    if (currentMode != lastMode) {
      lastMode = currentMode;

      stopMotors();

      if (debugEnabled) {
        Serial.print("TRYB -> ");
        Serial.println(currentMode == MODE_MANUAL ? "MANUAL" : "AUTO");
      }
    }

    if (currentMode == MODE_MANUAL) {
      runManualMode(data);
    } else {
      runAutoMode();
    }
  }

  if (millis() - lastPacketTime > SIGNAL_TIMEOUT) {
    stopMotors();
  }

  if (debugEnabled && millis() - lastPacketTime > 1000 && millis() - lastNoPacketDebugTime > 1000) {
    lastNoPacketDebugTime = millis();
    Serial.print("NO PACKETS for ");
    Serial.print(millis() - lastPacketTime);
    Serial.print(" ms, RADIO_CONNECTED=");
    Serial.println(radio.isChipConnected() ? "YES" : "NO");
  }

  updateSound();
}
