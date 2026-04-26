#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <ctype.h>
#include <string.h>
#include <avr/interrupt.h>

#include "protocol.h"

const int LEFT_IN1  = 6;
const int LEFT_IN2  = 5;
const int RIGHT_IN1 = 10;
const int RIGHT_IN2 = 9;

const byte CE_PIN  = 7;
const byte CSN_PIN = 8;

// ===== IR sensors =====
// faktyczne przypisanie:
const byte IR_LEFT_PIN   = 2;   // D2 = PD2
const byte IR_RIGHT_PIN  = 3;   // D3 = PD3
const byte IR_CENTER_PIN = 4;   // D4 = PD4

const unsigned long IR_DEBOUNCE_US = 300;

// ===== AUTO config =====
static constexpr bool AUTO_USE_PWM = false;   // <-- zmień na false, aby AUTO było bez PWM

const unsigned long AUTO_UPDATE_MS = 100;

const uint16_t IR_NONE_THRESHOLD   = 2;
const uint16_t IR_STRONG_THRESHOLD = 15;
const int IR_SIDE_MARGIN           = 4;

// PWM / continuous steering tuning
const int AUTO_BASE_SPEED_PWM      = 85;
const int AUTO_SLOW_SPEED_PWM      = 60;
const int AUTO_SEARCH_SPEED_PWM    = 70;
const int AUTO_MAX_SPEED_PWM       = 140;
const int AUTO_TURN_GAIN           = 10;
const int AUTO_MAX_TURN_PWM        = 90;

// non-PWM fallback still uses same logic, but outputs full ON/OFF
const int AUTO_BINARY_THRESHOLD    = 10;

volatile uint16_t irHitsLeft   = 0;
volatile uint16_t irHitsCenter = 0;
volatile uint16_t irHitsRight  = 0;

volatile unsigned long irLastEdgeLeftUs   = 0;
volatile unsigned long irLastEdgeCenterUs = 0;
volatile unsigned long irLastEdgeRightUs  = 0;

volatile uint8_t lastPortDState = 0;

RF24 radio(CE_PIN, CSN_PIN);

Packet data;

uint8_t currentMode = MODE_MANUAL;
uint8_t lastMode = 255;

unsigned long lastManualPacketTime = 0;
const unsigned long SIGNAL_TIMEOUT = 300;

bool debugEnabled = false;

char serialBuf[32];
uint8_t serialLen = 0;

// ===== AUTO state =====
unsigned long lastAutoUpdateMs = 0;
int8_t lastSeenDirection = 1;   // -1 = lewo, +1 = prawo
uint16_t lastAutoL = 0;
uint16_t lastAutoC = 0;
uint16_t lastAutoR = 0;

// ===== IR helpers =====

static void resetIrCounters() {
  noInterrupts();
  irHitsLeft = 0;
  irHitsCenter = 0;
  irHitsRight = 0;
  irLastEdgeLeftUs = 0;
  irLastEdgeCenterUs = 0;
  irLastEdgeRightUs = 0;
  interrupts();
}

static void snapshotAndClearIrCounters(uint16_t &l, uint16_t &c, uint16_t &r) {
  noInterrupts();
  l = irHitsLeft;
  c = irHitsCenter;
  r = irHitsRight;
  irHitsLeft = 0;
  irHitsCenter = 0;
  irHitsRight = 0;
  interrupts();
}

static void setupIrSensors() {
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_CENTER_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);

  lastPortDState = PIND;

  // Pin Change Interrupt dla portu D
  PCICR |= _BV(PCIE2);

  // D2, D3, D4 -> PCINT18, PCINT19, PCINT20
  PCMSK2 |= _BV(PCINT18);
  PCMSK2 |= _BV(PCINT19);
  PCMSK2 |= _BV(PCINT20);
}

ISR(PCINT2_vect) {
  uint8_t nowState = PIND;
  uint8_t changed = nowState ^ lastPortDState;
  uint8_t falling = changed & lastPortDState & (~nowState);
  lastPortDState = nowState;

  unsigned long nowUs = micros();

  // LEFT = D2 = PD2
  if (falling & _BV(PD2)) {
    if (nowUs - irLastEdgeLeftUs > IR_DEBOUNCE_US) {
      irHitsLeft++;
      irLastEdgeLeftUs = nowUs;
    }
  }

  // RIGHT = D3 = PD3
  if (falling & _BV(PD3)) {
    if (nowUs - irLastEdgeRightUs > IR_DEBOUNCE_US) {
      irHitsRight++;
      irLastEdgeRightUs = nowUs;
    }
  }

  // CENTER = D4 = PD4
  if (falling & _BV(PD4)) {
    if (nowUs - irLastEdgeCenterUs > IR_DEBOUNCE_US) {
      irHitsCenter++;
      irLastEdgeCenterUs = nowUs;
    }
  }
}

// ===== Motor helpers =====

// binary/manual
static void setLeftTrack(int dir) {
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
  analogWrite(LEFT_IN1, 0);
  analogWrite(LEFT_IN2, 0);
  analogWrite(RIGHT_IN1, 0);
  analogWrite(RIGHT_IN2, 0);

  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
}

static void setLeftTrackAuto(int speedVal) {
  speedVal = constrain(speedVal, -255, 255);

  if (!AUTO_USE_PWM) {
    if (speedVal > AUTO_BINARY_THRESHOLD) {
      digitalWrite(LEFT_IN1, HIGH);
      digitalWrite(LEFT_IN2, LOW);
    } else if (speedVal < -AUTO_BINARY_THRESHOLD) {
      digitalWrite(LEFT_IN1, LOW);
      digitalWrite(LEFT_IN2, HIGH);
    } else {
      digitalWrite(LEFT_IN1, LOW);
      digitalWrite(LEFT_IN2, LOW);
    }
    return;
  }

  if (speedVal > 0) {
    analogWrite(LEFT_IN1, speedVal);
    digitalWrite(LEFT_IN2, LOW);
  } else if (speedVal < 0) {
    digitalWrite(LEFT_IN1, LOW);
    analogWrite(LEFT_IN2, -speedVal);
  } else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
  }
}

static void setRightTrackAuto(int speedVal) {
  speedVal = constrain(speedVal, -255, 255);

  if (!AUTO_USE_PWM) {
    if (speedVal > AUTO_BINARY_THRESHOLD) {
      digitalWrite(RIGHT_IN1, HIGH);
      digitalWrite(RIGHT_IN2, LOW);
    } else if (speedVal < -AUTO_BINARY_THRESHOLD) {
      digitalWrite(RIGHT_IN1, LOW);
      digitalWrite(RIGHT_IN2, HIGH);
    } else {
      digitalWrite(RIGHT_IN1, LOW);
      digitalWrite(RIGHT_IN2, LOW);
    }
    return;
  }

  if (speedVal > 0) {
    analogWrite(RIGHT_IN1, speedVal);
    digitalWrite(RIGHT_IN2, LOW);
  } else if (speedVal < 0) {
    digitalWrite(RIGHT_IN1, LOW);
    analogWrite(RIGHT_IN2, -speedVal);
  } else {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
  }
}

// ===== MANUAL =====

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

// ===== AUTO continuous =====

static void runAutoMode() {
  if (millis() - lastAutoUpdateMs < AUTO_UPDATE_MS) {
    return;
  }

  lastAutoUpdateMs = millis();

  uint16_t l, c, r;
  snapshotAndClearIrCounters(l, c, r);

  lastAutoL = l;
  lastAutoC = c;
  lastAutoR = r;

  int leftSpeed = 0;
  int rightSpeed = 0;

  const bool lost = (l <= IR_NONE_THRESHOLD) &&
                    (c <= IR_NONE_THRESHOLD) &&
                    (r <= IR_NONE_THRESHOLD);

  if (lost) {
    // brak sygnału -> wolny obrót w ostatnią znaną stronę
    if (lastSeenDirection < 0) {
      leftSpeed = -AUTO_SEARCH_SPEED_PWM;
      rightSpeed = AUTO_SEARCH_SPEED_PWM;
    } else {
      leftSpeed = AUTO_SEARCH_SPEED_PWM;
      rightSpeed = -AUTO_SEARCH_SPEED_PWM;
    }
  } else {
    int error = (int)r - (int)l;

    if (error > IR_SIDE_MARGIN) {
      lastSeenDirection = 1;
    } else if (error < -IR_SIDE_MARGIN) {
      lastSeenDirection = -1;
    }

    int base = AUTO_BASE_SPEED_PWM;

    // jeśli sygnał środkowy słabszy, jedź ostrożniej
    if (c < IR_STRONG_THRESHOLD) {
      base = AUTO_SLOW_SPEED_PWM;
    }

    // jeśli różnica boków duża, zwolnij bazę żeby najpierw się ustawić
    if (abs(error) > 6) {
      base = AUTO_SLOW_SPEED_PWM;
    }

    int turn = constrain(error * AUTO_TURN_GAIN, -AUTO_MAX_TURN_PWM, AUTO_MAX_TURN_PWM);

    leftSpeed  = constrain(base + turn, -AUTO_MAX_SPEED_PWM, AUTO_MAX_SPEED_PWM);
    rightSpeed = constrain(base - turn, -AUTO_MAX_SPEED_PWM, AUTO_MAX_SPEED_PWM);

    // jeśli center nic nie widzi, a jeden bok widzi wyraźnie więcej, to bardziej obracaj niż jedź
    if (c <= IR_NONE_THRESHOLD) {
      if (r > l + IR_SIDE_MARGIN) {
        leftSpeed = AUTO_SEARCH_SPEED_PWM;
        rightSpeed = -AUTO_SEARCH_SPEED_PWM;
      } else if (l > r + IR_SIDE_MARGIN) {
        leftSpeed = -AUTO_SEARCH_SPEED_PWM;
        rightSpeed = AUTO_SEARCH_SPEED_PWM;
      }
    }
  }

  setLeftTrackAuto(leftSpeed);
  setRightTrackAuto(rightSpeed);

  if (debugEnabled) {
    Serial.print("AUTO IR L=");
    Serial.print(l);
    Serial.print(" C=");
    Serial.print(c);
    Serial.print(" R=");
    Serial.print(r);
    Serial.print(" -> LS=");
    Serial.print(leftSpeed);
    Serial.print(" RS=");
    Serial.println(rightSpeed);
  }
}

// ===== Serial/debug =====

static void printStatus() {
  Serial.print("STATUS MODE=");
  Serial.print(currentMode == MODE_MANUAL ? "MANUAL" : "AUTO");
  Serial.print(" LAST_BUTTONS=0x");
  if (data.buttons_bitmask < 16) Serial.print('0');
  Serial.print(data.buttons_bitmask, HEX);
  Serial.print(" IR L=");
  Serial.print(lastAutoL);
  Serial.print(" C=");
  Serial.print(lastAutoC);
  Serial.print(" R=");
  Serial.println(lastAutoR);
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
  setupIrSensors();
  resetIrCounters();

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

  lastAutoUpdateMs = millis();

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

      if (currentMode == MODE_AUTO) {
        resetIrCounters();
        lastAutoUpdateMs = millis();
        stopMotors();
      } else {
        stopMotors();
      }

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
    runAutoMode();
  } else {
    if (millis() - lastManualPacketTime > SIGNAL_TIMEOUT) {
      stopMotors();
    }
  }
}