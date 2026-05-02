#include <Arduino.h>
const uint16_t BURST_US = 800;    // czas nadawania nośnej
const uint16_t GAP_US   = 9200;   // przerwa między burstami

void enable38kHz() {
  // Podłącz PWM z Timer1 do pinu D9 (OC1A)
  TCCR1A |= _BV(COM1A1);
}

void disable38kHz() {
  // Odłącz PWM od D9 i wymuś stan niski
  TCCR1A &= ~_BV(COM1A1);
  digitalWrite(9, LOW);
}

void setup38kHzOnD9() {
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);

  // Timer1: Fast PWM, TOP = ICR1, bez preskalera
  TCCR1A = 0;
  TCCR1B = 0;

  TCCR1A |= _BV(WGM11);                    // Fast PWM część A
  TCCR1B |= _BV(WGM13) | _BV(WGM12);       // Fast PWM część B
  TCCR1B |= _BV(CS10);                     // preskaler = 1

  // f = 16 MHz / (1 * (1 + ICR1))
  // dla ~38 kHz: ICR1 ≈ 420
  ICR1 = 420;

  // 50% duty
  OCR1A = ICR1 / 2;

  disable38kHz();
}

void setup() {
  setup38kHzOnD9();
}

void loop() {
  enable38kHz();
  delayMicroseconds(BURST_US);

  disable38kHz();
  delayMicroseconds(GAP_US);
}