#include "../inc/Encoder.h"

Encoder::Encoder(byte pinA, byte pinB) : _pinA(pinA), _pinB(pinB) {
  _count = 0;
  _oldState = 0;
}

// Configuration (Call in setup())
void Encoder::init() {
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);
  
  // Read initial state for direction detection
  _oldState = (digitalRead(_pinB) << 1) | digitalRead(_pinA);
}

long Encoder::getCount() {
  noInterrupts();
  long currentCount = _count;
  interrupts();
  return currentCount;
}

void Encoder::reset() {
  noInterrupts();
  _count = 0;
  interrupts();
}

// The interrupt handler logic
// Must be called by the ISR in the main loop
void Encoder::update() {
  byte newState = (digitalRead(_pinB) << 1) | digitalRead(_pinA);
  
  // Check for a state change. All pins in the same pin group will generate
  // an interrupt, so only update if our state has changed, meaning this device
  // triggered the interrupt
  if (newState != _oldState) {
    // Logic for 2-bit Gray Code state transitions (00 -> 01 -> 11 -> 10 -> 00)
    if (((_oldState == 0b00) && (newState == 0b01)) || // 0 -> 1 (CW)
        ((_oldState == 0b01) && (newState == 0b11)) || // 1 -> 3 (CW)
        ((_oldState == 0b11) && (newState == 0b10)) || // 3 -> 2 (CW)
        ((_oldState == 0b10) && (newState == 0b00))) { // 2 -> 0 (CW)
      _count++;
    } else {
      _count--;
    }
    _oldState = newState;
  }
}