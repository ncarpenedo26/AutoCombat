#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
  Encoder(byte pinA, byte pinB);

  // Configuration (Call in setup())
  void init();

  // Get motor rotations before gearing
  double getRotations();

  long getCount();

  void reset();

  // The interrupt handler logic
  // Must be called by the ISR in the main loop
  void update();

private:
  byte _pinA;
  byte _pinB;
  volatile long _count;
  volatile byte _oldState;
};

#endif