#ifndef MOTOR_H
#define MOTOR_H

#include <Servo.h>
#include <Arduino.h>

#define ESC_PIN 9

#define NEUTRAL_PULSE 1500
#define MIN_PULSE 1000
#define MAX_PULSE 2000

class Motor {
public:
  Motor(byte pin);
  void init();
  void set(double velocity); // -1.0 to 1.0

private:
  Servo _esc;
  byte _pin;
};

#endif