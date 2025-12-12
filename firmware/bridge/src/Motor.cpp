#include "../inc/Motor.h"

#include <Arduino.h>

Motor::Motor(byte pin1, byte pin2)
  : _pin1(pin1), _pin2(pin2) {}

void Motor::init() {
  // Initialize PWM pins
  ledcAttach(_pin1, _freq, _resolution);
  ledcAttach(_pin2, _freq, _resolution);

  // Set seped 0
  ledcWrite(_pin1, 0);
  ledcWrite(_pin2, 0);
}

void Motor::set(double velocity) {
  // Map -1.0 -> MIN_PULSE, 1.0 -> MAX_PULSE
  int duty = abs(velocity) * _dutyCycleMax;
  int clamped = min(max(duty, 0), _dutyCycleMax);
  if (velocity < 0) {
    ledcWrite(_pin1, clamped);
    ledcWrite(_pin2, 0);
  } else {
    ledcWrite(_pin1, 0);
    ledcWrite(_pin2, clamped);
  }
}