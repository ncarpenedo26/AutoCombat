#include "../inc/Motor.h"

Motor::Motor(byte pin)
  : _pin(pin) {}

void Motor::init() {
  _esc.attach(ESC_PIN);
  // Set neutral position to arm the ESC
  _esc.writeMicroseconds(NEUTRAL_PULSE); 
  delay(200); // Wait for arming sequence to complete
}

void Motor::set(double velocity) {
  // Map -1.0 -> MIN_PULSE, 1.0 -> MAX_PULSE
  int pulse = velocity * (MAX_PULSE - MIN_PULSE) + MIN_PULSE;
  int constrained = constrain(pulse, MIN_PULSE, MAX_PULSE);
  _esc.writeMicroseconds(constrained);
}