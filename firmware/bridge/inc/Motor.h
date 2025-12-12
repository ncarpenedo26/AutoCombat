// --- Motor.h ---
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
  /**
   * @brief Constructor for the motor.
   * @param pin1 The first input pin (Digital/Direction).
   * @param pin2 The second input pin (PWM/Speed).
   * @param channel The ESP32 LEDC channel to use (0-15).
   */
  Motor(byte pin1, byte pin2);
  
  void init();
  
  /**
   * @brief Sets the motor speed and direction.
   * @param velocity A value from -1.0 (full reverse) to 1.0 (full forward).
   */
  void set(double velocity);

private:
  const int _freq = 5000;      // 5 kHz PWM Frequency
  const int _resolution = 8;   // 8-bit resolution (0-255 duty cycle)
  const int _dutyCycleMax = pow(2,_resolution) - 1;

  byte _pin1;
  byte _pin2;
};

#endif