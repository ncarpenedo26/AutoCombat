#include "../inc/PID.h"
#include <Arduino.h>

PID::PID(double kP, double kI, double kD, bool allowReverse=true, double maxAccumulatedError=0) :
  _kP(kP), _kI(kI), _kD(kD), _allowReverse(allowReverse), _maxAccumulatedError(maxAccumulatedError) {}

void PID::setSetpoint(double setpoint) {
  _setpoint = setpoint;
}

void PID::debugPrint() {
  Serial.print("Real_Speed:");
  Serial.print(String(_prevInput));
  Serial.print(" ");
  // Serial.print("Output_PWM_Signal:");
  // Serial.print(String(_));
  Serial.print(" ");
  Serial.print("Goal_Speed:");
  Serial.print(String(_setpoint));
  Serial.print(" ");
  Serial.print("Error_Sum:");
  Serial.println(String(_accumulatedError));
}

double PID::compute(double input) {
  double err = _setpoint - input;
  double errdot = err - _prevError;
  double output = _kP * err + _kD * errdot + _kI * _accumulatedError; //+ prevPWM;

  _prevError = err;
  if (abs(_accumulatedError + err) < _maxAccumulatedError && abs(err) < 1.0) {
    _accumulatedError += err;
  }

  // Output 0 if reverse is disallowed
  if (!_allowReverse && _setpoint * input < 0) {
    output = 0;
  }

  _prevInput = input;

  return output;
}