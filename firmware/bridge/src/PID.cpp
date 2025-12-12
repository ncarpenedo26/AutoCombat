#include "../inc/PID.h"
#include <Arduino.h>

PID::PID(double kP, double kI, double kD, bool allowReverse=true, double maxAccumulatedError=0) :
  _kP(kP), _kI(kI), _kD(kD), _allowReverse(allowReverse), _maxAccumulatedError(maxAccumulatedError) {}

int signs(double num) {
  if (num > 0) {
    return 1;
  } else if (num < 0) {
    return -1;
  } else {
    return 0;
  }
}

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
  double output = _kP * err + _kD * errdot + _kI * _accumulatedError;

  _prevError = err;
  _accumulatedError += err;
  if (_accumulatedError > _maxAccumulatedError) {
    _accumulatedError = _maxAccumulatedError;
  } else if (_accumulatedError < -_maxAccumulatedError) {
    _accumulatedError = -_maxAccumulatedError;
  }
  // Output 0 if reverse is disallowed
  if (signs(_setpoint) != signs(output)) {
    output = 0;
  }

  _prevInput = input;

  return output;
}

