#include "../inc/PID.h"

PID::PID(double kP, double kI, double kD, bool allowReverse=true, double maxAccumulatedError=0) :
  _kP(kP), _kI(kI), _kD(kD), _allowReverse(allowReverse), _maxAccumulatedError(maxAccumulatedError) {}

void PID::setSetpoint(double setpoint) {
  _setpoint = setpoint;
}

double PID::compute(double input) {
  double err = _setpoint - input;
  double errdot = err - _prevError;
  double output = _kP * err + _kD * errdot + _kI * _accumulatedError; //+ prevPWM;

  _prevError = err;
  _accumulatedError += err;

  // Clamp accumulated error
  if (_accumulatedError && _accumulatedError > _maxAccumulatedError) {
    _accumulatedError = _maxAccumulatedError;
  }

  if (_accumulatedError && _accumulatedError < -_maxAccumulatedError) {
    _accumulatedError = -_maxAccumulatedError;
  }

  // Output 0 if reverse is disallowed
  if (!_allowReverse && _setpoint * input < 0) {
    output = 0;
  }

  return output;
}