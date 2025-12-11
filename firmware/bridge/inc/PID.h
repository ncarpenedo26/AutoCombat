#ifndef PID_H
#define PID_H

class PID {
public:

  PID(double kP, double kI, double kD, bool allowReverse, double maxAccumulatedError);

  double compute(double input);
  void setSetpoint(double setpoint);

private:
  double _setpoint = 0;

  double _kP = 0;
  double _kI = 0;
  double _kD = 0;

  double _prevError = 0;
  double _accumulatedError = 0;

  bool _allowReverse = true;

  // 0 == no limit
  double _maxAccumulatedError = 0;

};

#endif