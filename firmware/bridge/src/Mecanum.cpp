#include "../inc/Mecanum.h"

MecanumDrive::MecanumDrive(byte flPin, byte frPin, byte blPin, byte brPin)
  : _flMotor(flPin), _frMotor(frPin), _blMotor(blPin), _brMotor(brPin) {
}

void MecanumDrive::init() {
  _flMotor.init();
  _frMotor.init();
  _blMotor.init();
  _brMotor.init();
}

/**
 * @brief Inverse Kinematics calculation and motor setting.
 * The core of the mecanum drive.
 */
void MecanumDrive::drive(double forward, double strafe, double rotation) {
  double v1 = forward + strafe + rotation; 
  double v2 = forward - strafe - rotation;
  double v3 = forward - strafe + rotation;
  double v4 = forward + strafe - rotation;

  normalize(v1, v2, v3, v4);

  _flMotor.set(v1);
  _frMotor.set(v2);
  _blMotor.set(v3);
  _brMotor.set(v4);
}

/**
 * @brief Finds the maximum absolute speed and scales all speeds down if necessary.
 * This ensures the commanded speed does not exceed the maximum motor velocity (1.0).
 */
void MecanumDrive::normalize(double& v1, double& v2, double& v3, double& v4) {
  // Find the maximum absolute value among all four speeds
  double maxAbsSpeed = 0.0;
  maxAbsSpeed = max(maxAbsSpeed, abs(v1));
  maxAbsSpeed = max(maxAbsSpeed, abs(v2));
  maxAbsSpeed = max(maxAbsSpeed, abs(v3));
  maxAbsSpeed = max(maxAbsSpeed, abs(v4));

  // If the maximum speed exceeds 1.0, scale all speeds down by that factor.
  if (maxAbsSpeed > 1.0) {
    v1 /= maxAbsSpeed;
    v2 /= maxAbsSpeed;
    v3 /= maxAbsSpeed;
    v4 /= maxAbsSpeed;
  }
}