#ifndef MECANUM_H
#define MECANUM_H

#include "Motor.h"
#include <Arduino.h>

/**
 * @class MecanumDrive
 * @brief Controls a 4-wheel mecanum drivetrain using Motor objects.
 * * Assumes a standard 'X' configuration:
 * - M1: Front Left (FL)
 * - M2: Front Right (FR)
 * - M3: Back Left (BL)
 * - M4: Back Right (BR)
 */
class MecanumDrive {
public:
  /**
   * @brief Constructor for the MecanumDrive class.
   */
  MecanumDrive(Motor &flMotor, Motor &frMotor, Motor &blMotor, Motor &brMotor);

  /**
   * @brief Initializes all four underlying Motor objects.
   * Call this in the Arduino setup() function.
   */
  void init();

  /**
   * @brief Sets the desired velocity and direction for the drivetrain.
   * @param forward The forward/backward speed (-1.0 to 1.0).
   * @param strafe The left/right strafing speed (-1.0 to 1.0).
   * @param rotation The turning speed (clockwise/counter-clockwise) (-1.0 to 1.0).
   */
  void drive(double forward, double strafe, double rotation);

private:
  Motor& _flMotor; // Front Left
  Motor& _frMotor; // Front Right
  Motor& _blMotor; // Back Left
  Motor& _brMotor; // Back Right
  
  /**
   * @brief Normalizes the motor speeds to ensure no motor exceeds the [-1.0, 1.0] range.
   * @param v1 Speed for M1 (FL).
   * @param v2 Speed for M2 (FR).
   * @param v3 Speed for M3 (BL).
   * @param v4 Speed for M4 (BR).
   */
  void normalize(double& v1, double& v2, double& v3, double& v4);
};

#endif