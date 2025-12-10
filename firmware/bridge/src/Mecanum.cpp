#include "../inc/Mecanum.h"
#include <PID_v1.h>

#define MAX_MOTOR_SPEED 2.0 // Revolutions / second
#define REDUCTION 31.5 // 31.5:1 gear reduction
#define PPR 7 // Encoder pulses per revolution

//front left PID
double flSetpoint, flInput, flOutput;
double flKp=0.5, flKi=0.0, flKd=0.2;
PID flPID(&flInput, &flOutput, &flSetpoint, flKp, flKi, flKd, DIRECT);

//front right PID
double frSetpoint, frInput, frOutput;
double frKp=0.5, frKi=0.0, frKd=0.2;
PID frPID(&frInput, &frOutput, &frSetpoint, frKp, frKi, frKd, DIRECT);

//back left PID
double blSetpoint, blInput, blOutput;
double blKp=0.5, blKi=0.0, blKd=0.2;
PID blPID(&blInput, &blOutput, &blSetpoint, blKp, blKi, blKd, DIRECT);

//back right PID
double brSetpoint, brInput, brOutput;
double brKp=0.5, brKi=0.0, brKd=0.2;
PID brPID(&brInput, &brOutput, &brSetpoint, brKp, brKi, brKd, DIRECT);

MecanumDrive::MecanumDrive(Motor &flMotor, Motor &frMotor, Motor &blMotor, Motor &brMotor)
  : _flMotor(flMotor), _frMotor(frMotor), _blMotor(blMotor), _brMotor(brMotor) {
}

double countsToRevolutions(double count) {
  // Extra divide by 2 because quadrature?
  return ((double)count) / (PPR * REDUCTION * 2);
}

void MecanumDrive::init() {
  _flMotor.init();
  _frMotor.init();
  _blMotor.init();
  _brMotor.init();

  flSetpoint = 0;
  frSetpoint = 0;
  blSetpoint = 0;
  brSetpoint = 0;

  flPID.SetMode(AUTOMATIC);
  frPID.SetMode(AUTOMATIC);
  blPID.SetMode(AUTOMATIC);
  brPID.SetMode(AUTOMATIC);

  flPID.SetOutputLimits(-255, 255);
  frPID.SetOutputLimits(-255, 255);
  blPID.SetOutputLimits(-255, 255);
  brPID.SetOutputLimits(-255, 255);

}

/**
 * @brief Inverse Kinematics calculation and motor setting.
 * The core of the mecanum drive.
 */
void MecanumDrive::drive(double forward, double strafe, double rotation, double e1, double e2, double e3, double e4) {
  double v1 = forward + strafe + rotation; 
  double v2 = forward - strafe - rotation;
  double v3 = forward - strafe + rotation;
  double v4 = forward + strafe - rotation;

  // TODO: do we need to normalize here?
  // normalize(v1, v2, v3, v4);

  flSetpoint = v1;
  frSetpoint = v2;
  blSetpoint = v3;
  brSetpoint = v4;
  
  //encoder counts to linear m/s: enc / 882counts/rev * 0.024m/s
  flInput = countsToRevolutions(e1);
  frInput = countsToRevolutions(e2);
  blInput = countsToRevolutions(e3);
  brInput = countsToRevolutions(e4);

  flPID.Compute();
  frPID.Compute();
  blPID.Compute();
  brPID.Compute();

  // Serial.println("Setpoint 1: " + String(v1, 2) + " | Setpoint 2: " + String(v2, 2) + " | Setpoint 3: " + String(v3, 2) + " | Setpoint 4: " + String(v4, 2));
  // Serial.println("E 1: " + String(e1, 2) + " | E 2: " + String(e2, 2) + " | E 3: " + String(e3, 2) + " | E 4: " + String(e4, 2));
  // Serial.println("RPS 1: " + String(flInput, 2) + " | RPS 2: " + String(frInput, 2) + " | RPS 3: " + String(blInput, 2) + " | RPS 4: " + String(brInput, 2));
  // Serial.println("V1: " + String(flOutput, 2) + " | V2: " + String(frOutput, 2) + " | V3: " + String(blOutput, 2) + " | V4: " + String(brOutput, 2));

  Serial.println(String(flInput) + "," + String(flOutput) + "," + String(flSetpoint) );

  // _flMotor.set(flOutput);
  // _frMotor.set(frOutput);
  // _blMotor.set(blOutput);
  // _brMotor.set(brOutput);

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
  if (maxAbsSpeed > MAX_MOTOR_SPEED) {
    v1 /= maxAbsSpeed;
    v2 /= maxAbsSpeed;
    v3 /= maxAbsSpeed;
    v4 /= maxAbsSpeed;
  }
}