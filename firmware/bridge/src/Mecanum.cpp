#include "../inc/Mecanum.h"
#include "../inc/PID.h"

#define MAX_MOTOR_SPEED 1.0 // Revolutions / second
#define REDUCTION 31.5 // 31.5:1 gear reduction
#define PPR 7 // Encoder pulses per revolution

//front left PID
double flKp=0.5, flKi=0.05, flKd=0.2;
PID flPID(flKp, flKi, flKd, false, 10);

double frKp=0.5, frKi=0.05, frKd=0.2;
PID frPID(frKp, frKi, frKd, false, 10);

//back left PID
double blKp=0.5, blKi=0.05, blKd=0.2;
PID blPID(blKp, blKi, blKd, false, 10);

//back right PID
double brKp=0.5, brKi=0.05, brKd=0.2;
PID brPID(brKp, brKi, brKd, false, 10);

MecanumDrive::MecanumDrive(Motor &flMotor, Motor &frMotor, Motor &blMotor, Motor &brMotor)
  : _flMotor(flMotor), _frMotor(frMotor), _blMotor(blMotor), _brMotor(brMotor) {
}

// Calculates encoder counts to revolutions
double countsToRevolutions(double count) {
  // Extra divide by 2 because quadrature?
  return ((double)count) / (PPR * REDUCTION * 2);
}

// calculates signs
int sign(double num) {
  if (num > 0) {
    return 1;
  } else if (num < 0) {
    return -1;
  } else {
    return 0;
  }
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
void MecanumDrive::drive(double forward, double strafe, double rotation, double e1, double e2, double e3, double e4) {
  double v1 = forward + strafe + rotation; 
  double v2 = forward - strafe - rotation;
  double v3 = forward - strafe + rotation;
  double v4 = forward + strafe - rotation;

  flPID.setSetpoint(v1);
  frPID.setSetpoint(v2);
  blPID.setSetpoint(v3);
  brPID.setSetpoint(v4);
  
  //encoder counts to linear m/s: enc / 882counts/rev * 0.024m/s
  double flInput = countsToRevolutions(e1);
  double frInput = countsToRevolutions(e2);
  double blInput = countsToRevolutions(e3);
  double brInput = countsToRevolutions(e4);

  double flOut = flPID.compute(flInput);
  double frOut = frPID.compute(frInput);
  double blOut = blPID.compute(blInput);
  double brOut = brPID.compute(brInput);

  normalize(flOut, frOut, blOut, brOut);
  
  // Serial.print("Real_Speed:");
  // Serial.print(String(flInput));
  // Serial.print(" ");
  // Serial.print("Output_PWM_Signal:");
  // Serial.print(String(flOut));
  // Serial.print(" ");
  // Serial.print("Goal_Speed:");
  // Serial.print(String(v1));
  // Serial.print(" ");
  // Serial.print("Error_Sum:");
  // Serial.println(String(fltotalErr));
  flPID.debugPrint();

  _flMotor.set(flOut);
  _frMotor.set(frOut);
  _blMotor.set(blOut);
  _brMotor.set(brOut);
}

/**
 * @brief Finds the maximum absolute speed and scales all speeds down if necessary.
 * This ensures the commanded speed does not exceed the maximum motor velocity (1.0).
 */
void MecanumDrive::normalize(double& v1, double& v2, double& v3, double& v4) {

  v1 = (abs(v1) > MAX_MOTOR_SPEED) ? v1 / abs(v1) : v1;
  v2 = (abs(v2) > MAX_MOTOR_SPEED) ? v2 / abs(v2) : v2;
  v3 = (abs(v3) > MAX_MOTOR_SPEED) ? v3 / abs(v3) : v3;
  v4 = (abs(v4) > MAX_MOTOR_SPEED) ? v4 / abs(v4) : v4;

  int dirv1 = sign(v1);
  int dirv2 = sign(v2);
  int dirv3 = sign(v3);
  int dirv4 = sign(v4);
  
  v1 = dirv1 * map(int(abs(v1) * 100), 0, 100, 50, 100) / 100.0;
  v2 = dirv2 * map(int(abs(v2) * 100), 0, 100, 50, 100) / 100.0;
  v3 = dirv3 * map(int(abs(v3) * 100), 0, 100, 50, 100) / 100.0;
  v4 = dirv4 * map(int(abs(v4) * 100), 0, 100, 50, 100) / 100.0;
}