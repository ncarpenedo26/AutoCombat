// Arduino Serial Sketch for Mecanum Robot Control Bridge
//
// Protocol Summary:
// Inbound (Pi -> Arduino): 'T' for Twist Command (Motor Speeds)
// Outbound (Arduino -> Pi): 'M' for Mecanum Encoder Data


#include <ESP32Encoder.h>
#include "inc/Mecanum.h"
#include "inc/Motor.h"

// ----------------------------------------------------
// Configuration & Objects
// ----------------------------------------------------
const long BAUD_RATE = 115200;
const long ODOMETRY_COMPUTE_INTERVAL_MS = 50;  // Send encoder data 20 times per second
const long DRIVETRAIN_UPDATE_INTERVAL_MS = 5.0;
const double DRIVETRAIN_WHEELBASE = 0.15; // meters
const double REDUCTION = 31.5; // 31.5:1 gear reduction
const int PPR = 7; // Encoder pulses per revolution
const double WHEEL_RADIUS = 0.024;

unsigned long lastOdometryCalcTime = 0;
unsigned long lastDrivetrainUpdateTime = 0;

// Calculates encoder counts to revolutions
double countsToRadians(double count) {
  // Extra divide by 2 because quadrature?
  double revolutions = ((double)count) / (PPR * REDUCTION * 2);
  return revolutions * 2 * PI;
}

// dt in SECONDS
double getAngularVelocity(int deltaCounts, double elapsed_ms) {
  double countsPerSecond = ((double)(deltaCounts)) * 1000 / ((double)elapsed_ms);
  return countsToRadians(countsPerSecond);
}

// TODO: PINS NOT FINAL
Motor bl(14, 27);
Motor br(13, 12);
Motor fr(18, 19);
Motor fl(25, 26);
MecanumDrive drivetrain(fl, fr, bl, br);

ESP32Encoder EncoderFL;
ESP32Encoder EncoderFR;
ESP32Encoder EncoderBL;
ESP32Encoder EncoderBR;

int prevFLCountsDrive = 0;
int prevFRCountsDrive = 0;
int prevBLCountsDrive = 0;
int prevBRCountsDrive = 0;

int prevFLCountsOdom = 0;
int prevFRCountsOdom = 0;
int prevBLCountsOdom = 0;
int prevBRCountsOdom = 0;

// Variables for incoming serial data
String inputString = "";      // A string to hold incoming data
bool stringComplete = false;  // Whether the string is complete

double targetXVel = 0;
double targetYVel = 0;
double targetRot = 0;

double posX = 0;
double posY = 0;
double theta = 0;

void setup() {
  Serial.begin(BAUD_RATE);
  inputString.reserve(200);  // Reserve buffer space for incoming data

  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  // TODO: PINS NOT FINAL
  EncoderFL.attachHalfQuad(36, 39);
  EncoderFR.attachHalfQuad(35, 34);
  EncoderBL.attachHalfQuad(32, 33);  // VN, VP
  EncoderBR.attachHalfQuad(22, 23);

  EncoderFL.clearCount();
  EncoderFR.clearCount();
  EncoderBL.clearCount();
  EncoderBR.clearCount();

  // Initialize peripherals
  drivetrain.init();

  Serial.println("Mecanum Serial Bridge Initialized.");
  Serial.println("Ready for RPi Control (Twist 'T').");
}

void loop() {
  // Check if a command has been received and parsed (non-blocking)
  if (stringComplete) {
    handleIncomingCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  // Non-blocking Encoder Data Transmission
  if (millis() - lastOdometryCalcTime >= ODOMETRY_COMPUTE_INTERVAL_MS) {
    sendOdometryData();
    lastOdometryCalcTime = millis();
  }

  if (millis() - lastDrivetrainUpdateTime >= DRIVETRAIN_UPDATE_INTERVAL_MS) {
    double flAngularVelocity = getAngularVelocity(EncoderFL.getCount() - prevFLCountsDrive, DRIVETRAIN_UPDATE_INTERVAL_MS);
    double frAngularVelocity = getAngularVelocity(EncoderFR.getCount() - prevFRCountsDrive, DRIVETRAIN_UPDATE_INTERVAL_MS);
    double blAngularVelocity = getAngularVelocity(EncoderBL.getCount() - prevBLCountsDrive, DRIVETRAIN_UPDATE_INTERVAL_MS);
    double brAngularVelocity = getAngularVelocity(EncoderBR.getCount() - prevBRCountsDrive, DRIVETRAIN_UPDATE_INTERVAL_MS);

    drivetrain.drive(
      targetXVel,
      targetYVel,
      targetRot,
      flAngularVelocity,
      frAngularVelocity,
      blAngularVelocity,
      brAngularVelocity);

    prevFLCountsDrive = EncoderFL.getCount();
    prevFRCountsDrive = EncoderFR.getCount();
    prevBLCountsDrive = EncoderBL.getCount();
    prevBRCountsDrive = EncoderBR.getCount();
    lastDrivetrainUpdateTime = millis();
  }
}


void sendOdometryData() {
  int rotFL = EncoderFL.getCount();
  int rotFR = EncoderFR.getCount();
  int rotBL = EncoderBL.getCount();
  int rotBR = EncoderBR.getCount();

  double flAngularVel = getAngularVelocity(rotFL - prevFLCountsOdom, ODOMETRY_COMPUTE_INTERVAL_MS);
  double frAngularVel = getAngularVelocity(rotFR - prevFRCountsOdom, ODOMETRY_COMPUTE_INTERVAL_MS);
  double blAngularVel = getAngularVelocity(rotBL - prevBLCountsOdom, ODOMETRY_COMPUTE_INTERVAL_MS);
  double brAngularVel = getAngularVelocity(rotBR - prevBRCountsOdom, ODOMETRY_COMPUTE_INTERVAL_MS);

  double vx = (flAngularVel + frAngularVel + blAngularVel + brAngularVel) * WHEEL_RADIUS / 4;
  double vy = (-flAngularVel + frAngularVel + blAngularVel - brAngularVel) * WHEEL_RADIUS / 4;
  double wz = (-flAngularVel + frAngularVel - blAngularVel + brAngularVel) * WHEEL_RADIUS / (4 * DRIVETRAIN_WHEELBASE);

  posX += vx * ODOMETRY_COMPUTE_INTERVAL_MS / 1000;
  posY += vy * ODOMETRY_COMPUTE_INTERVAL_MS / 1000;
  theta += wz * ODOMETRY_COMPUTE_INTERVAL_MS / 1000;

  String serializedData = "M";
  serializedData += String(posX, 3);
  serializedData += ",";
  serializedData += String(posY, 3);
  serializedData += ",";
  serializedData += String(theta, 3);

  // String serializedData = "M";
  // serializedData += String(rotFL, 3);
  // serializedData += ",";
  // serializedData += String(rotFR, 3);
  // serializedData += ",";
  // serializedData += String(rotBL, 3);
  // serializedData += ",";
  // serializedData += String(rotBR, 3);

  Serial.println(serializedData);

  prevFLCountsOdom = rotFL;
  prevFRCountsOdom = rotFR;
  prevBLCountsOdom = rotBL;
  prevBRCountsOdom = rotBR;

  return;
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    // Check for the newline character, which marks the end of a message
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

// Handles parsing and executing commands received from the Pi
void handleIncomingCommand(String fullCommand) {
  fullCommand.trim();

  if (fullCommand.length() < 3) return;  // Minimum length for 'T,x' or 'R,x'

  // Get the Message Type ID (First character)
  char typeId = fullCommand.charAt(0);
  // dataPayload starts after the ID and comma (e.g., "T,0.5,0.0,0.1" -> "0.5,0.0,0.1")
  String dataPayload = fullCommand.substring(1);

  switch (typeId) {
    case 'T':
      // Twist Command: T,<linear_x>,<linear_y>,<angular_z>
      handleTwistCommand(dataPayload);
      break;

    default:
      break;
  }
}

void handleTwistCommand(String payload) {
  float x_vel = 0.0, y_vel = 0.0, z_angular = 0.0;

  // Find the first comma (separates X from Y)
  int comma1 = payload.indexOf(',');
  if (comma1 == -1) {
    return;
  }

  // Find the second comma (separates Y from Z)
  int comma2 = payload.indexOf(',', comma1 + 1);
  if (comma2 == -1) {
    return;
  }

  // Extract substrings and convert to float
  // Start at 1 to skip the message type char, e.g. 'T' for twist
  String xStr = payload.substring(0, comma1);
  String yStr = payload.substring(comma1 + 1, comma2);
  String zStr = payload.substring(comma2 + 1);

  x_vel = xStr.toFloat();
  y_vel = yStr.toFloat();
  z_angular = zStr.toFloat();

  // Serial.println("X: " + String(x_vel) + ", Y: " + String(y_vel) + ", Rot: " + String(z_angular));
  targetXVel = x_vel;
  targetYVel = y_vel;
  targetRot = z_angular;
}