// Arduino Serial Sketch for Mecanum Robot Control Bridge
//
// Protocol Summary:
// Inbound (Pi -> Arduino): 'T' for Twist Command (Motor Speeds)
// Outbound (Arduino -> Pi): 'M' for Mecanum Encoder Data


#include "inc/Encoder.h"
#include "inc/Mecanum.h"

// ----------------------------------------------------
// Configuration & Objects
// ----------------------------------------------------
const long BAUD_RATE = 115200;
const long ENCODER_SEND_INTERVAL_MS = 100; // Send encoder data 10 times per second
unsigned long lastEncoderSendTime = 0;

#define ENCODER_1_A_PIN 8 
#define ENCODER_1_B_PIN 9

MecanumDrive drivetrain(2, 3, 4, 5); // FL, FR, BL, BR pins
Encoder EncoderFL(ENCODER_1_A_PIN, ENCODER_1_B_PIN);
Encoder EncoderFR(10, 11);
Encoder EncoderBL(12, 1);
Encoder EncoderBR(6, 7);

// Variables for incoming serial data
String inputString = "";         // A string to hold incoming data
bool stringComplete = false;     // Whether the string is complete

void setup() {
  Serial.begin(BAUD_RATE);
  inputString.reserve(200); // Reserve buffer space for incoming data
  
  // Initialize peripherals
  drivetrain.init();
  EncoderFL.init();

  Serial.println("Mecanum Serial Bridge Initialized.");
  Serial.println("Ready for RPi Control (Twist 'T').");
  
  // Hardware-specific: Enable Pin Change Interrupts for Port B (D8-D13)
  // PCMSK0 is the register for PORTB
  PCICR |= (1 << PCIE0);     // Enable PC Interrupt for Port B (PCIE0)
  PCMSK0 |= (1 << PCINT0);   // Enable interrupt on D8 (PCINT0)
  PCMSK0 |= (1 << PCINT1);   // Enable interrupt on D9 (PCINT1)
}

void loop() {
  // Check if a command has been received and parsed (non-blocking)
  if (stringComplete) {
    handleIncomingCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  // Non-blocking Encoder Data Transmission
  if (millis() - lastEncoderSendTime >= ENCODER_SEND_INTERVAL_MS) {
    sendEncoderData();
    lastEncoderSendTime = millis();
  }
}


void sendEncoderData() {
  double rotFL = EncoderFL.getRotations();
  double rotFR = EncoderFR.getRotations();
  double rotBL = EncoderBL.getRotations();
  double rotBR = EncoderBR.getRotations();

  String serializedData = "M,";
  serializedData += String(rotFL, 3);
  serializedData += ",";
  serializedData += String(rotFR, 3);
  serializedData += ",";
  serializedData += String(rotBL, 3);
  serializedData += ",";
  serializedData += String(rotBR, 3);
  
  Serial.println(serializedData);
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

  if (fullCommand.length() < 3) return; // Minimum length for 'T,x' or 'R,x'

  // Get the Message Type ID (First character)
  char typeId = fullCommand.charAt(0);
  // dataPayload starts after the ID and comma (e.g., "T,0.5,0.0,0.1" -> "0.5,0.0,0.1")
  String dataPayload = fullCommand.substring(2); 
  
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
  String xStr = payload.substring(0, comma1);
  String yStr = payload.substring(comma1 + 1, comma2);
  String zStr = payload.substring(comma2 + 1);
  
  x_vel = xStr.toFloat();
  y_vel = yStr.toFloat();
  z_angular = zStr.toFloat();

  drivetrain.drive(x_vel, y_vel, z_angular);
}

// PCINT0_vect handles interrupts for PORTB (Pins D8 to D13)
ISR(PCINT0_vect) {
  // Check which pin caused the interrupt and call the update function
  // In this simple case, we know it's Encoder1, so we just call its update() method.
  EncoderFL.update();
  EncoderFR.update();
  EncoderBL.update();
  EncoderBR.update();
}