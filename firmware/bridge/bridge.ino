#include "inc/Encoder.h"
#include "inc/Mecanum.h"

#define ENCODER_1_A_PIN 8 
#define ENCODER_1_B_PIN 9 
#define MOTOR_FL_PIN 2
#define MOTOR_FR_PIN 3
#define MOTOR_BL_PIN 4
#define MOTOR_BR_PIN 5


Encoder Encoder1(ENCODER_1_A_PIN, ENCODER_1_B_PIN);
MecanumDrive drivetrain(
  MOTOR_FL_PIN,
  MOTOR_FR_PIN,
  MOTOR_BL_PIN,
  MOTOR_BR_PIN
);

void setup() {
  Serial.begin(115200);
  Serial.println("Modular Encoder Counter Initialized.");

  Encoder1.init(); // Configure pins using the class method
  drivetrain.init();
  
  // Hardware-specific: Enable Pin Change Interrupts for Port B (D8-D13)
  // PCMSK0 is the register for PORTB
  PCICR |= (1 << PCIE0);     // Enable PC Interrupt for Port B (PCIE0)
  PCMSK0 |= (1 << PCINT0);   // Enable interrupt on D8 (PCINT0)
  PCMSK0 |= (1 << PCINT1);   // Enable interrupt on D9 (PCINT1)
}

void loop() {
  // Call the public getter method
  double currentE1 = Encoder1.getRotations();
  
  // Print the counts to the Serial Monitor/RPi
  Serial.print("Motor Rotations: ");
  Serial.println(currentE1);

  // TODO: Real values
  drivetrain.drive(0.1, 0.1, 0.1);
  
  delay(1000);
}


// PCINT0_vect handles interrupts for PORTB (Pins D8 to D13)
ISR(PCINT0_vect) {
  // Check which pin caused the interrupt and call the update function
  // In this simple case, we know it's Encoder1, so we just call its update() method.
  Encoder1.update();
}