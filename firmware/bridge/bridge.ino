#include "inc/Encoder.h"
#include "inc/Motor.h"

// Define the pins for the specific hardware (D8 and D9 are on Port B)
#define ENCODER_1_A_PIN 8 
#define ENCODER_1_B_PIN 9 

Encoder Encoder1(ENCODER_1_A_PIN, ENCODER_1_B_PIN);

void setup() {
  Serial.begin(115200);
  Serial.println("Modular Encoder Counter Initialized.");

  Encoder1.init(); // Configure pins using the class method
  
  // Hardware-specific: Enable Pin Change Interrupts for Port B (D8-D13)
  // PCMSK0 is the register for PORTB
  PCICR |= (1 << PCIE0);     // Enable PC Interrupt for Port B (PCIE0)
  PCMSK0 |= (1 << PCINT0);   // Enable interrupt on D8 (PCINT0)
  PCMSK0 |= (1 << PCINT1);   // Enable interrupt on D9 (PCINT1)
}

void loop() {
  // Call the public getter method
  long currentE1 = Encoder1.getCount();
  
  // Print the counts to the Serial Monitor/RPi
  Serial.print("E1: ");
  Serial.println(currentE1);
  
  delay(100);
}


// PCINT0_vect handles interrupts for PORTB (Pins D8 to D13)
ISR(PCINT0_vect) {
  // Check which pin caused the interrupt and call the update function
  // In this simple case, we know it's Encoder1, so we just call its update() method.
  Encoder1.update();
}