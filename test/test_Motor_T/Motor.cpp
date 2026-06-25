#include <Arduino.h>
#include <SparkFun_TB6612.h> // For the motor driver

// --- Pin Definitions ---
// Fixed the syntax (removed ':-' and replaced with spaces)

// Left Motor (Treated as Motor A)
#define PWMR 26
#define RIN1 4
#define RIN2 25

// Right Motor (Treated as Motor B)
#define LIN1 17
#define LIN2 5
#define PWML 18 

// Standby Pin
#define STBY 16

// --- Motor Offsets ---
// These are constants that help fix wiring issues. 
// If a motor spins backward when it should go forward, change its offset from 1 to -1.
const int offsetL = -1;
const int offsetR = 1;

// --- Initialize Motors ---
// Passing the pins into the library's Motor class
Motor motorLeft = Motor(LIN1, LIN2, PWML, offsetL, STBY);
Motor motorRight = Motor(RIN1, RIN2, PWMR, offsetR, STBY);

void setup() {
  // Start the serial monitor so we can see what the code is currently doing
  Serial.begin(115200);
  delay(1000); 
  Serial.println("TB6612 Motor Test Starting...");
}

void loop() {
  // 1. Move Forward
  Serial.println("Moving Forward");
  // The drive() function takes a speed between -255 and 255.
  motorRight.drive(150);  
  motorLeft.drive(150);
  delay(2000); // Let it run for 2 seconds

  // // 4. Brake / Stop
  Serial.println("Braking");
  motorLeft.brake();
  motorRight.brake();
  delay(2000); // Stop for 2 seconds before repeating the loop
}
