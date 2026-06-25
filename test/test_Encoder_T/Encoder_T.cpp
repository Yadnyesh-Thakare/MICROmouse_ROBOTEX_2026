#include <Arduino.h>

// Define Encoder Pins
#define ENCL_A 19
#define ENCL_B 23

#define ENCR_A 2
#define ENCR_B 15

// Variables to store the encoder counts
// Marked as 'volatile' because they are modified inside an Interrupt Service Routine (ISR)
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Variables to keep track of previous counts for printing
long prevLeftCount = 0;
long prevRightCount = 0;

// Interrupt Service Routine for Left Encoder
// IRAM_ATTR is used if you are running this on an ESP32 to load the ISR into RAM
// Interrupt Service Routine for Left Encoder
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENCL_B) == LOW) {
    leftEncoderCount = leftEncoderCount + 1; // Replaced ++
  } else {
    leftEncoderCount = leftEncoderCount - 1; // Replaced --
  }
}

// Interrupt Service Routine for Right Encoder
void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENCR_B) == LOW) {
    rightEncoderCount = rightEncoderCount + 1; // Replaced ++
  } else {
    rightEncoderCount = rightEncoderCount - 1; // Replaced --
  }
}

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  
  // Set encoder pins as inputs with internal pullups to prevent floating states
  pinMode(ENCL_A, INPUT_PULLUP);
  pinMode(ENCL_B, INPUT_PULLUP);
  pinMode(ENCR_A, INPUT_PULLUP);
  pinMode(ENCR_B, INPUT_PULLUP);

  // Attach interrupts to Channel A of both encoders
  // Triggering on 'RISING' edge gives 1x resolution. 
  attachInterrupt(digitalPinToInterrupt(ENCL_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCR_A), rightEncoderISR, RISING);
  
  Serial.println("Encoder CPR Calibration Tool Ready!");
  Serial.println("Rotate the wheel EXACTLY 1 full revolution (360 degrees) and read the count.");
}

void loop() {
  // Only print to the Serial Monitor if the count has changed
  if (leftEncoderCount != prevLeftCount || rightEncoderCount != prevRightCount) {
    
    Serial.print("Left Encoder: ");
    Serial.print(leftEncoderCount);
    Serial.print("  |  Right Encoder: ");
    Serial.println(rightEncoderCount);
    
    // Update previous counts
    prevLeftCount = leftEncoderCount;
    prevRightCount = rightEncoderCount;
  }
  
  // Small delay to prevent Serial Monitor flooding
  delay(10); 
}