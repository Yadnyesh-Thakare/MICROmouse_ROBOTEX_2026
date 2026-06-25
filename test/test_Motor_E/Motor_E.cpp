#include <Arduino.h>
#include <SparkFun_TB6612.h>

// --- Encoder Pins ---
#define ENCL_A 19
#define ENCL_B 23
#define ENCR_A 2
#define ENCR_B 15

// --- Motor Pins ---
#define PWMR 26
#define RIN1 4
#define RIN2 25
#define LIN1 17
#define LIN2 5
#define PWML 18 
#define STBY 16

// Motor Offsets (Change to -1 if a motor spins backward)
const int offsetL = -1;
const int offsetR = 1;

// Initialize Motors
Motor motorLeft = Motor(LIN1, LIN2, PWML, offsetL, STBY);
Motor motorRight = Motor(RIN1, RIN2, PWMR, offsetR, STBY);

// --- Odometry Constants ---
const float ENCODER_CPR = 700.0;
const float WHEEL_DIAMETER_MM = 38.0;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER_MM;

// Encoder Tick Counters
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Left Interrupt Service Routine
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENCL_B) == LOW) {
    leftEncoderCount = leftEncoderCount + 1;
  } else {
    leftEncoderCount = leftEncoderCount - 1;
  }
}

// Right Interrupt Service Routine
void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENCR_B) == LOW) {
    rightEncoderCount = rightEncoderCount + 1;
  } else {
    rightEncoderCount = rightEncoderCount - 1;
  }
}

// --- Dynamic Speed Distance Control ---
void driveDistance(float targetDistance_mm, int targetMaxSpeed) {
  long targetTicks = (targetDistance_mm / WHEEL_CIRCUMFERENCE) * ENCODER_CPR;
  
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  Serial.print("Target Ticks: ");
  Serial.println(targetTicks);

  float Kp = 0.9; 
  
  // Speed Profiling Variables
  int minSpeed = 50; // Lowest PWM where the motors actually move
  long accelTicks = targetTicks * 0.25; // Ramp up over the first 15% of the distance
  long decelTicks = targetTicks * 0.35; // Ramp down over the last 25% of the distance

  while (true) {
    long currentLeft = abs(leftEncoderCount);
    long currentRight = abs(rightEncoderCount);
    long averageTicks = (currentLeft + currentRight) / 2;
    
    // Check if destination is reached
    if (averageTicks >= targetTicks) {
      break; 
    }

    // --- 1. Calculate Dynamic Base Speed ---
    int currentBaseSpeed = 0;

    if (averageTicks < accelTicks) {
      // We are in the Acceleration Phase
      // map(value, fromLow, fromHigh, toLow, toHigh)
      currentBaseSpeed = map(averageTicks, 0, accelTicks, minSpeed, targetMaxSpeed);
    } 
    else if (averageTicks > (targetTicks - decelTicks)) {
      // We are in the Deceleration Phase
      long remainingTicks = targetTicks - averageTicks;
      currentBaseSpeed = map(remainingTicks, decelTicks, 0, targetMaxSpeed, minSpeed);
    } 
    else {
      // We are in the Cruise Phase
      currentBaseSpeed = targetMaxSpeed;
    }

    // --- 2. Apply Straight-Line Correction ---
    long error = currentLeft - currentRight;
    int adjustment = error * Kp;

    int leftSpeed = currentBaseSpeed - adjustment;
    int rightSpeed = currentBaseSpeed + adjustment;

    // Constrain speeds to valid bounds
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // Command the motors
    motorLeft.drive(leftSpeed);
    motorRight.drive(rightSpeed);

    delay(5); 
  }

  // Target reached, apply brakes firmly
  motorLeft.brake();
  motorRight.brake();
  Serial.println("Movement Complete.");
}

void setup() {
  Serial.begin(115200);
  
  pinMode(ENCL_A, INPUT_PULLUP);
  pinMode(ENCL_B, INPUT_PULLUP);
  pinMode(ENCR_A, INPUT_PULLUP);
  pinMode(ENCR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCL_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCR_A), rightEncoderISR, RISING);
  
  Serial.println("Place robot on ground. Starting in 3 seconds...");

}

void loop() {
  delay(3000); 
  
  Serial.println("Executing 300mm Profile Test...");
  
  // Drive 300mm, with a peak cruise speed of 150
  driveDistance(170.0, 250); 
}