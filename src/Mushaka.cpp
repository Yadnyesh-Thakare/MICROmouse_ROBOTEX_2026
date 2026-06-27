#include <Arduino.h> // For Arduino functions
#include <Wire.h>  // For I2C communication
#include <PINS.h> // For the defined pins 
#include <SparkFun_TB6612.h> // For the motor driver
#include <Adafruit_VL53L0X.h> // For the distance sensor
#include <VL53L0X.h> // For the distance sensor by pololu


//? ===========================================================================================//

// --- Initialize Motors ---
// Passing the pins into the library's Motor class
Motor motorLeft = Motor(LIN1, LIN2, PWML, offsetL, STBY);
Motor motorRight = Motor(RIN1, RIN2, PWMR, offsetR, STBY);

//? ===========================================================================================//

//* ==========================================
//* ODOMETRY CONSTANTS
//* ==========================================

const float ENCODER_CPR        = 2800.0;
const float WHEEL_DIAMETER_MM  = 50.0;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER_MM;  // ~141.37 mm
const float WHEELBASE_MM       = 100.0;   // center-to-center wheel distance

volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;

//* ==========================================
//* WALL PID PARAMETERS  — tune as needed
//* ==========================================
float Kp_wall = 0.9;
float Ki_wall = 0.0;
float Kd_wall = 0.2;

const int WallLostThreshold_U = 135;        // upper threshold for side wall
const int WallLostThreshold_L = 125;        // lower threshold for side wall 
const int FrontobstacleThreshold_U  = 265;  // front wall detection 

const int targetWallDistance = 95;          // Side wall distance for pid 
const int FrontStopThreshold = 85;          // Front wall detection threshold for stoping 
const int wallLostTurnSpeed  = 200;         // outer-wheel PWM for the re-acquire arc turn
const int baseSpeed = 220;                  //* Base forward speed (0-255)

float previousWallError = 0;
float wallIntegral      = 0;

// Sync-correction gain used while decelerating, to keep both wheels
// slowing down at the same actual rate (measured via encoders)
float Kp_sync = 2.5;

//* ENCODER ISRs  (4x quadrature resolution)

void IRAM_ATTR leftEncoderAISR() 
{
  leftEncoderCount += (digitalRead(ENCL_A) == digitalRead(ENCL_B)) ? -1 : 1;
}
void IRAM_ATTR leftEncoderBISR() 
{
  leftEncoderCount += (digitalRead(ENCL_A) != digitalRead(ENCL_B)) ? -1 : 1;
}
void IRAM_ATTR rightEncoderAISR() 
{
  rightEncoderCount += (digitalRead(ENCR_A) == digitalRead(ENCR_B)) ? -1 : 1;
}
void IRAM_ATTR rightEncoderBISR() 
{
  rightEncoderCount += (digitalRead(ENCR_A) != digitalRead(ENCR_B)) ? -1 : 1;
}

//? ===========================================================================================//

// Create data structures to hold the measurements
Adafruit_VL53L0X sensorCenter = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLeft   = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight  = Adafruit_VL53L0X();

// Global sensor distance variables (in mm)
int leftDist_mm   ;
int centerDist_mm ;
int rightDist_mm  ;

// SENSOR INITIALISATION

void initSensors() 
{
  pinMode(XSHUT_CENTER, OUTPUT);
  pinMode(XSHUT_LEFT,   OUTPUT);
  pinMode(XSHUT_RIGHT,  OUTPUT);

  // Pull all XSHUT pins LOW to reset every sensor
  digitalWrite(XSHUT_CENTER, LOW);
  digitalWrite(XSHUT_LEFT,   LOW);
  digitalWrite(XSHUT_RIGHT,  LOW);
  delay(10);

  // Bring up sensors one-by-one and assign unique I2C addresses
  digitalWrite(XSHUT_CENTER, HIGH); delay(10);
  if (!sensorCenter.begin(ADDRESS_CENTER)) {
    Serial.println("FATAL: Center sensor failed!"); while (1);
  }

  digitalWrite(XSHUT_LEFT, HIGH); delay(10);
  if (!sensorLeft.begin(ADDRESS_LEFT)) {
    Serial.println("FATAL: Left sensor failed!"); while (1);
  }

  digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
  if (!sensorRight.begin(ADDRESS_RIGHT)) {
    Serial.println("FATAL: Right sensor failed!"); while (1);
  }

  Serial.println("All sensors initialised.");
}

//? ===========================================================================================//


void readSensors()
{
  VL53L0X_RangingMeasurementData_t mLeft, mCenter, mRight;

  sensorLeft.rangingTest(&mLeft,     false);
  sensorCenter.rangingTest(&mCenter, false);
  sensorRight.rangingTest(&mRight,   false);

  //* Cap out-of-range readings to safe defaults
  leftDist_mm   = (mLeft.RangeStatus   != 4) ? mLeft.RangeMilliMeter   : 200;
  centerDist_mm = (mCenter.RangeStatus != 4) ? mCenter.RangeMilliMeter : 800;
  rightDist_mm  = (mRight.RangeStatus  != 4) ? mRight.RangeMilliMeter  : 200;

}

int WallCase() 
{
  static bool leftWallVisible = true;
  static bool centerObstacleVisible = true; 
  static bool rightWallVisible = true;

    // Left Hysteresis
    if (leftDist_mm > WallLostThreshold_U)      leftWallVisible = false;
    else if (leftDist_mm < WallLostThreshold_L) leftWallVisible = true;

    // Center Hysteresis 
    if (centerDist_mm > FrontobstacleThreshold_U) centerObstacleVisible = false;
    else centerObstacleVisible = true;

    // Right Hysteresis
    if (rightDist_mm > WallLostThreshold_U)      rightWallVisible = false;
    else if (rightDist_mm < WallLostThreshold_L) rightWallVisible = true;
            //! from line no. 36 - 39
            //! | bit wise or 
            // Using your 130-120 mm threshold for left wall loss detection
            // Using your 250 mm threshold for front obstacle detection
            // Using your 130-120 mm threshold for right wall loss detection
  
  int mask = ((!leftWallVisible) << 2) | ((!centerObstacleVisible) << 1) | (!rightWallVisible);
  //? Map the mask result to your original 1-7 case numbers
  //?              Mask:  0b111, 0b011, 0b010, 0b101, 0b001, 0b110, 0b010, 0b000
    const int caseMap[8] = { 7  ,  3  ,   2  ,   5  ,   1  ,   6  ,   4  ,   0  };    
    return caseMap[mask];

  //* ==========================================
  //* CLEARANCE COMBINATIONS
  //* ==========================================

  // Case 1: ONLY left is clear (Front and Right wall present)
  // Case 2: ONLY Front is clear (Right and left wall present)
  // Case 3: ONLY Right is clear (Front and left wall present)
  // Case 4: ONLY Front and Left are clear (Right wall present)
  // Case 5: ONLY Front and Right are clear (Left wall present)
  // Case 6: ONLY Left and Right are clear (Front wall present)
  // Case 7: No way left (Front, Left, and Right walls present)
  // case 0: Default fallback if a configuration doesn't match your 7 cases


}

//? ===========================================================================================//

// RESET WALL PID STATE //! form line no. 32 -34
void resetWallPID() 
{
  wallIntegral      = 0.0;
  previousWallError = 0.0;
}

//? ===========================================================================================//

// MPU6050 GYROSCOPE FUNCTIONS

float gyroBiasZ = 0.0;  // stored in dps
float yaw = 0.0;
unsigned long lastUpdate = 0;

//* ======================================
void WKnC_MPU() 
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Gyro ±500 dps
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission(true);

  delay(10);

  Serial.println("Keep robot completely still...");
  delay(1000);

  float sum = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2);
    sum += (int16_t)(Wire.read() << 8 | Wire.read()) / 65.5;
    delay(10);
  }
  gyroBiasZ = sum / 200.0;
  Serial.print("Gyro Bias Z = ");
  Serial.println(gyroBiasZ);
}

//* ======================================
void updateYAW() 
{
  if (micros() - lastUpdate >= 1000) {

    float dt = (micros() - lastUpdate) / 1000000.0;
    lastUpdate = micros();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2);

    int16_t rawGz = (Wire.read() << 8) | Wire.read();
    float gyroZ = (rawGz / 65.5) - gyroBiasZ;

    if (fabs(gyroZ) < 0.4) gyroZ = 0;

    yaw += gyroZ * dt;

    //Serial.print("Gyro Z: ");
    //Serial.print(gyroZ);
    //Serial.print(" dps   Yaw: ");
    //Serial.println(yaw);
  }
}

//? ===========================================================================================//

void driveDistance(float targetDistance_mm, int speed) {
  // Calculate total ticks required
  long targetTicks = (long)((targetDistance_mm / WHEEL_CIRCUMFERENCE) * ENCODER_CPR);
  
  // Reset encoders
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  // P-Controller Constant (Adjust this based on your robot's weight/friction)
  const float Kp = 1.2; 

  // Continue loop until target distance is met
  while (true) {
    long currentLeft = abs(leftEncoderCount);
    long currentRight = abs(rightEncoderCount);
    long averageTicks = (currentLeft + currentRight) / 2;
    
    if (averageTicks >= targetTicks) break;

    // --- Straight-Line Correction ---
    // Error is difference between encoders. 
    // Positive error means Left is ahead, so we slow Left/speed up Right.
    long error = currentLeft - currentRight;
    int adjustment = (int)(error * Kp);

    // Calculate motor speeds
    int leftSpeed  = speed - adjustment;
    int rightSpeed = speed + adjustment;

    // Command motors (constrained to valid PWM range)
    motorLeft.drive(constrain(leftSpeed, 0, 255));
    motorRight.drive(constrain(rightSpeed, 0, 255));
  }

  // Stop motors immediately
  motorLeft.brake();
  motorRight.brake();
}

//? ===========================================================================================//


void driveForward_2_Wall(int baseSpeed)
{
  resetWallPID();
  Serial.println("Driving forward until front wall is detected...");

  while (true)
  {
    readSensors();

    // 3. Wall Alignment PID Calculation
    bool leftWallLost  = (leftDist_mm  >= WallLostThreshold_L);
    bool rightWallLost = (rightDist_mm >= WallLostThreshold_L);
    float wallError    = 0.0f;

    if (!leftWallLost && !rightWallLost) {
      wallError = (float)(leftDist_mm - rightDist_mm);
    }  
    else if (!leftWallLost && rightWallLost) {
      wallError = (float)(leftDist_mm - targetWallDistance) * 2.0f; 
    } 
    else if (leftWallLost && !rightWallLost) {
      wallError = (float)(targetWallDistance - rightDist_mm) * 2.0f;
    }

    wallIntegral += wallError;
    float wallDerivative = wallError - previousWallError;
    float wallCorrection = (Kp_wall * wallError) + (Ki_wall * wallIntegral) + (Kd_wall * wallDerivative);
    previousWallError    = wallError;

    int leftSpeed  = baseSpeed - (int)wallCorrection;
    int rightSpeed = baseSpeed + (int)wallCorrection;

    motorLeft.drive(constrain(leftSpeed, 0, 255));
    motorRight.drive(constrain(rightSpeed, 0, 255));
    delay(1);
  }

  // Active Momentum Termination Pulse
  motorLeft.drive(-220);
  motorRight.drive(-220);
  delay(40); 
 
  motorLeft.brake();
  motorRight.brake();
  resetWallPID();
  delay(100); // Allow physical bounce to dissipate
}

//? ===========================================================================================//

// ---------------------------------------------------------------------------
// turnArc — generic arc turn for any angle
//
// Parameters:
//   angleDeg      — turn angle in degrees (e.g. 90, 180)
//   turnRadius_mm — radius to robot centre (use WHEELBASE_MM for tight arc)
//   maxOuterSpeed — peak PWM for the faster (outer) wheel
//   turnLeft      — true = turn left (left=inner), false = turn right (right=inner)
//
// Inner wheel arc = (R - WHEELBASE/2) * angleRad
// Outer wheel arc = (R + WHEELBASE/2) * angleRad
// Speed ratio     = innerArc / outerArc  (< 1.0, keeps arc smooth)
// ---------------------------------------------------------------------------

void turnArc(float angleDeg, float turnRadius_mm, int maxOuterSpeed, bool turnLeft) 
{

  float angleRad    = angleDeg * (PI / 180.0f);
  float innerRadius = turnRadius_mm - WHEELBASE_MM / 2.0f;
  float outerRadius = turnRadius_mm + WHEELBASE_MM / 2.0f;

  float innerArc_mm = innerRadius * angleRad;
  float outerArc_mm = outerRadius * angleRad;

  long innerTargetTicks = (innerArc_mm / WHEEL_CIRCUMFERENCE) * ENCODER_CPR;
  long outerTargetTicks = (outerArc_mm / WHEEL_CIRCUMFERENCE) * ENCODER_CPR;

  float speedRatio = innerArc_mm / outerArc_mm;  // < 1.0

  //Serial.print("turnArc ");   Serial.print(angleDeg);
  //Serial.print("° — inner ticks: "); Serial.print(innerTargetTicks);
  //Serial.print("  outer ticks: ");   Serial.println(outerTargetTicks);
  //Serial.print("Speed ratio (inner/outer): "); Serial.println(speedRatio);

  noInterrupts();
  leftEncoderCount  = 0;
  rightEncoderCount = 0;
  interrupts();

  const int minOuterSpeed = 60;
  int       minInnerSpeed = max(30, (int)(minOuterSpeed * speedRatio));

  long accelTicks = outerTargetTicks * 0.35;
  long decelTicks = outerTargetTicks * 0.35;

  while (true) {
    noInterrupts();
    long currentLeft  = abs(leftEncoderCount);
    long currentRight = abs(rightEncoderCount);
    interrupts();

    // Outer wheel is the progress reference (it travels the longer arc)
    long progressTicks = turnLeft ? currentRight : currentLeft;
    if (progressTicks >= outerTargetTicks) break;

    // --- Trapezoidal speed profile for outer wheel ---
    int outerSpeed;
    if (progressTicks < accelTicks) {
      outerSpeed = map(progressTicks, 0, accelTicks, minOuterSpeed, maxOuterSpeed);
    } else if (progressTicks > (outerTargetTicks - decelTicks)) {
      long remaining = outerTargetTicks - progressTicks;
      outerSpeed = map(remaining, decelTicks, 0, maxOuterSpeed, minOuterSpeed);
    } else {
      outerSpeed = maxOuterSpeed;
    }

    // --- Inner wheel speed from ratio ---
    int innerSpeed = (int)(outerSpeed * speedRatio);
    innerSpeed = constrain(innerSpeed, minInnerSpeed, 255);

    // --- Closed-loop correction on inner wheel ---
    long outerProgress = turnLeft ? currentRight : currentLeft;
    long innerCurrent  = turnLeft ? currentLeft  : currentRight;
    long expectedInner = (long)(outerProgress * speedRatio);
    long innerError    = expectedInner - innerCurrent;  // +ve = inner lagging
    int  correction    = (int)(innerError * 0.4f);
    innerSpeed = constrain(innerSpeed + correction, 0, 255);

    // --- Drive motors: inner wheel is the slower one ---
    if (turnLeft) {
      motorLeft.drive(innerSpeed);   // left  = inner
      motorRight.drive(outerSpeed);  // right = outer
    } else {
      motorLeft.drive(outerSpeed);   // left  = outer
      motorRight.drive(innerSpeed);  // right = inner
    }

    delay(5);
  }

  //driveDistance(200,180);

  motorLeft.brake();
  motorRight.brake();
  Serial.print("turnArc "); Serial.print(angleDeg); Serial.println("° complete.");
}

//* ---------------------------------------------------------------------------
//* Convenience wrappers
//* ---------------------------------------------------------------------------

void turnArc90Left (float turnRadius_mm, int maxOuterSpeed) {
  turnArc(90.0f,  turnRadius_mm, maxOuterSpeed, true);
}
void turnArc90Right(float turnRadius_mm, int maxOuterSpeed) {
  turnArc(90.0f,  turnRadius_mm, maxOuterSpeed, false);
}
void turnArc180Left(float turnRadius_mm, int maxOuterSpeed) {
  turnArc(180.0f, turnRadius_mm, maxOuterSpeed, true);
}
void turnArc180Right(float turnRadius_mm, int maxOuterSpeed) {
  turnArc(180.0f, turnRadius_mm, maxOuterSpeed, false);
}

//? ===========================================================================================//

//! ===========================================================================================//

void setup()
{
    Serial.begin(115200); 
    Wire.begin();               // change to Wire.begin(SDA, SCL) if needed
    Wire.setClock(400000);

    while (!Serial) { delay(1); }

    //! Encoder pins
    pinMode(ENCL_A, INPUT_PULLUP);
    pinMode(ENCL_B, INPUT_PULLUP);
    pinMode(ENCR_A, INPUT_PULLUP);
    pinMode(ENCR_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCL_A), leftEncoderAISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCL_B), leftEncoderBISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCR_A), rightEncoderAISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCR_B), rightEncoderBISR, CHANGE);

    initSensors();    //!-- Initialize the VL53L0X sensors

    //? ======================================

    yaw = 0;
    lastUpdate = micros();
    WKnC_MPU();        //!-- Wake up and Calibrate the MPU6050 gyroscope
    
    //? ======================================
    
    pinMode(LEDL, OUTPUT);
    pinMode(LEDR, OUTPUT);
    pinMode(BTNL, INPUT);    //! LED pins
    pinMode(BTNR, INPUT);  

    //? ======================================
    
    Serial.println("Place robot on ground. Starting in 3 seconds...");
    delay(3000);

    digitalWrite(LEDL, LOW);
    digitalWrite(LEDR, LOW);

}


unsigned long lastBlinkTime = 0;  //! for LED blinking timing
bool ledState = false;

//! ===========================================================================================//

void loop() 
  {
    //? ======================================

    updateYAW();     //!-- Update the yaw angle

    //? ======================================
   
    readSensors();
    int check = WallCase();  //*-- Read the distance sensors and get the clearance case 
    {
      Serial.println("No valid wall configuration detected. Stopping.");
      motorLeft.brake();
      motorRight.brake();
    }
    Serial.print("Wall Case: ");
    Serial.println(check);

    turnArc90Left(50, 220);  //*  Call the Arc180Left function with the clearance case
    delay(2000);
}
