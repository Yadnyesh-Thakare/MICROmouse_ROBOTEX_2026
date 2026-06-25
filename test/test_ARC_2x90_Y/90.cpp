#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <SparkFun_TB6612.h>

// ==========================================
// ENCODER PINS
// ==========================================
#define ENCL_A 19
#define ENCL_B 23
#define ENCR_A 2
#define ENCR_B 15

// ==========================================
// MOTOR PINS
// ==========================================
#define PWMR 26
#define RIN1 4
#define RIN2 25
#define LIN1 17
#define LIN2 5
#define PWML 18
#define STBY 16

const int offsetL = -1;
const int offsetR = 1;

Motor motorLeft  = Motor(LIN1, LIN2, PWML, offsetL, STBY);
Motor motorRight = Motor(RIN1, RIN2, PWMR, offsetR, STBY);

// ==========================================
// SENSOR PINS & I2C
// ==========================================
#define XSHUT_CENTER 27
#define XSHUT_LEFT   13
#define XSHUT_RIGHT  33

#define ADDRESS_CENTER 0x30
#define ADDRESS_LEFT   0x31
#define ADDRESS_RIGHT  0x32

Adafruit_VL53L0X sensorCenter = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLeft   = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight  = Adafruit_VL53L0X();

// ==========================================
// ODOMETRY CONSTANTS
// ==========================================
const float ENCODER_CPR        = 2800.0;
const float WHEEL_DIAMETER_MM  = 45.0;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER_MM;  // ~141.37 mm
const float WHEELBASE_MM       = 100.0;   // center-to-center wheel distance

volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;

// ==========================================
// WALL-CENTERING PID PARAMETERS  — tune as needed
// ==========================================

float Kp_wall = 0.5; //---0.7 = G  ||--- 0.6 = G   || --- 0.5 == VG 
float Ki_wall = 0.0; //---0        ||              || --- 
float Kd_wall = 0.4; //---0.4      ||--- 0.4       || --- 0.4

int   targetWallDistance = 95;    // mm L and R distance from wall 
int   obstacleThreshold  = 70;    // mm — front stop distance
int   WallLostThreshold  = 125;   // mm - after this distane the bot take it as no wall 

float previousWallError = 0;
float wallIntegral      = 0;

// ==========================================
// ENCODER ISRs  (4x quadrature resolution)
// ==========================================
void IRAM_ATTR leftEncoderAISR() {
  leftEncoderCount += (digitalRead(ENCL_A) == digitalRead(ENCL_B)) ? -1 : 1;
}
void IRAM_ATTR leftEncoderBISR() {
  leftEncoderCount += (digitalRead(ENCL_A) != digitalRead(ENCL_B)) ? -1 : 1;
}
void IRAM_ATTR rightEncoderAISR() {
  rightEncoderCount += (digitalRead(ENCR_A) == digitalRead(ENCR_B)) ? -1 : 1;
}
void IRAM_ATTR rightEncoderBISR() {
  rightEncoderCount += (digitalRead(ENCR_A) != digitalRead(ENCR_B)) ? -1 : 1;
}

// ==========================================
// SENSOR INITIALISATION
// ==========================================
void initSensors() {
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

// ==========================================
// READ SENSORS  (returns false if obstacle)
// ==========================================
// Fills leftDist_mm, centerDist_mm, and rightDist_mm by reference.
// Returns true  → clear to drive
// Returns false → front obstacle detected, caller should stop
int readSensors(int &leftDist_mm, int &centerDist_mm, int &rightDist_mm)
{
  VL53L0X_RangingMeasurementData_t mLeft, mCenter, mRight;

  sensorLeft.rangingTest(&mLeft,     false);
  sensorCenter.rangingTest(&mCenter, false);
  sensorRight.rangingTest(&mRight,   false);

  //* Cap out-of-range readings to safe defaults
  leftDist_mm   = (mLeft.RangeStatus   != 4) ? mLeft.RangeMilliMeter   : 200;
  centerDist_mm = (mCenter.RangeStatus != 4) ? mCenter.RangeMilliMeter : 800;
  rightDist_mm  = (mRight.RangeStatus  != 4) ? mRight.RangeMilliMeter  : 200;

  //* Compress 7 conditional cases into a single 3-bit integer lookup table map
  //* Bit 2: Left, Bit 1: Front, Bit 0: Right 
  int mask = ((leftDist_mm   > WallLostThreshold)  << 2) |
             ((centerDist_mm > obstacleThreshold)  << 1) | 
             ((rightDist_mm  > WallLostThreshold)  << 0);
  
             //? | bit wise or 
  //* Map the mask result to your original 1-7 case numbers
  //*              Mask:  0b000, 0b001, 0b010, 0b011, 0b100, 0b101, 0b110, 0b111
  const int caseMap[8] = {    7,     3,     2,     5,     1,     6,     4,     0 };

  return caseMap[mask];
}


// ==========================================
// RESET WALL PID STATE
// ==========================================
void resetWallPID()
{
  wallIntegral      = 0.0;
  previousWallError = 0.0;
}

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
void turnArc(float angleDeg, float turnRadius_mm, int maxOuterSpeed, bool turnLeft) {

  float angleRad    = angleDeg * (PI / 180.0f);
  float innerRadius = turnRadius_mm - WHEELBASE_MM / 2.0f;
  float outerRadius = turnRadius_mm + WHEELBASE_MM / 2.0f;

  float innerArc_mm = innerRadius * angleRad;
  float outerArc_mm = outerRadius * angleRad;

  long innerTargetTicks = (innerArc_mm / WHEEL_CIRCUMFERENCE) * ENCODER_CPR;
  long outerTargetTicks = (outerArc_mm / WHEEL_CIRCUMFERENCE) * ENCODER_CPR;

  float speedRatio = innerArc_mm / outerArc_mm;  // < 1.0

  Serial.print("turnArc ");   Serial.print(angleDeg);
  Serial.print("° — inner ticks: "); Serial.print(innerTargetTicks);
  Serial.print("  outer ticks: ");   Serial.println(outerTargetTicks);
  Serial.print("Speed ratio (inner/outer): "); Serial.println(speedRatio);

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

  motorLeft.brake();
  motorRight.brake();
  Serial.print("turnArc "); Serial.print(angleDeg); Serial.println("° complete.");
}

// ---------------------------------------------------------------------------
// Convenience wrappers
// ---------------------------------------------------------------------------
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


// ==========================================
// DRIVE DISTANCE  — forward run, centered between left & right walls
// ==========================================
// targetDistance_mm : how far to travel
// targetMaxSpeed    : peak PWM (0-255)
//
// The encoder profiler sets the *base* speed for each loop tick.
// The wall PID computes a *correction* applied symmetrically around
// that base speed, keeping the robot centered between the left and
// right walls (error = leftDist_mm - rightDist_mm, driven to zero).
// A front obstacle instantly halts the run (returns early).
// ==========================================
void driveDistance(int targetDistance_mm, int targetMaxSpeed)
{

  long targetTicks = (targetDistance_mm / WHEEL_CIRCUMFERENCE) * ENCODER_CPR;

  noInterrupts();
  leftEncoderCount  = 0;
  rightEncoderCount = 0;
  interrupts();

  resetWallPID();

  Serial.print("Target Ticks: "); Serial.println(targetTicks);

  // Speed-profiling parameters
  int   minSpeed   = 100;
  long  accelTicks = targetTicks * 0.2;   // first 10% → ramp up
  long  decelTicks = targetTicks * 0.2;   // last  15% → ramp down

  while (true)
  {

    // --- Read encoders safely ---
    noInterrupts();
    long currentLeft  = abs(leftEncoderCount);
    long currentRight = abs(rightEncoderCount);
    interrupts();

    long averageTicks = (currentLeft + currentRight) / 2;

    // --- Distance goal reached? ---
    if (averageTicks >= targetTicks)
    break;

    // --- Read sensors; abort if obstacle ahead ---
    int leftDist_mm, centerDist_mm, rightDist_mm;
    
    int tem = readSensors(leftDist_mm, centerDist_mm, rightDist_mm);
    if (   readSensors(leftDist_mm, centerDist_mm, rightDist_mm)== 4 
        || readSensors(leftDist_mm, centerDist_mm, rightDist_mm)== 5 
        || readSensors(leftDist_mm, centerDist_mm, rightDist_mm)== 6 
        || readSensors(leftDist_mm, centerDist_mm, rightDist_mm)== 7)
    {
        Serial.println("OBSTACLE AHEAD — stopping drive.");
        motorLeft.brake();
        motorRight.brake();
        resetWallPID();
        return;   // Exit early; caller decides what to do next
    }
    
    //==========================================

    // ---- 1. Encoder-based speed profiling (base speed) ----
    int baseSpeed;
    if (averageTicks < accelTicks)
    {
      baseSpeed = map(averageTicks, 0, accelTicks, minSpeed, targetMaxSpeed);
    } else if (averageTicks > (targetTicks - decelTicks)) {
      long remaining = targetTicks - averageTicks;
      baseSpeed = map(remaining, decelTicks, 0, targetMaxSpeed, minSpeed);
    } else {
      baseSpeed = targetMaxSpeed;
    }

    // ---- 2. Wall-centering PID correction ----
    bool leftWallLost  = (leftDist_mm  >= WallLostThreshold);
    bool rightWallLost = (rightDist_mm >= WallLostThreshold);

    float wallError;
    if (!leftWallLost && !rightWallLost) 
    {
      // Both walls visible → center between them
      wallError = (float)(leftDist_mm - rightDist_mm);
    } 
    else if (!leftWallLost && rightWallLost) 
    {
    // Right wall gone → hold standoff off the left wall
    wallError = (float)(leftDist_mm - targetWallDistance);
    } 
    else if (leftWallLost && !rightWallLost) {
    // Left wall gone → hold standoff off the right wall
    wallError = (float)(targetWallDistance - rightDist_mm);
    } 
    else 
    {
    // Neither wall visible → no side reference, drive straight
    wallError = 0.0f;
    }
    wallIntegral += wallError;
    float wallDerivative = wallError - previousWallError;

    float wallCorrection = (Kp_wall * wallError)
                         + (Ki_wall * wallIntegral)
                         + (Kd_wall * wallDerivative);

    previousWallError = wallError;

    // Positive error → closer to right wall than left → curve left
    //   slow left wheel, speed up right wheel
    int leftSpeed  = baseSpeed - (int)wallCorrection;
    int rightSpeed = baseSpeed + (int)wallCorrection;

    leftSpeed  = constrain(leftSpeed,  0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    motorLeft.drive(leftSpeed);
    motorRight.drive(rightSpeed);

    if (leftWallLost ) 
      {
        turnArc90Left(WHEELBASE_MM, rightSpeed);  // re-acquire wall on the left
        resetWallPID();
      }

    if (rightWallLost ) 
      {
        turnArc90Right(WHEELBASE_MM, leftSpeed);  // re-acquire wall on the left
        resetWallPID();
      }
    //==========================================
    
    // Debug output
    Serial.print("Tem: ");   Serial.print(tem);
    Serial.print("Ticks: ");   Serial.print(averageTicks);
    Serial.print(" | Base: "); Serial.print(baseSpeed);
    Serial.print(" | LWall: ");Serial.print(leftDist_mm);
    Serial.print(" | RWall: ");Serial.print(rightDist_mm);
    Serial.print(" | Err: ");  Serial.print(wallError);
    Serial.print(" | Corr: "); Serial.print(wallCorrection);
    Serial.print(" | L: ");    Serial.print(leftSpeed);
    Serial.print(" | R: ");    Serial.println(rightSpeed);

    delay(1);
  }

  motorLeft.brake();
  motorRight.brake();
  Serial.println("Movement complete.");
}

// ==========================================
// SETUP
// ==========================================
void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  // Encoder pins
  pinMode(ENCL_A, INPUT_PULLUP);
  pinMode(ENCL_B, INPUT_PULLUP);
  pinMode(ENCR_A, INPUT_PULLUP);
  pinMode(ENCR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCL_A), leftEncoderAISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCL_B), leftEncoderBISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCR_A), rightEncoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCR_B), rightEncoderBISR, CHANGE);

  // Sensors
  Wire.begin();
  initSensors();

  Serial.println("Place robot on ground. Starting in 5 seconds...");
  delay(2000);
}

// ==========================================
// LOOP
// ==========================================
void loop()
{
  Serial.println("Starting run");
  driveDistance(720,250);

  Serial.println("Run complete. Waiting before next run...");
  delay(3000);
}
