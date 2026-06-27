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
// WALL PID PARAMETERS  — tune as needed
// ==========================================
float Kp_wall = 0.9;
float Ki_wall = 0.0;
float Kd_wall = 0.2;

int   targetLeftDistance    = 95;    // mm from left wall
int   obstacleThreshold     = 55;    // mm — front stop distance
int   leftWallLostThreshold = 250;   // mm — left wall reference lost (opening/end of wall)
int   wallLostTurnSpeed     = 150;   // outer-wheel PWM for the re-acquire arc turn

float previousWallError  = 0;
float wallIntegral       = 0;

// Sync-correction gain used while decelerating, to keep both wheels
// slowing down at the same actual rate (measured via encoders)
float Kp_sync = 2.5;

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
// Fills leftDist_mm and centerDist_mm by reference.
// Returns true  → clear to drive
// Returns false → front obstacle detected, caller should stop/turn
bool readSensors(int &leftDist_mm, int &centerDist_mm) {
  VL53L0X_RangingMeasurementData_t mLeft, mCenter;

  sensorLeft.rangingTest(&mLeft,   false);
  sensorCenter.rangingTest(&mCenter, false);

  // Cap out-of-range readings to safe defaults
  leftDist_mm   = (mLeft.RangeStatus   != 4) ? mLeft.RangeMilliMeter   : 200;
  centerDist_mm = (mCenter.RangeStatus != 4) ? mCenter.RangeMilliMeter : 999;

  return (centerDist_mm > obstacleThreshold);
}

// ==========================================
// RESET WALL PID STATE
// ==========================================
void resetWallPID() {
  wallIntegral      = 0.0;
  previousWallError = 0.0;
}

// ==========================================
// DECELERATE TO ZERO, THEN BRAKE  (encoder-synced)
// ==========================================
// Ramps both motors down from their current commanded speeds to 0,
// correcting for any L/R mismatch using encoder deltas between
// steps — keeps the robot straight while slowing instead of
// drifting. Ends with an active brake to hold position before the
// turn begins.
// ==========================================
void decelerateAndStop(int startLeftSpeed, int startRightSpeed) {
  const int decelSteps  = 20;   // number of ramp-down increments
  const int stepDelayMs = 15;   // ms between increments (~300ms total ramp)

  noInterrupts();
  long prevLeftTicks  = leftEncoderCount;
  long prevRightTicks = rightEncoderCount;
  interrupts();

  for (int i = decelSteps; i >= 0; i--) {
    int targetLeft  = (startLeftSpeed  * i) / decelSteps;
    int targetRight = (startRightSpeed * i) / decelSteps;

    noInterrupts();
    long currLeftTicks  = leftEncoderCount;
    long currRightTicks = rightEncoderCount;
    interrupts();

    long deltaLeft  = abs(currLeftTicks  - prevLeftTicks);
    long deltaRight = abs(currRightTicks - prevRightTicks);

    prevLeftTicks  = currLeftTicks;
    prevRightTicks = currRightTicks;

    int syncError      = (int)(deltaRight - deltaLeft);
    int syncCorrection = (int)(Kp_sync * syncError);

    int leftCmd  = constrain(targetLeft  + syncCorrection, 0, 255);
    int rightCmd = constrain(targetRight - syncCorrection, 0, 255);

    motorLeft.drive(leftCmd);
    motorRight.drive(rightCmd);

    delay(stepDelayMs);
  }

  motorLeft.brake();
  motorRight.brake();
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
// DRIVE DISTANCE  with wall-following PID
// ==========================================
// targetDistance_mm : how far to travel
// targetMaxSpeed    : peak PWM (0-255)
//
// The encoder profiler sets the *base* speed for each loop tick.
// The wall PID computes a *correction* applied symmetrically around
// that base speed, keeping the robot parallel to the left wall.
// A front obstacle instantly halts the run (returns early).
// A lost left-wall reference triggers an encoder-synced decel,
// then a 180° arc turn so the wall is re-acquired on the left —
// letting the robot continuously trace the wall boundary.
// ==========================================
void driveDistance(float targetDistance_mm, int targetMaxSpeed) {

  long targetTicks = (targetDistance_mm / WHEEL_CIRCUMFERENCE) * ENCODER_CPR;

  noInterrupts();
  leftEncoderCount  = 0;
  rightEncoderCount = 0;
  interrupts();

  resetWallPID();

  Serial.print("Target Ticks: "); Serial.println(targetTicks);

  // Speed-profiling parameters
  int   minSpeed   = 50;
  long  accelTicks = targetTicks * 0.10;   // first 10% → ramp up
  long  decelTicks = targetTicks * 0.15;   // last  15% → ramp down

  // Track last commanded speeds so decel knows what to ramp down from
  int lastLeftSpeed  = 0;
  int lastRightSpeed = 0;

  while (true) {

    // --- Read encoders safely ---
    noInterrupts();
    long currentLeft  = abs(leftEncoderCount);
    long currentRight = abs(rightEncoderCount);
    interrupts();

    long averageTicks = (currentLeft + currentRight) / 2;

    // --- Distance goal reached? ---
    if (averageTicks >= targetTicks) break;

    // --- Read sensors; abort if obstacle ahead ---
    int leftDist_mm, centerDist_mm;
    if (!readSensors(leftDist_mm, centerDist_mm)) {
      Serial.println("OBSTACLE AHEAD — stopping drive.");
      motorLeft.brake();
      motorRight.brake();
      resetWallPID();
      return;   // Exit early; caller decides what to do next
    }




    // // --- Left wall reference lost → decel, then 180° arc turn to re-acquire it ---
    // if (leftDist_mm >= leftWallLostThreshold) {
    //   Serial.println("LEFT WALL LOST — decelerating then arc-turning 180.");
    //   decelerateAndStop(lastLeftSpeed, lastRightSpeed);
    //   turnArc180Left(WHEELBASE_MM, wallLostTurnSpeed);  // re-acquire wall on the left
    //   resetWallPID();
    //   return;   // Exit; next driveDistance() call resumes wall-following
    // }

if (leftDist_mm >= leftWallLostThreshold) {
  Serial.println("LEFT TURN DETECTED — Evaluating speed for braking profile.");

  // 1. Snapshot the exact encoder count at the moment of detection
  noInterrupts();
  long startTicks = (abs(leftEncoderCount) + abs(rightEncoderCount)) / 2;
  interrupts();

  // =========================================================================
  // TUNING PARAMETERS FOR SPEED-BASED STOPPING
  // =========================================================================
  int speedThresholdPWM          = 100;   // Below this average PWM, the bot is considered "slow"
  float lowSpeedExtraDistance_mm = 50.0;  // Tunable: Min distance to stretch forward at low speed
  float highSpeedBraking_mm      = 5.0;  // Your normal snappy braking distance for high speeds
  // =========================================================================

  // Calculate the average speed right before detection
  int currentAverageSpeed = (abs(lastLeftSpeed) + abs(lastRightSpeed)) / 2;
  float brakingDistance_mm = highSpeedBraking_mm;

  // Split into two parts based on initial speed
  if (currentAverageSpeed < speedThresholdPWM) {
    Serial.print("Low speed detected ("); Serial.print(currentAverageSpeed);
    Serial.print("). Adding extra forward clearance: "); Serial.println(lowSpeedExtraDistance_mm);
    
    brakingDistance_mm = lowSpeedExtraDistance_mm;
  } else {
    Serial.print("High speed detected ("); Serial.print(currentAverageSpeed);
    Serial.println("). Using normal snappy stop.");
  }

  long brakingTicksRequired = (brakingDistance_mm / WHEEL_CIRCUMFERENCE) * ENCODER_CPR;
  long targetStopTicks = startTicks + brakingTicksRequired;

  int initialLeftSpeed  = lastLeftSpeed;
  int initialRightSpeed = lastRightSpeed;
  int minCrawlSpeed     = 40; 

  // 3. Distance-based deceleration loop
  while (true) {
    noInterrupts();
    long currentLeft  = abs(leftEncoderCount);
    long currentRight = abs(rightEncoderCount);
    interrupts();

    long currentAverage = (currentLeft + currentRight) / 2;

    // Stop condition: Target distance reached
    if (currentAverage >= targetStopTicks) break;

    // Calculate progress ratio through the braking zone (1.0 at start, 0.0 at end)
    long ticksRemaining = targetStopTicks - currentAverage;
    float progressRatio = (float)ticksRemaining / brakingTicksRequired;
    progressRatio = constrain(progressRatio, 0.0f, 1.0f);

    // 4. Compute scaled base speed based on distance remaining
    int baseLeft  = minCrawlSpeed + (int)((initialLeftSpeed - minCrawlSpeed) * progressRatio);
    int baseRight = minCrawlSpeed + (int)((initialRightSpeed - minCrawlSpeed) * progressRatio);

    // 5. Apply encoder synchronization to ensure it brakes in a perfectly straight line
    long deltaLeft  = currentLeft - startTicks;
    long deltaRight = currentRight - startTicks;
    int syncError   = (int)(abs(deltaRight) - abs(deltaLeft));
    int syncCorrection = (int)(Kp_sync * syncError);

    int finalLeftCmd  = constrain(baseLeft + syncCorrection, minCrawlSpeed, 255);
    int finalRightCmd = constrain(baseRight - syncCorrection, minCrawlSpeed, 255);

    motorLeft.drive(finalLeftCmd);
    motorRight.drive(finalRightCmd);

    delay(1); // Small loop pacing delay
  }

  // 6. Complete the stop
  motorLeft.brake();
  motorRight.brake();
  Serial.println("Position locked. Beginning turn execution.");

  // Execute your turn now that the loop has completely finished braking
  turnArc180Left(WHEELBASE_MM, wallLostTurnSpeed);
  resetWallPID();
  return;
}














    // ---- 1. Encoder-based speed profiling (base speed) ----
    int baseSpeed;
    if (averageTicks < accelTicks) {
      baseSpeed = map(averageTicks, 0, accelTicks, minSpeed, targetMaxSpeed);
    } else if (averageTicks > (targetTicks - decelTicks)) {
      long remaining = targetTicks - averageTicks;
      baseSpeed = map(remaining, decelTicks, 0, targetMaxSpeed, minSpeed);
    } else {
      baseSpeed = targetMaxSpeed;
    }

    // ---- 2. Wall-following PID correction ----
    float wallError    = (float)(leftDist_mm - targetLeftDistance);
    wallIntegral      += wallError;
    float wallDerivative = wallError - previousWallError;

    float wallCorrection = (Kp_wall * wallError)
                         + (Ki_wall * wallIntegral)
                         + (Kd_wall * wallDerivative);

    previousWallError = wallError;

    // Positive correction  → too far from left wall → curve left
    //   slow left wheel, speed up right wheel
    int leftSpeed  = baseSpeed - (int)wallCorrection;
    int rightSpeed = baseSpeed + (int)wallCorrection;

    leftSpeed  = constrain(leftSpeed,  0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    motorLeft.drive(leftSpeed);
    motorRight.drive(rightSpeed);

    lastLeftSpeed  = leftSpeed;
    lastRightSpeed = rightSpeed;

    // Debug output
    Serial.print("Ticks: ");   Serial.print(averageTicks);
    Serial.print(" | Base: "); Serial.print(baseSpeed);
    Serial.print(" | LWall: ");Serial.print(leftDist_mm);
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
void setup() {
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
  delay(5000);
}

// ==========================================
// LOOP
// ==========================================
void loop() {
  Serial.println("Starting run: 1260mm @ speed 250");
  driveDistance(1080.0, 250);

  Serial.println("Run complete. Waiting 5s...");
  delay(2000);
}