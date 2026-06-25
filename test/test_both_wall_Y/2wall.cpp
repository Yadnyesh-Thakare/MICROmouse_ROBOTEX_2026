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
bool readSensors(int &leftDist_mm, int &centerDist_mm, int &rightDist_mm)
{
  VL53L0X_RangingMeasurementData_t mLeft, mCenter, mRight;

  sensorLeft.rangingTest(&mLeft,     false);
  sensorCenter.rangingTest(&mCenter, false);
  sensorRight.rangingTest(&mRight,   false);

  // Cap out-of-range readings to safe defaults
  leftDist_mm   = (mLeft.RangeStatus   != 4) ? mLeft.RangeMilliMeter   : 200;
  centerDist_mm = (mCenter.RangeStatus != 4) ? mCenter.RangeMilliMeter : 800;
  rightDist_mm  = (mRight.RangeStatus  != 4) ? mRight.RangeMilliMeter  : 200;

  return (centerDist_mm > obstacleThreshold);
}

// ==========================================
// RESET WALL PID STATE
// ==========================================
void resetWallPID()
{
  wallIntegral      = 0.0;
  previousWallError = 0.0;
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


    if (!readSensors(leftDist_mm, centerDist_mm, rightDist_mm))
    {
        Serial.println("OBSTACLE AHEAD — stopping drive.");
        motorLeft.brake();
        motorRight.brake();
        resetWallPID();
        return;   // Exit early; caller decides what to do next
    }

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

    // Debug output
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
