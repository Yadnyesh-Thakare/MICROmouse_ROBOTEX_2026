#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>      // Re-optimized to Pololu High-Speed Library
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

// ==========================================
// ODOMETRY CONSTANTS
// ==========================================
const float ENCODER_CPR         = 2800.0;
const float WHEEL_DIAMETER_MM   = 45.0;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER_MM;  // ~141.37 mm
const float WHEELBASE_MM       = 100.0;   

volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;

// ==========================================
// WALL-CENTERING PID PARAMETERS
// ==========================================
float Kp_wall = 0.5; 
float Ki_wall = 0.0; 
float Kd_wall = 0.4; 

int   targetWallDistance = 95;    // mm 
int   obstacleThreshold  = 80;    // mm — front stop distance
int   WallLostThreshold  = 145;   // mm 

int leftDist_mm = 1000, centerDist_mm = 1000, rightDist_mm = 1000;
float previousWallError = 0;
float wallIntegral      = 0;

VL53L0X sensorLeft, sensorCenter, sensorRight;

// ==========================================
// FAST CRASH-SAFE ENCODER ISRs
// ==========================================
void IRAM_ATTR leftEncoderAISR()  { leftEncoderCount  += (GPIO.in >> ENCL_A & 1) == (GPIO.in >> ENCL_B & 1) ? -1 : 1; }
void IRAM_ATTR leftEncoderBISR()  { leftEncoderCount  += (GPIO.in >> ENCL_A & 1) != (GPIO.in >> ENCL_B & 1) ? -1 : 1; }
void IRAM_ATTR rightEncoderAISR() { rightEncoderCount += (GPIO.in >> ENCR_A & 1) == (GPIO.in >> ENCR_B & 1) ? -1 : 1; }
void IRAM_ATTR rightEncoderBISR() { rightEncoderCount += (GPIO.in >> ENCR_A & 1) != (GPIO.in >> ENCR_B & 1) ? -1 : 1; }

// ==========================================
// SENSOR INITIALISATION (Fixed Reset Bug)
// ==========================================
void initSensors() 
{
  pinMode(XSHUT_CENTER, OUTPUT); pinMode(XSHUT_LEFT, OUTPUT); pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_CENTER, LOW); digitalWrite(XSHUT_LEFT, LOW); digitalWrite(XSHUT_RIGHT, LOW);
  delay(50); // Hard reset physical discharge

  auto initIndiv = [](VL53L0X& s, int pin, uint8_t addr) {
    digitalWrite(pin, HIGH); delay(50);
    s.setAddress(addr);
    if (!s.init()) { Serial.printf("FATAL: Sensor %X failed!\n", addr); while (1); }
    s.setTimeout(200);
    s.startContinuous();
  };

  initIndiv(sensorCenter, XSHUT_CENTER, ADDRESS_CENTER);
  initIndiv(sensorLeft,   XSHUT_LEFT,   ADDRESS_LEFT);
  initIndiv(sensorRight,  XSHUT_RIGHT,  ADDRESS_RIGHT);
  Serial.println("All sensors initialized in non-blocking continuous mode.");
}

// ==========================================
// READ SENSORS (Instantaneous Cache Grab)
// ==========================================
int readSensors()
{
  uint16_t l = sensorLeft.readRangeContinuousMillimeters();
  uint16_t c = sensorCenter.readRangeContinuousMillimeters();
  uint16_t r = sensorRight.readRangeContinuousMillimeters();

  leftDist_mm   = (l > 2000 || sensorLeft.timeoutOccurred())   ? 1000 : l;
  centerDist_mm = (c > 2000 || sensorCenter.timeoutOccurred()) ? 1000 : c;
  rightDist_mm  = (r > 2000 || sensorRight.timeoutOccurred())  ? 1000 : r;

  int mask = ((leftDist_mm   > WallLostThreshold)  << 2) |
             ((centerDist_mm > obstacleThreshold)  << 1) | 
             ((rightDist_mm  > WallLostThreshold)  << 0);
  
  const int caseMap[8] = { 7, 3, 2, 5, 1, 6, 4, 0 };
  return caseMap[mask];
}

void resetWallPID() {
  wallIntegral      = 0.0;
  previousWallError = 0.0;
}

// ==========================================
// CONTROLLED ARC TURNS
// ==========================================
void turnArc(float angleDeg, float turnRadius_mm, int maxOuterSpeed, bool turnLeft) {
  float angleRad    = angleDeg * (PI / 180.0f);
  float innerRadius = turnRadius_mm - WHEELBASE_MM / 2.0f;
  float outerRadius = turnRadius_mm + WHEELBASE_MM / 2.0f;

  float innerArc_mm = innerRadius * angleRad;
  float outerArc_mm = outerRadius * angleRad;

  long innerTargetTicks = (innerArc_mm / WHEEL_CIRCUMFERENCE) * ENCODER_CPR;
  long outerTargetTicks = (outerArc_mm / WHEEL_CIRCUMFERENCE) * ENCODER_CPR;

  float speedRatio = innerArc_mm / outerArc_mm;  

  noInterrupts();
  leftEncoderCount  = 0;
  rightEncoderCount = 0;
  interrupts();

  const int minOuterSpeed = 60;
  int       minInnerSpeed = max(30, (int)(minOuterSpeed * speedRatio));

  long accelTicks = outerTargetTicks * 0.25; // Snappier curves
  long decelTicks = outerTargetTicks * 0.25;

  while (true) {
    noInterrupts();
    long currentLeft  = abs(leftEncoderCount);
    long currentRight = abs(rightEncoderCount);
    interrupts();

    long progressTicks = turnLeft ? currentRight : currentLeft;
    if (progressTicks >= outerTargetTicks) break;

    int outerSpeed;
    if (progressTicks < accelTicks) {
      outerSpeed = map(progressTicks, 0, accelTicks, minOuterSpeed, maxOuterSpeed);
    } else if (progressTicks > (outerTargetTicks - decelTicks)) {
      long remaining = outerTargetTicks - progressTicks;
      outerSpeed = map(remaining, decelTicks, 0, maxOuterSpeed, minOuterSpeed);
    } else {
      outerSpeed = maxOuterSpeed;
    }

    int innerSpeed = (int)(outerSpeed * speedRatio);
    innerSpeed = constrain(innerSpeed, minInnerSpeed, 255);

    long outerProgress = turnLeft ? currentRight : currentLeft;
    long innerCurrent  = turnLeft ? currentLeft  : currentRight;
    long expectedInner = (long)(outerProgress * speedRatio);
    long innerError    = expectedInner - innerCurrent;  
    int   correction    = (int)(innerError * 0.4f);
    innerSpeed = constrain(innerSpeed + correction, 0, 255);

    if (turnLeft) {
      motorLeft.drive(innerSpeed);   
      motorRight.drive(outerSpeed);  
    } else {
      motorLeft.drive(outerSpeed);   
      motorRight.drive(innerSpeed);  
    }
    delay(1);
  }

  // Actively stop turning momentum
  motorLeft.drive(turnLeft ? 150 : -150);
  motorRight.drive(turnLeft ? -150 : 150);
  delay(20);

  motorLeft.brake();
  motorRight.brake();
  delay(150); // Let chassis stabilize completely before reading sensors next
}

void turnArc90Left (float turnRadius_mm, int maxOuterSpeed) { turnArc(90.0f,  turnRadius_mm, maxOuterSpeed, true); }
void turnArc90Right(float turnRadius_mm, int maxOuterSpeed) { turnArc(90.0f,  turnRadius_mm, maxOuterSpeed, false); }

// ==========================================
// DRIVE UNTIL WALL DETECTED WITH ACTIVE MOMENTUM BRAKE
// ==========================================
void driveUntilWall(int baseSpeed)
{
  resetWallPID();
  Serial.println("Driving forward until front wall is detected...");

  while (true)
  {
    readSensors();

    // 1. Front Obstacle Check
    if (centerDist_mm <= obstacleThreshold) {
        Serial.printf("FRONT WALL DETECTED (%d mm) — Active Brake Triggered.\n", centerDist_mm);
        break;
    }

    // 2. Approach Deceleration Buffer Zone
    int adjustedBase = baseSpeed;
    if (centerDist_mm < 200) {
      adjustedBase = map(centerDist_mm, obstacleThreshold, 200, 70, baseSpeed);
    }

    // 3. Wall Alignment PID Calculation
    bool leftWallLost  = (leftDist_mm  >= WallLostThreshold);
    bool rightWallLost = (rightDist_mm >= WallLostThreshold);
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

    int leftSpeed  = adjustedBase - (int)wallCorrection;
    int rightSpeed = adjustedBase + (int)wallCorrection;

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

// ==========================================
// SETUP
// ==========================================
void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  pinMode(ENCL_A, INPUT_PULLUP); pinMode(ENCL_B, INPUT_PULLUP);
  pinMode(ENCR_A, INPUT_PULLUP); pinMode(ENCR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCL_A), leftEncoderAISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCL_B), leftEncoderBISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCR_A), rightEncoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCR_B), rightEncoderBISR, CHANGE);

  Wire.begin();
  Wire.setClock(400000); // Elevate standard I2C throughput 
  initSensors();

  Serial.println("Robot Ready. Place inside maze now...");
  delay(3000);
}


// ==========================================
// LOOP (Fixed Wallcase Navigation Pipeline)
// ==========================================
void loop()
{
  // 1. Evaluate the environment and print for debugging
  int wallcase = readSensors();
  Serial.printf("Wall Case Evaluated: %d | L:%d C:%d R:%d\n", 
                wallcase, leftDist_mm, centerDist_mm, rightDist_mm);

  // 2. CHOOSE ACTION BASED ON THE CASE NUMBER
  if (wallcase == 2 || wallcase == 4 || wallcase == 5 || wallcase == 7) 
  {
    // Any case where the Front is Open -> Keep driving forward
    Serial.println("-> Path ahead is open. Driving...");
    driveUntilWall(180);
  }
  else if (wallcase == 1) 
  {
    // Front is blocked, but LEFT side is wide open
    Serial.println("-> Front blocked. Executing 90 Left Turn...");
    turnArc90Left(WHEELBASE_MM, 160);
    delay(50); // Small pause to let sensors stabilize after turning
  }
  else if (wallcase == 3) 
  {
    // Front is blocked, but RIGHT side is wide open
    Serial.println("-> Front blocked. Executing 90 Right Turn...");
    turnArc90Right(WHEELBASE_MM, 160);
    delay(50); 
  }
  else if (wallcase == 6)
  {
    // Front is blocked, but BOTH Left and Right are open. (Defaulting to Left)
    Serial.println("-> Intersection split! Turning Left by default...");
    turnArc90Left(WHEELBASE_MM, 160);
    delay(50);
  }
  else // Case 0 (Dead End - All 3 walls blocked)
  {
    Serial.println("-> DEAD END! Firing 180 turnaround escape...");
    turnArc(180.0f, WHEELBASE_MM, 160, true); // Rotate 180 degrees
    delay(200); 
  }
}