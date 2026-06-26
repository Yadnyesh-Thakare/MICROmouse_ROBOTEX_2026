/*
 * ================================================================
 * Micromouse — Right-Hand Rule Navigation
 * Encoder odometry + Gyro turns + Wall PID
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <SparkFun_TB6612.h>

// ================================================================
//  ENCODER PINS
// ================================================================
#define ENCL_A 19
#define ENCL_B 23
#define ENCR_A 2
#define ENCR_B 15

// ================================================================
//  MOTOR PINS
// ================================================================
#define PWMR  26
#define RIN1   4
#define RIN2  25
#define LIN1  17
#define LIN2   5
#define PWML  18
#define STBY  16

const int offsetL = -1;
const int offsetR =  1;

Motor motorLeft  = Motor(LIN1, LIN2, PWML, offsetL, STBY);
Motor motorRight = Motor(RIN1, RIN2, PWMR, offsetR, STBY);

// ================================================================
//  SENSOR PINS
// ================================================================
#define XSHUT_CENTER  27
#define XSHUT_LEFT    13
#define XSHUT_RIGHT   33

#define ADDRESS_CENTER  0x30
#define ADDRESS_LEFT    0x31
#define ADDRESS_RIGHT   0x32

Adafruit_VL53L0X sensorCenter = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLeft   = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight  = Adafruit_VL53L0X();

// ================================================================
//  MPU6050
// ================================================================
#define MPU_ADDR  0x68

float gyroBiasZ  = 0.0f;
float headingDeg = 0.0f;

// ================================================================
//  ODOMETRY
// ================================================================
const float ENCODER_CPR         = 2800.0f;
const float WHEEL_DIAMETER_MM   = 45.0f;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER_MM;  // ~141.37 mm
const float WHEELBASE_MM        = 100.0f;

volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;

// ================================================================
//  NAVIGATION TUNING — TUNE THESE
// ================================================================
const int   BASE_SPEED          = 200;    // normal drive speed
const int   FRONT_MAINTAIN_DIST = 50;     // mm — hold this gap from front wall
const int   WALL_LOST_THRESHOLD = 150;    // mm — beyond this = no wall
const int   TARGET_WALL_DIST    = 95;     // mm — ideal side wall distance

// Right wall absent: drive this far before turning right
// TUNE: one full cell = ~180mm, half cell = ~90mm
// Start at 160mm — increase if turn happens too early
const int   CELL_ADVANCE_MM     = 100;

// ================================================================
//  WALL PID — TUNE THESE
// ================================================================
float Kp_wall = 0.5f;
float Ki_wall = 0.0f;
float Kd_wall = 0.4f;

float prevWallError = 0.0f;
float wallIntegral  = 0.0f;

// ================================================================
//  TURN TUNING — TUNE THESE
// ================================================================
const int   TURN_SPEED_90       = 140;
const int   TURN_SPEED_180      = 130;
const int   TURN_TIMEOUT_MS     = 3000;

// TUNE: start at 15.0, adaptive will converge automatically
// Watch serial: "brakeLead" value should stabilize in 3-4 turns
float       g_brakeLeadAdaptive = 15.0f;

const float GYRO_STOP_THRESHOLD = 5.0f;   // dps — coast exit
const int   MAX_SETTLE_MS       = 200;    // ms  — coast safety cap
const float ADAPTIVE_RATE       = 0.4f;   // learning rate (0.2=slow, 0.5=fast)
const int   BIAS_WINDOW_MS      = 80;     // ms — local bias sample before turn
const float GYRO_THRESHOLD      = 0.4f;   // dps — noise floor deadzone

// ================================================================
//  ENCODER ISRs
// ================================================================
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

// ================================================================
//  FUNCTION PROTOTYPES
// ================================================================
void  wakeMPU();
void  calibrateGyro();
float readGyroRaw();
float readGyroZ();
void  initSensors();
void  readSensors(int &L, int &C, int &R);
void  stopMotors();
void  resetWallPID();
void  spotTurnGyro(float targetDeg, bool clockwise, int turnSpeed);
void  turnRight();
void  uTurn();
void  driveUntilFrontClose();
void  driveOneCell();
void  navigate();

// ================================================================
//  MPU6050
// ================================================================
void wakeMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x08);   // ±500 dps
  Wire.endTransmission(true);
}

void calibrateGyro() {
  Serial.println("Calibrating gyro — keep still...");
  delay(1000);
  float sum = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, true);
    int16_t raw = (Wire.read() << 8) | Wire.read();
    sum += raw / 65.5f;
    delay(10);
  }
  gyroBiasZ = sum / 200.0f;
  Serial.print("Gyro bias Z = "); Serial.println(gyroBiasZ, 4);
}

// Raw dps — no bias removed (turn function handles its own bias)
float readGyroRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, true);
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return raw / 65.5f;
}

// Bias-corrected — used during straight driving
float readGyroZ() {
  float gz = readGyroRaw() - gyroBiasZ;
  if (fabs(gz) < GYRO_THRESHOLD) gz = 0.0f;
  return gz;
}

// ================================================================
//  SENSOR INIT
// ================================================================
void initSensors() {
  pinMode(XSHUT_CENTER, OUTPUT);
  pinMode(XSHUT_LEFT,   OUTPUT);
  pinMode(XSHUT_RIGHT,  OUTPUT);

  digitalWrite(XSHUT_CENTER, LOW);
  digitalWrite(XSHUT_LEFT,   LOW);
  digitalWrite(XSHUT_RIGHT,  LOW);
  delay(10);

  digitalWrite(XSHUT_CENTER, HIGH); delay(10);
  if (!sensorCenter.begin(ADDRESS_CENTER)) {
    Serial.println("FATAL: Center sensor!"); while (1);
  }
  digitalWrite(XSHUT_LEFT, HIGH); delay(10);
  if (!sensorLeft.begin(ADDRESS_LEFT)) {
    Serial.println("FATAL: Left sensor!"); while (1);
  }
  digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
  if (!sensorRight.begin(ADDRESS_RIGHT)) {
    Serial.println("FATAL: Right sensor!"); while (1);
  }
  Serial.println("All sensors ready.");
}

// ================================================================
//  READ ALL 3 SENSORS
// ================================================================
void readSensors(int &L, int &C, int &R) {
  VL53L0X_RangingMeasurementData_t mL, mC, mR;
  sensorLeft.rangingTest(&mL,   false);
  sensorCenter.rangingTest(&mC, false);
  sensorRight.rangingTest(&mR,  false);

  L = (mL.RangeStatus != 4) ? (int)mL.RangeMilliMeter : 999;
  C = (mC.RangeStatus != 4) ? (int)mC.RangeMilliMeter : 999;
  R = (mR.RangeStatus != 4) ? (int)mR.RangeMilliMeter : 999;
}

// ================================================================
//  HELPERS
// ================================================================
void stopMotors() {
  motorLeft.drive(0);
  motorRight.drive(0);
}

void resetWallPID() {
  wallIntegral  = 0.0f;
  prevWallError = 0.0f;
}

// ================================================================
//  SPOT TURN — GYRO WITH ADAPTIVE BRAKE LEAD
//  Uses readGyroRaw() + local bias to avoid double-subtract bug
// ================================================================
void spotTurnGyro(float targetDeg, bool clockwise, int turnSpeed) {

  float deltaYaw = 0.0f;
  unsigned long tUs = micros();

  // Fresh local bias sample — compensates thermal drift between turns
  float localBias = 0.0f;
  {
    unsigned long biasStart = millis();
    int n = 0;
    while (millis() - biasStart < (unsigned long)BIAS_WINDOW_MS) {
      localBias += readGyroRaw();
      n++;
      delayMicroseconds(500);
    }
    if (n > 0) localBias /= n;
  }

  float brakeAt = targetDeg - g_brakeLeadAdaptive;
  if (brakeAt < 2.0f) brakeAt = 2.0f;  // hard minimum

  unsigned long startMs = millis();

  // Phase 1 — full speed spin
  if (clockwise) {
    motorLeft.drive( turnSpeed);
    motorRight.drive(-turnSpeed);
  } else {
    motorLeft.drive(-turnSpeed);
    motorRight.drive( turnSpeed);
  }

  while (true) {
    unsigned long now = micros();
    float dt = (now - tUs) / 1000000.0f;
    tUs = now;

    float gz = readGyroRaw() - localBias;
    if (fabs(gz) < GYRO_THRESHOLD) gz = 0.0f;
    deltaYaw += gz * dt;

    if (fabs(deltaYaw) >= brakeAt) break;

    if (millis() - startMs > (unsigned long)TURN_TIMEOUT_MS) {
      Serial.println("Turn | TIMEOUT");
      break;
    }
    delayMicroseconds(200);
  }

  stopMotors();

  // Phase 2 — coast: keep integrating until gyro settles
  unsigned long settleStart = millis();
  while (millis() - settleStart < (unsigned long)MAX_SETTLE_MS) {
    unsigned long now = micros();
    float dt = (now - tUs) / 1000000.0f;
    tUs = now;

    float gz = readGyroRaw() - localBias;
    if (fabs(gz) < GYRO_THRESHOLD) gz = 0.0f;
    deltaYaw += gz * dt;

    if (fabs(gz) < GYRO_STOP_THRESHOLD) break;
    delayMicroseconds(200);
  }

  // Adaptive brake lead — self-corrects each turn
  float error = fabs(deltaYaw) - targetDeg;  // +ve=overshot, -ve=undershot
  g_brakeLeadAdaptive += error * ADAPTIVE_RATE;
  g_brakeLeadAdaptive  = constrain(g_brakeLeadAdaptive, 1.0f, 45.0f);

  // Update absolute heading
  if (clockwise) headingDeg += fabs(deltaYaw);
  else           headingDeg -= fabs(deltaYaw);
  headingDeg = fmod(headingDeg + 360.0f, 360.0f);

  Serial.print("Turn | target:"); Serial.print(targetDeg, 1);
  Serial.print(" actual:");       Serial.print(fabs(deltaYaw), 2);
  Serial.print(" error:");        Serial.print(error, 2);
  Serial.print(" brakeLead:");    Serial.println(g_brakeLeadAdaptive, 2);
}

// ================================================================
//  TURN WRAPPERS
// ================================================================
void turnRight() {
  Serial.println("--- TURN RIGHT ---");
  stopMotors(); delay(100);
  spotTurnGyro(90.0f, true, TURN_SPEED_90);
  resetWallPID();
}

void uTurn() {
  Serial.println("--- U-TURN ---");
  stopMotors(); delay(100);
  spotTurnGyro(180.0f, true, TURN_SPEED_180);
  resetWallPID();
}

// ================================================================
//  WALL PID CORRECTION — Right-Hand Rule
//
//  Priority:
//    Both walls  → center between them (left - right = 0)
//    Right only  → maintain TARGET_WALL_DIST from right
//    Left only   → maintain TARGET_WALL_DIST from left
//    No wall     → drive straight (correction = 0)
//
//  Positive correction → steer left  (right wall too close)
//  Negative correction → steer right (left wall too close)
// ================================================================
float computeWallPID(int L, int R) {
  bool leftWall  = (L < WALL_LOST_THRESHOLD);
  bool rightWall = (R < WALL_LOST_THRESHOLD);

  float wallError = 0.0f;

  if (leftWall && rightWall) {
    // Both visible — center between them
    wallError = (float)(L - R);
  } else if (rightWall && !leftWall) {
    // Right wall only — hold standoff from right
    // Positive error = too close to right → steer left
    wallError = (float)(TARGET_WALL_DIST - R);
    wallError = -wallError;   // flip: closer to right = positive correction = steer left
  } else if (leftWall && !rightWall) {
    // Left wall only — hold standoff from left
    wallError = (float)(L - TARGET_WALL_DIST);
  } else {
    // No wall — go straight
    wallError = 0.0f;
  }

  wallIntegral += wallError;
  wallIntegral  = constrain(wallIntegral, -2000.0f, 2000.0f);
  float deriv   = wallError - prevWallError;
  prevWallError = wallError;

  return (Kp_wall * wallError) + (Ki_wall * wallIntegral) + (Kd_wall * deriv);
}

// ================================================================
//  DRIVE ONE CELL — encoder distance with wall PID
//  Used for normal forward movement and post-right-wall-loss advance
// ================================================================
void driveDistance(int targetMM, int maxSpeed) {
  long targetTicks = (long)((targetMM / WHEEL_CIRCUMFERENCE) * ENCODER_CPR);

  noInterrupts();
  leftEncoderCount  = 0;
  rightEncoderCount = 0;
  interrupts();

  resetWallPID();

  const int minSpeed   = 100;
  long accelTicks = targetTicks * 0.20f;
  long decelTicks = targetTicks * 0.20f;

  while (true) {
    noInterrupts();
    long curL = abs(leftEncoderCount);
    long curR = abs(rightEncoderCount);
    interrupts();

    long avgTicks = (curL + curR) / 2;
    if (avgTicks >= targetTicks) break;

    int L, C, R;
    readSensors(L, C, R);

    // Front wall check — stop early if hit during cell advance
    if (C <= FRONT_MAINTAIN_DIST) {
      Serial.println("DriveDistance | front wall hit early");
      stopMotors();
      return;
    }

    // Speed profile
    int baseSpeed;
    if (avgTicks < accelTicks) {
      baseSpeed = map(avgTicks, 0, accelTicks, minSpeed, maxSpeed);
    } else if (avgTicks > (targetTicks - decelTicks)) {
      long remaining = targetTicks - avgTicks;
      baseSpeed = map(remaining, decelTicks, 0, maxSpeed, minSpeed);
    } else {
      baseSpeed = maxSpeed;
    }

    float correction = computeWallPID(L, R);

    int lSpd = constrain(baseSpeed - (int)correction, 0, 255);
    int rSpd = constrain(baseSpeed + (int)correction, 0, 255);

    motorLeft.drive(lSpd);
    motorRight.drive(rSpd);

    Serial.print("Ticks:"); Serial.print(avgTicks);
    Serial.print(" C:");    Serial.print(C);
    Serial.print(" L:");    Serial.print(L);
    Serial.print(" R:");    Serial.print(R);
    Serial.print(" Corr:"); Serial.println((int)correction);

    delay(1);
  }

  stopMotors();
  Serial.println("DriveDistance done.");
}

// ================================================================
//  DRIVE UNTIL FRONT WALL — stop at FRONT_MAINTAIN_DIST
//  Wall PID active throughout; used when approaching dead end
// ================================================================
void driveUntilFrontClose() {
  resetWallPID();
  Serial.println("DriveUntilFront start");

  while (true) {
    int L, C, R;
    readSensors(L, C, R);

    Serial.print("FWD | C:"); Serial.print(C);
    Serial.print(" L:");      Serial.print(L);
    Serial.print(" R:");      Serial.println(R);

    // Stop and maintain 75mm from front wall
    if (C <= FRONT_MAINTAIN_DIST) {
      stopMotors();
      Serial.println("Front wall — maintained 75mm gap");
      return;
    }

    float correction = computeWallPID(L, R);

    int lSpd = constrain(BASE_SPEED - (int)correction, 0, 255);
    int rSpd = constrain(BASE_SPEED + (int)correction, 0, 255);

    motorLeft.drive(lSpd);
    motorRight.drive(rSpd);

    delay(1);
  }
}

// ================================================================
//  NAVIGATE — Right-Hand Rule decision loop
//
//  Logic:
//    1. Right wall present  → drive forward (wall PID keeps us parallel)
//    2. Right wall absent   → advance CELL_ADVANCE_MM then turn right
//    3. Front wall at 75mm  → U-turn
// ================================================================
void navigate() {
  int L, C, R;
  readSensors(L, C, R);

  bool rightWall = (R < WALL_LOST_THRESHOLD);
  bool frontWall = (C <= FRONT_MAINTAIN_DIST);

  Serial.print("NAV | C:"); Serial.print(C);
  Serial.print(" L:");      Serial.print(L);
  Serial.print(" R:");      Serial.print(R);
  Serial.print(" | rWall:"); Serial.print(rightWall);
  Serial.print(" front:");   Serial.println(frontWall);

  // ── Priority 1: Front wall → U-turn ──────────────────────────
  if (frontWall) {
    Serial.println("NAV → U-TURN (front wall)");
    uTurn();
    return;
  }

  // ── Priority 2: Right wall gone → advance then turn right ────
  if (!rightWall) {
    Serial.println("NAV → right wall lost — advancing then turning right");
    // Drive forward to clear the corner before turning
    // TUNE: CELL_ADVANCE_MM — how far to go after wall disappears
    driveDistance(CELL_ADVANCE_MM, BASE_SPEED);
    delay(100);
    turnRight();
    return;
  }

  // ── Priority 3: Right wall present → keep going ──────────────
  // Normal forward pass — caller (loop) handles wall PID driving
  // Nothing to do here; loop() will call driveUntilFrontClose
  // or keep driving. This branch shouldn't normally be hit if
  // loop() checks front first.
  Serial.println("NAV → right wall present, continue");
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== MICROMOUSE RIGHT-HAND RULE ===");

  // Motor standby
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Encoders
  pinMode(ENCL_A, INPUT_PULLUP);
  pinMode(ENCL_B, INPUT_PULLUP);
  pinMode(ENCR_A, INPUT_PULLUP);
  pinMode(ENCR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCL_A), leftEncoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCL_B), leftEncoderBISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCR_A), rightEncoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCR_B), rightEncoderBISR, CHANGE);

  // I2C + sensors
  Wire.begin();
  Wire.setClock(400000);
  wakeMPU();
  calibrateGyro();

  headingDeg = 0.0f;
  initSensors();

  Serial.println("Starting in 2s...");
  delay(1000);
  resetWallPID();
}

// ================================================================
//  MAIN LOOP — Right-Hand Rule
//
//  Each iteration:
//    1. Read sensors
//    2. Front wall at 75mm → U-turn
//    3. Right wall gone    → advance CELL_ADVANCE_MM + right turn
//    4. Right wall present → drive forward with wall PID
// ================================================================
void loop() {
  int L, C, R;
  readSensors(L, C, R);

  bool rightWall = (R < WALL_LOST_THRESHOLD);
  bool frontWall = (C <= FRONT_MAINTAIN_DIST);
  bool leftWall = (L < WALL_LOST_THRESHOLD);
  // ── Front wall: stop at 75mm, then U-turn ───────────────────
  /*if (frontWall) {
    stopMotors();
    delay(1000);
    Serial.println("LOOP → front wall → U-turn");
    //uTurn();
    //delay(200);
    return;
  }*/

  // ── Right wall absent: advance one cell then turn right ──────
  if (!rightWall ) {
    Serial.println("LOOP → right wall lost → advance + right turn");
    driveDistance(CELL_ADVANCE_MM, BASE_SPEED);
    delay(100);
    turnRight();
    /*// Re-check front after advance
    readSensors(L, C, R);
    if (C <= FRONT_MAINTAIN_DIST) {
      Serial.println("LOOP → front hit after advance → U-turn");
      uTurn();
    } else {
      turnRight();
    }*/
    delay(200);
    return;
    
  }
  if(rightWall && frontWall && leftWall){
      stopMotors();
      delay(3000);
  }
  // ── Right wall present: drive forward with wall PID ──────────
  float correction = computeWallPID(L, R);

  int lSpd = constrain(BASE_SPEED - (int)correction, 0, 255);
  int rSpd = constrain(BASE_SPEED + (int)correction, 0, 255);

  motorLeft.drive(lSpd);
  motorRight.drive(rSpd);

  Serial.print("FWD | C:"); Serial.print(C);
  Serial.print(" L:");      Serial.print(L);
  Serial.print(" R:");      Serial.print(R);
  Serial.print(" Corr:");   Serial.println((int)correction);

  delay(1);
}