/*
 * ================================================================
 * Micromouse — Straight + U-Turn Loop Test
 * Drives straight with wall PID, stops at 50mm front wall,
 * performs gyro U-turn, then repeats indefinitely.
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <SparkFun_TB6612.h>

#define MPU_ADDR  0x68

float gyroBiasZ    = 0.0f;
float yaw          = 0.0f;
unsigned long lastGyroUpdate = 0;

#define XSHUT_CENTER  27
#define XSHUT_LEFT    13
#define XSHUT_RIGHT   33

#define ADDRESS_CENTER  0x30
#define ADDRESS_LEFT    0x31
#define ADDRESS_RIGHT   0x32

#define RIN1   4
#define RIN2  25
#define PWMR  26
#define LIN1  17
#define LIN2   5
#define PWML  18
#define STBY  16

#define LED_LEFT   12
#define LED_RIGHT  15

const int OFFSET_LEFT  = -1;
const int OFFSET_RIGHT =  1;

Motor motorLeft  = Motor(LIN1, LIN2, PWML, OFFSET_LEFT,  STBY);
Motor motorRight = Motor(RIN1, RIN2, PWMR, OFFSET_RIGHT, STBY);

Adafruit_VL53L0X sensorCenter = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLeft   = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight  = Adafruit_VL53L0X();

int distLeft   = 999;
int distRight  = 999;
int distCenter = 999;

// ================================================================
//  TUNE THESE
// ================================================================
const int   BASE_SPEED       = 200;
const int   NUDGE_SPEED      = 80;

const int   FRONT_STOP_DIST  = 75;    // mm — stop and U-turn trigger
const int   SIDE_WALL_DETECT = 150;   // mm — wall present threshold
const int   IDEAL_SIDE_DIST  = 95;    // mm — target side distance
const int   NUDGE_TOLERANCE  = 8;     // mm — wall correction deadband

const int   TURN_SPEED_180   = 130;   // PWM during U-turn
const float BRAKE_LEAD_180   = 15.0f; // deg early stop for coast ← tune
const float GYRO_THRESHOLD   = 0.4f;  // dps deadband
const int   TURN_SETTLE_MS   = 80;
const int   TURN_TIMEOUT_MS  = 3000;

// ================================================================
//  WALL PID
// ================================================================
float Kp = 0.75f;
float Ki = 0.00001f;
float Kd = 0.2f;
const int MAX_INTEGRAL = 2000;

float prevError   = 0.0f;
float integral    = 0.0f;
unsigned long lastPIDTime = 0;

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
  Serial.println("Calibrating gyro — keep robot still...");
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
  Serial.print("Gyro bias Z = ");
  Serial.println(gyroBiasZ, 4);
}

float readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, true);
  int16_t raw = (Wire.read() << 8) | Wire.read();
  float gz = (raw / 65.5f) - gyroBiasZ;
  if (fabs(gz) < GYRO_THRESHOLD) gz = 0.0f;
  return gz;
}

// ================================================================
//  HELPERS
// ================================================================
int readSensor(Adafruit_VL53L0X &sensor, int fallback) {
  VL53L0X_RangingMeasurementData_t m;
  sensor.rangingTest(&m, false);
  return (m.RangeStatus != 4) ? (int)m.RangeMilliMeter : fallback;
}

void stopMotors() {
  motorLeft.drive(0);
  motorRight.drive(0);
}

void resetPID() {
  integral    = 0.0f;
  prevError   = 0.0f;
  lastPIDTime = millis();
}

void blinkLED(int pin, int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH); delay(200);
    digitalWrite(pin, LOW);  delay(200);
  }
}

// ================================================================
//  SENSORS INIT
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
    Serial.println("FATAL: Center VL53L0X!"); while (1);
  }
  digitalWrite(XSHUT_LEFT, HIGH); delay(10);
  if (!sensorLeft.begin(ADDRESS_LEFT)) {
    Serial.println("FATAL: Left VL53L0X!"); while (1);
  }
  digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
  if (!sensorRight.begin(ADDRESS_RIGHT)) {
    Serial.println("FATAL: Right VL53L0X!"); while (1);
  }
  Serial.println("All sensors ready.");
}

// ================================================================
//  GYRO U-TURN
// ================================================================
void uTurn() {
  stopMotors();
  delay(100);

  // Reset yaw
  yaw = 0.0f;
  lastGyroUpdate = micros();

  float stopAt = 180.0f - BRAKE_LEAD_180;
  if (stopAt < 1.0f) stopAt = 1.0f;

  unsigned long startMs = millis();

  // CW spin
  motorLeft.drive( TURN_SPEED_180);
  motorRight.drive(-TURN_SPEED_180);

  // Phase 1 — spin to brake point
  while (true) {
    unsigned long now = micros();
    float dt = (now - lastGyroUpdate) / 1000000.0f;
    lastGyroUpdate = now;
    yaw += readGyroZ() * dt;

    if (fabs(yaw) >= stopAt) {
      Serial.print("U-Turn | brake point | yaw: ");
      Serial.println(yaw, 2);
      break;
    }
    if (millis() - startMs > TURN_TIMEOUT_MS) {
      Serial.println("U-Turn | TIMEOUT");
      break;
    }
    delayMicroseconds(200);
  }

  stopMotors();

  // Phase 2 — keep integrating during coast
  unsigned long settleStart = millis();
  while (millis() - settleStart < (unsigned long)TURN_SETTLE_MS) {
    unsigned long now = micros();
    float dt = (now - lastGyroUpdate) / 1000000.0f;
    lastGyroUpdate = now;
    yaw += readGyroZ() * dt;
    delayMicroseconds(200);
  }

  Serial.print("U-Turn | final yaw: ");
  Serial.print(yaw, 2);
  Serial.println(" deg (target 180.0)");

  blinkLED(LED_LEFT,  2);
  blinkLED(LED_RIGHT, 2);

  // Clean PID state before driving again
  resetPID();
}

// ================================================================
//  WALL CORRECTION  (after U-turn)
// ================================================================
void correctWallPosition() {
  for (int attempt = 0; attempt < 10; attempt++) {
    distLeft  = readSensor(sensorLeft,  999);
    distRight = readSensor(sensorRight, 999);

    int refDist = 999;
    int sign    = 1;

    // Use whichever wall is present
    if      (distLeft  < SIDE_WALL_DETECT) { refDist = distLeft;  sign =  1; }
    else if (distRight < SIDE_WALL_DETECT) { refDist = distRight; sign = -1; }

    if (refDist == 999) {
      Serial.println("WallCorrect: no wall, skipping");
      break;
    }

    int error = refDist - IDEAL_SIDE_DIST;
    Serial.print("WallCorrect | ref:"); Serial.print(refDist);
    Serial.print(" err:");              Serial.println(error);

    if (abs(error) <= NUDGE_TOLERANCE) break;

    if ((sign * error) > 0) {
      motorLeft.drive(-NUDGE_SPEED);
      motorRight.drive( NUDGE_SPEED);
    } else {
      motorLeft.drive( NUDGE_SPEED);
      motorRight.drive(-NUDGE_SPEED);
    }
    delay(30);
    stopMotors();
    delay(50);
  }

  resetPID();
}

// ================================================================
//  PID FUNCTION (Extracted from Loop)
// ================================================================
void followWallPID() {
  unsigned long now = millis();
  float dt = (float)(now - lastPIDTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  lastPIDTime = now;

  bool hasLeft  = (distLeft  < SIDE_WALL_DETECT);
  bool hasRight = (distRight < SIDE_WALL_DETECT);

  digitalWrite(LED_LEFT,  hasLeft  ? HIGH : LOW);
  digitalWrite(LED_RIGHT, hasRight ? HIGH : LOW);

  float error    = 0.0f;
  bool  wallFound = false;

  // Prefer left wall, fall back to right
  if (hasLeft) {
    error     =  (float)(distLeft  - IDEAL_SIDE_DIST);
    wallFound = true;
  } else if (hasRight) {
    error     = -(float)(distRight - IDEAL_SIDE_DIST);
    wallFound = true;
  }

  float correction = 0.0f;
  if (wallFound) {
    integral   += error * dt;
    integral    = constrain(integral, (float)-MAX_INTEGRAL, (float)MAX_INTEGRAL);
    float deriv = (error - prevError) / dt;
    prevError   = error;
    correction  = (Kp * error) + (Ki * integral) + (Kd * deriv);
  } else {
    resetPID();   // no walls — drive straight, no correction
  }

  int lSpd = constrain(BASE_SPEED - (int)correction, 0, 255);
  int rSpd = constrain(BASE_SPEED + (int)correction, 0, 255);

  motorLeft.drive(lSpd);
  motorRight.drive(rSpd);

  Serial.print("FWD | C:"); Serial.print(distCenter);
  Serial.print(" L:");      Serial.print(distLeft);
  Serial.print(" R:");      Serial.print(distRight);
  Serial.print(" Err:");    Serial.print(error);
  Serial.print(" Corr:");   Serial.println((int)correction);
}


// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== STRAIGHT + U-TURN LOOP TEST ===");

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  pinMode(LED_LEFT,  OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);

  Wire.begin();
  Wire.setClock(400000);
  wakeMPU();
  calibrateGyro();

  yaw = 0.0f;
  lastGyroUpdate = micros();

  initSensors();

  Serial.println("Starting in 2 s...");
  delay(2000);
  resetPID();
}

// ================================================================
//  MAIN LOOP
//  1. Read front sensor
//  2. If front wall <= 50mm → stop, U-turn, wall correct, resetPID
//  3. Otherwise → read side sensors, run wall-following PID straight
// ================================================================
void loop() {
  distCenter = readSensor(sensorCenter, 999);

  // ── FRONT WALL HIT ───────────────────────────────────────────
  if (distCenter <= FRONT_STOP_DIST) {
    Serial.println("Front wall detected — stopping");
    stopMotors();
    delay(150);                   // brief settle before turn

    uTurn();                      // gyro 180° + resetPID inside
    correctWallPosition();        // nudge to ideal side distance
    return;                       // skip driveStep this iteration
  }

  // ── DRIVE STRAIGHT WITH WALL PID ────────────────────────────
  distLeft  = readSensor(sensorLeft,  999);
  distRight = readSensor(sensorRight, 999);

  followWallPID();                // Process logic & drive motors
}