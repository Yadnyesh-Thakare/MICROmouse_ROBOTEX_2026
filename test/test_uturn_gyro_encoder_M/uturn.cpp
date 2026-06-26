/*
 * ================================================================
 *  Micromouse — Continuous Wall Following Robot
 *  Sensors  : 3x VL53L0X ToF (Center front, Left/Right at 45°)
 *  Driver   : TB6612FNG
 *  MCU      : ESP32
 *  Encoders : ESP32Encoder (PCNT hardware quadrature)
 *
 *  Turns    : 90° → time-based spot turns
 *             180° U-turn → encoder-based closed-loop (accurate)
 *  PID      : Time-based dt, resets after every turn
 *  Post-turn: Wall correction nudges to IDEAL_SIDE_DIST
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <SparkFun_TB6612.h>
#include <ESP32Encoder.h>

// ================================================================
//  ENCODER PINS  (hardware PCNT — no ISRs needed)
// ================================================================
#define ENCL_A  19
#define ENCL_B  23
#define ENCR_A   2
#define ENCR_B  15

// ================================================================
//  SENSOR PINS
// ================================================================
#define XSHUT_CENTER  27
#define XSHUT_LEFT    13
#define XSHUT_RIGHT   33

#define ADDRESS_CENTER  0x30
#define ADDRESS_LEFT    0x31
#define ADDRESS_RIGHT   0x32

// ================================================================
//  MOTOR PINS
// ================================================================
#define RIN1   4
#define RIN2  25
#define PWMR  26
#define LIN1  17
#define LIN2   5
#define PWML  18
#define STBY  16

const int OFFSET_LEFT  = -1;
const int OFFSET_RIGHT =  1;

// ================================================================
//  LED PINS
// ================================================================
#define LED_LEFT   12
#define LED_RIGHT  32

// ================================================================
//  HARDWARE OBJECTS
// ================================================================
Motor motorLeft  = Motor(LIN1, LIN2, PWML, OFFSET_LEFT,  STBY);
Motor motorRight = Motor(RIN1, RIN2, PWMR, OFFSET_RIGHT, STBY);

ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

Adafruit_VL53L0X sensorCenter = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLeft   = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight  = Adafruit_VL53L0X();


// ================================================================
//  SENSOR READINGS  (mm)
// ================================================================
int distLeft   = 999;
int distRight  = 999;
int distCenter = 999;

// ================================================================
//  SPOT TURN TIMING  ← used only for 90° turns now
// ================================================================
const int TURN_SPEED      = 160;
const int TURN_TIME_90_MS = 400;  // ← tune this for 90°

// ================================================================
//  ENCODER U-TURN PARAMETERS
//
//  HOW TO CALIBRATE:
//  1. Measure wheel-center to wheel-center distance → set AXLE_TRACK_MM
//  2. Flash, run uTurn() standalone, read Serial: "est. angle: X.X deg"
//  3. If X < 180 → increase AXLE_TRACK_MM
//     If X > 180 → decrease AXLE_TRACK_MM
//  4. Use UTURN_OVERSHOOT_COMP (in ticks) to fine-tune braking lag
//     Increase by ~5 if robot overshoots after angle is correct.
// ================================================================
const float ENCODER_CPR         = 2800.0f;
const float WHEEL_DIAMETER_MM   = 45.0f;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER_MM;  // ≈ 141.4 mm

const float AXLE_TRACK_MM       = 90.0f;   // ← SET THIS to your measured track width

// Derived automatically — do not edit
const float TICKS_PER_DEGREE =
    ((PI * AXLE_TRACK_MM / 360.0f) / WHEEL_CIRCUMFERENCE) * ENCODER_CPR;

const int UTURN_SPEED           = 140;  // PWM during encoder U-turn ← tune
const int UTURN_OVERSHOOT_COMP  = 0;    // ticks to stop early for braking lag

// ================================================================
//  MOVEMENT PARAMETERS
// ================================================================
const int BASE_SPEED  = 200;
const int NUDGE_SPEED = 80;

const bool useLeftHandRule = true;

// ================================================================
//  SENSOR THRESHOLDS  (mm)
// ================================================================
const int SIDE_WALL_DETECT = 250;
const int IDEAL_SIDE_DIST  = 97;
const int FRONT_STOP_DIST  = 100;
const int NUDGE_TOLERANCE  = 8;

// ================================================================
//  PID CONSTANTS
// ================================================================
float Kp = 0.71f;
float Ki = 0.0f;
float Kd = 0.2f;
const int MAX_INTEGRAL = 2000;

float prevError  = 0.0f;
float integral   = 0.0f;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== MICROMOUSE — ENCODER U-TURN BUILD ===");
  Serial.print("Strategy: ");
  Serial.println(useLeftHandRule ? "LEFT-HAND" : "RIGHT-HAND");
  Serial.print("TICKS_PER_DEGREE (computed): ");
  Serial.println(TICKS_PER_DEGREE, 2);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  pinMode(LED_LEFT,  OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoderLeft.attachFullQuad(ENCL_A, ENCL_B);
  encoderRight.attachFullQuad(ENCR_A, ENCR_B);
  encoderLeft.clearCount();
  encoderRight.clearCount();

  Wire.begin();
  initSensors();

  if (useLeftHandRule) blinkLED(LED_LEFT,  3);
  else                 blinkLED(LED_RIGHT, 3);

  Serial.println("Starting in 2 s...");
  delay(2000);
  resetPID();
}

// ================================================================
//  VL53L0X INIT
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
  Serial.println("Center  OK (0x30)");

  digitalWrite(XSHUT_LEFT, HIGH); delay(10);
  if (!sensorLeft.begin(ADDRESS_LEFT)) {
    Serial.println("FATAL: Left VL53L0X!"); while (1);
  }
  Serial.println("Left    OK (0x31)");

  digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
  if (!sensorRight.begin(ADDRESS_RIGHT)) {
    Serial.println("FATAL: Right VL53L0X!"); while (1);
  }
  Serial.println("Right   OK (0x32)");
  Serial.println("All sensors ready.");
}

// ================================================================
//  SENSOR READ
// ================================================================
int readSensor(Adafruit_VL53L0X &sensor, int fallback) {
  VL53L0X_RangingMeasurementData_t m;
  sensor.rangingTest(&m, false);
  return (m.RangeStatus != 4) ? (int)m.RangeMilliMeter : fallback;
}

void readAllSensors() {
  distCenter = readSensor(sensorCenter, 999);
  distLeft   = readSensor(sensorLeft,   999);
  distRight  = readSensor(sensorRight,  999);
}
void resetPID() {
  integral  = 0.0f;
  prevError = 0.0f;
  lastTime  = millis();
}

void stopMotors() {
  motorLeft.drive(0);
  motorRight.drive(0);
}

void blinkLED(int pin, int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(150);
    digitalWrite(pin, LOW);
    delay(150);
  }
}

// ================================================================
//  TIME-BASED SPOT TURN  — for 90° turns only
// ================================================================
void spotTurn(int durationMs, bool clockwise) {
  if (clockwise) {
    motorLeft.drive( TURN_SPEED);
    motorRight.drive(-TURN_SPEED);
  } else {
    motorLeft.drive(-TURN_SPEED);
    motorRight.drive( TURN_SPEED);
  }
  delay(durationMs);
  stopMotors();
  delay(100);
}

// ================================================================
//  ENCODER-BASED SPOT TURN  — for U-turn (180°)
//
//  Polls PCNT hardware counters in a tight loop.
//  Prints calibration info to Serial after every turn.
// ================================================================
void spotTurnEncoder(float degrees, bool clockwise) {
  long targetTicks = (long)(degrees * TICKS_PER_DEGREE) - UTURN_OVERSHOOT_COMP;
  if (targetTicks < 1) targetTicks = 1;

  encoderLeft.clearCount();
  encoderRight.clearCount();

  Serial.print("EncoderTurn | target: ");
  Serial.print(targetTicks);
  Serial.print(" ticks for ");
  Serial.print(degrees, 1);
  Serial.println(" deg");

  unsigned long startMs = millis();

  if (clockwise) {
    motorLeft.drive( UTURN_SPEED);
    motorRight.drive(-UTURN_SPEED);
  } else {
    motorLeft.drive(-UTURN_SPEED);
    motorRight.drive( UTURN_SPEED);
  }

  while (true) {
    long cntL = abs((long)encoderLeft.getCount());
    long cntR = abs((long)encoderRight.getCount());
    long avg  = (cntL + cntR) / 2;

    if (avg >= targetTicks) break;

    if (millis() - startMs > 2000) {
      Serial.println("EncoderTurn | TIMEOUT — check UTURN_SPEED / wiring");
      break;
    }

    delayMicroseconds(200);
  }

  stopMotors();
  delay(80);

  // Calibration feedback
  long finalL   = abs((long)encoderLeft.getCount());
  long finalR   = abs((long)encoderRight.getCount());
  long finalAvg = (finalL + finalR) / 2;
  float actualDeg = finalAvg / TICKS_PER_DEGREE;

  Serial.print("EncoderTurn | done | L:");
  Serial.print(finalL);
  Serial.print(" R:");
  Serial.print(finalR);
  Serial.print(" avg:");
  Serial.print(finalAvg);
  Serial.print(" | est. angle: ");
  Serial.print(actualDeg, 1);
  Serial.println(" deg");
}

// ================================================================
//  POST-TURN WALL CORRECTION
// ================================================================
void correctWallPosition() {
  for (int attempt = 0; attempt < 10; attempt++) {
    distLeft  = readSensor(sensorLeft,  999);
    distRight = readSensor(sensorRight, 999);

    int refDist = 999;
    int sign    = 1;

    if (useLeftHandRule) {
      if (distLeft < SIDE_WALL_DETECT) {
        refDist = distLeft;  sign = 1;
      } else if (distRight < SIDE_WALL_DETECT) {
        refDist = distRight; sign = -1;
      }
    } else {
      if (distRight < SIDE_WALL_DETECT) {
        refDist = distRight; sign = -1;
      } else if (distLeft < SIDE_WALL_DETECT) {
        refDist = distLeft;  sign = 1;
      }
    }

    if (refDist == 999) {
      Serial.println("WallCorrect: no wall visible, skipping");
      break;
    }

    int error = refDist - IDEAL_SIDE_DIST;

    Serial.print("WallCorrect | ref:");
    Serial.print(refDist);
    Serial.print(" err:");
    Serial.println(error);

    if (abs(error) <= NUDGE_TOLERANCE) break;

    int correctedError = sign * error;
    if (correctedError > 0) {
      motorLeft.drive(-NUDGE_SPEED);
      motorRight.drive( NUDGE_SPEED);
    } else {
      motorLeft.drive( NUDGE_SPEED);
      motorRight.drive(-NUDGE_SPEED);
    }
    delay(20);
    stopMotors();
    delay(50);
  }

  resetPID();
}
void uTurn() {
  stopMotors(); delay(100);
  spotTurnEncoder(180.0f, true);   // ← encoder-closed-loop 180°
  correctWallPosition();
  blinkLED(LED_LEFT,  2);
  blinkLED(LED_RIGHT, 2);
}
void loop() {
  distCenter = readSensor(sensorCenter, 999);
  uTurn();
  delay(2000);
  correctWallPosition();
}