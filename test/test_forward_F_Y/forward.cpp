#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <SparkFun_TB6612.h>

// ==========================================
// PINS DEFINITIONS
// ==========================================
#define ENCL_A 19
#define ENCL_B 23
#define ENCR_A 2
#define ENCR_B 15

#define PWMR 26
#define RIN1 4
#define RIN2 25
#define LIN1 17
#define LIN2 5
#define PWML 18
#define STBY 16

#define XSHUT_CENTER 27
#define XSHUT_LEFT   13
#define XSHUT_RIGHT  33

#define ADDRESS_CENTER 0x30
#define ADDRESS_LEFT   0x31
#define ADDRESS_RIGHT  0x32

// ==========================================
// MOTOR & ODOMETRY SETUP
// ==========================================
const int offsetL = -1;
const int offsetR = 1;
Motor motorLeft  = Motor(LIN1, LIN2, PWML, offsetL, STBY);
Motor motorRight = Motor(RIN1, RIN2, PWMR, offsetR, STBY);

volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;

// ==========================================
// TUNING PARAMETERS
// ==========================================
float Kp_wall = 0.5; 
float Ki_wall = 0.0; 
float Kd_wall = 0.4; 

int targetWallDistance = 95;   // mm
int obstacleThreshold  = 100;  // mm (Stop when front wall is this close)
int WallLostThreshold  = 125;  // mm

int leftDist_mm = 1000, centerDist_mm = 1000, rightDist_mm = 1000;
float previousWallError = 0;
float wallIntegral      = 0;

VL53L0X sensorLeft, sensorCenter, sensorRight;

// ==========================================
// CRASH-SAFE ENCODER ISRs
// ==========================================
void IRAM_ATTR leftEncoderAISR()  { leftEncoderCount  += (GPIO.in >> ENCL_A & 1) == (GPIO.in >> ENCL_B & 1) ? -1 : 1; }
void IRAM_ATTR leftEncoderBISR()  { leftEncoderCount  += (GPIO.in >> ENCL_A & 1) != (GPIO.in >> ENCL_B & 1) ? -1 : 1; }
void IRAM_ATTR rightEncoderAISR() { rightEncoderCount += (GPIO.in >> ENCR_A & 1) == (GPIO.in >> ENCR_B & 1) ? -1 : 1; }
void IRAM_ATTR rightEncoderBISR() { rightEncoderCount += (GPIO.in >> ENCR_A & 1) != (GPIO.in >> ENCR_B & 1) ? -1 : 1; }

// ==========================================
// NON-BLOCKING SENSOR SETUP
// ==========================================
void initSensors() {
  pinMode(XSHUT_CENTER, OUTPUT); pinMode(XSHUT_LEFT, OUTPUT); pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_CENTER, LOW); digitalWrite(XSHUT_LEFT, LOW); digitalWrite(XSHUT_RIGHT, LOW);
  delay(50); 

  auto initIndiv = [](VL53L0X& s, int pin, uint8_t addr) {
    digitalWrite(pin, HIGH); delay(50); 
    s.setAddress(addr);
    if (!s.init()) { Serial.printf("FATAL: Sensor %X dead.\n", addr); while(1); }
    s.setTimeout(200);
    s.startContinuous();
  };

  initIndiv(sensorCenter, XSHUT_CENTER, ADDRESS_CENTER);
  initIndiv(sensorLeft,   XSHUT_LEFT,   ADDRESS_LEFT);
  initIndiv(sensorRight,  XSHUT_RIGHT,  ADDRESS_RIGHT);
}

void readSensors() {
  uint16_t l = sensorLeft.readRangeContinuousMillimeters();
  uint16_t c = sensorCenter.readRangeContinuousMillimeters();
  uint16_t r = sensorRight.readRangeContinuousMillimeters();

  leftDist_mm   = (l > 2000 || sensorLeft.timeoutOccurred())   ? 1000 : l;
  centerDist_mm = (c > 2000 || sensorCenter.timeoutOccurred()) ? 1000 : c;
  rightDist_mm  = (r > 2000 || sensorRight.timeoutOccurred())  ? 1000 : r;
}

void resetWallPID() {
  wallIntegral      = 0.0;
  previousWallError = 0.0;
}

// ==========================================
// DRIVE UNTIL WALL DETECTED
// ==========================================
void driveUntilWall(int baseSpeed)
{
  resetWallPID();
  Serial.println("Driving forward until front wall is detected...");

  while (true)
  {
    // 1. Instantly read background sensor data
    readSensors();

    // 2. Front Obstacle Check (The ONLY exit condition)
    if (centerDist_mm <= obstacleThreshold) {
        Serial.printf("FRONT WALL DETECTED (%d mm) — Braking.\n", centerDist_mm);
        break;
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

    // 4. Set Motor Adjustments
    int leftSpeed  = baseSpeed - (int)wallCorrection;
    int rightSpeed = baseSpeed + (int)wallCorrection;

    motorLeft.drive(constrain(leftSpeed, 0, 255));
    motorRight.drive(constrain(rightSpeed, 0, 255));

    // Non-blocking serial telemetry
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      Serial.printf("Dist L:%d C:%d R:%d | Err: %.1f | Corr: %.1f\n", 
                    leftDist_mm, centerDist_mm, rightDist_mm, wallError, wallCorrection);
    }
    delay(1);
  }

  // Active braking when the front wall limit is breached
  motorLeft.brake();
  motorRight.brake();
  resetWallPID();
}

void setup() {
  Serial.begin(115200);
  
  pinMode(ENCL_A, INPUT_PULLUP); pinMode(ENCL_B, INPUT_PULLUP);
  pinMode(ENCR_A, INPUT_PULLUP); pinMode(ENCR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCL_A), leftEncoderAISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCL_B), leftEncoderBISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCR_A), rightEncoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCR_B), rightEncoderBISR, CHANGE);

  Wire.begin();
  Wire.setClock(400000); 
  initSensors();

  Serial.println("System Ready.");
  delay(3000);
}

void loop() {
  // Drive forward at speed 180 until it hits a front wall
  driveUntilWall(180); 

  Serial.println("Run stopped. Pausing for 5 seconds.");
  delay(5000);
}