#include <Arduino.h> // For Arduino functions
#include <Wire.h>  // For I2C communication
#include <PINS.h> // For the defined pins 
#include <SparkFun_TB6612.h> // For the motor driver
#include <Adafruit_VL53L0X.h> // For the distance sensor


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
const float WHEEL_DIAMETER_MM  = 45.0;
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

int   targetSideDistance    = 95;    // mm from left wall
int   obstacleThreshold     = 55;    // mm — front stop distance
int   WallLostThreshold     = 250;   // mm — left wall reference lost (opening/end of wall)
int   wallLostTurnSpeed     = 200;   // outer-wheel PWM for the re-acquire arc turn
int   baseSpeed = 250;               //* Base forward speed (0-255)
float previousWallError = 0;
float wallIntegral      = 0;

// Sync-correction gain used while decelerating, to keep both wheels
// slowing down at the same actual rate (measured via encoders)
float Kp_sync = 2.5;

//* ENCODER ISRs  (4x quadrature resolution)

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

//? ===========================================================================================//

// Create data structures to hold the measurements
Adafruit_VL53L0X sensorCenter = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLeft   = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight  = Adafruit_VL53L0X();

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

int readSensors() 
{
  VL53L0X_RangingMeasurementData_t mLeft, mCenter, mRight;

  sensorLeft.rangingTest(&mLeft,     false);
  sensorCenter.rangingTest(&mCenter, false);
  sensorRight.rangingTest(&mRight,   false);

  // Sanitize data (Error 4 means out of bounds -> clear path)
  int left   = (mLeft.RangeStatus   != 4) ? mLeft.RangeMilliMeter   : 200;
  int center = (mCenter.RangeStatus != 4) ? mCenter.RangeMilliMeter : 999;
  int right  = (mRight.RangeStatus  != 4) ? mRight.RangeMilliMeter  : 200;

  // Determine True/False status for each direction //! from line no. 36 - 39 
  bool leftClear  = (left > WallLostThreshold);   // Using your 250mm threshold for left wall loss detection
  bool frontClear = (center > obstacleThreshold); // Using your 55mm threshold for front obstacle detection
  bool rightClear = (right > WallLostThreshold);  // Using your 250mm threshold for right wall loss detection 

  //* ==========================================
  //* CLEARANCE COMBINATIONS
  //* ==========================================
  
  // Case 1: ONLY left is clear (Front and Right wall present)
  if (leftClear && !frontClear && !rightClear) {
    return 1;
  }
  // Case 2: ONLY Front is clear (Right and left wall present)
  if (!leftClear && frontClear && !rightClear) {
    return 2;
  }
  // Case 3: ONLY Right is clear (Front and left wall present)
  if (!leftClear && !frontClear && rightClear) {
    return 3;
  }
  // Case 4: ONLY Front and Left are clear (Right wall present)
  if (leftClear && frontClear && !rightClear) {
    return 4;
  }
  // Case 5: ONLY Front and Right are clear (Left wall present)
  if (!leftClear && frontClear && rightClear) {
    return 5;
  }
  // Case 6: ONLY Left and Right are clear (Front wall present)
  if (leftClear && !frontClear && rightClear) {
    return 6;
  }
  // Case 7: No way left (Front, Left, and Right walls present)
  if (!leftClear && !frontClear && !rightClear) {
    return 7;
  }
  // Default fallback if a configuration doesn't match your 7 cases
  return 0; 
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
void wakeMPU() 
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
}

//* ======================================
void calibrateGyro() 
{
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

    Serial.print("Gyro Z: ");
    Serial.print(gyroZ);
    Serial.print(" dps   Yaw: ");
    Serial.println(yaw);
  }
}

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
    wakeMPU();        //!-- Wake up the MPU6050 gyroscope
    calibrateGyro();  //!-- Calibrate the gyroscope
    
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

//! ===========================================================================================//

void loop() 
  {
    //? ======================================

    updateYAW();     //!-- Update the yaw angle

    //? ======================================
   
    int check = readSensors();  //*-- Read the distance sensors and get the clearance case

    switch (check) 
    {
      case 1: //* Only left is clear
        digitalWrite(LEDL, HIGH);
        digitalWrite(LEDR, LOW);
        break;
      case 2: //* Only front is clear
        digitalWrite(LEDL, LOW);
        digitalWrite(LEDR, LOW);
        delay(500);
        digitalWrite(LEDL, HIGH);
        digitalWrite(LEDR, HIGH);
        delay(500);
        break;
      case 3: //* Only right is clear
        digitalWrite(LEDL, LOW);
        digitalWrite(LEDR, HIGH);
        break;
      case 4: //* Front and left are clear
        digitalWrite(LEDL, HIGH);
        digitalWrite(LEDR, LOW);
        delay(500);
        digitalWrite(LEDL, LOW);
        digitalWrite(LEDR, LOW);
        delay(500);
        digitalWrite(LEDL, HIGH);
        digitalWrite(LEDR, HIGH);
        delay(500);
        break;
      case 5: //* Front and right are clear
        digitalWrite(LEDL, LOW);
        digitalWrite(LEDR, HIGH);
        delay(500);
        digitalWrite(LEDL, LOW);
        digitalWrite(LEDR, LOW);
        delay(500);
        digitalWrite(LEDL, HIGH);
        digitalWrite(LEDR, HIGH);
        delay(500);
        break;
      case 6: //* Left and right are clear
        digitalWrite(LEDL, HIGH);
        digitalWrite(LEDR, LOW);
        delay(500);
        digitalWrite(LEDL, LOW);
        digitalWrite(LEDR, HIGH);
        delay(500);
        break;
      case 7: //* No way left (all blocked)
        digitalWrite(LEDL, LOW);
        digitalWrite(LEDR, LOW);
        delay(100);
        digitalWrite(LEDL, HIGH);
        digitalWrite(LEDR, HIGH);
        delay(100);
        break;
      default:
        digitalWrite(LEDL, LOW);
        digitalWrite(LEDR, LOW);
        break;
    }
  }
