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

int   targetSideDistance    = 100;    // mm from left wall
int   obstacleThreshold     = 55;    // mm — front stop distance
int   WallLostThreshold     = 125;   // mm — left wall reference lost (opening/end of wall)
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

  Serial.print("Left: ");   Serial.print(leftDist_mm);   Serial.print(" mm, ");
  Serial.print("Center: "); Serial.print(centerDist_mm); Serial.print(" mm, "); 
  Serial.print("Right: ");  Serial.print(rightDist_mm);  Serial.println(" mm");
}

int wallCase()
{
  //* Compress 7 conditional cases into a single 3-bit integer lookup table map
  //* Bit 2: Left, Bit 1: Front, Bit 0: Right  
  int mask = ((leftDist_mm   > WallLostThreshold)  << 2) |
             ((centerDist_mm > obstacleThreshold)  << 1) | 
             ((rightDist_mm  > WallLostThreshold)  << 0);
            //! from line no. 36 - 39
            //! | bit wise or 
            // Using your 250mm threshold for left wall loss detection
            // Using your 55mm threshold for front obstacle detection
            // Using your 250mm threshold for right wall loss detection
  //? Map the mask result to your original 1-7 case numbers
  //?              Mask:  0b000, 0b001, 0b010, 0b011, 0b100, 0b101, 0b110, 0b111
  const int caseMap[8] = {    7,     3,     2,     5,     1,     6,     4,     0 };

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

  return caseMap[mask];
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
    int check = wallCase();  //*-- Read the distance sensors and get the clearance case

    // Grab the current running time in milliseconds
  unsigned long currentMillis = millis();

  // Handle standard 500ms blinking state tracking automatically
  if (currentMillis - lastBlinkTime >= 500) {
    lastBlinkTime = currentMillis;
    ledState = !ledState; // Toggle state between true/false every 500ms
  }

  // Handle fast 100ms blinking state tracking for Case 7 (All Blocked)
  bool fastLedState = ((currentMillis / 100) % 2 == 0);

  // 3. Evaluate your states without pausing the system
  switch (check) 
  {
    case 1: // Only left is clear -> Solid Left
      digitalWrite(LEDL, HIGH);
      digitalWrite(LEDR, LOW);
      break;

    case 2: // Only front is clear -> Both Blink together (500ms)
      digitalWrite(LEDL, ledState ? HIGH : LOW);
      digitalWrite(LEDR, ledState ? HIGH : LOW);
      break;

    case 3: // Only right is clear -> Solid Right
      digitalWrite(LEDL, LOW);
      digitalWrite(LEDR, HIGH);
      break;

    case 4: // Front and left are clear -> Left Solid, Right Blinks
      digitalWrite(LEDL, HIGH);
      digitalWrite(LEDR, ledState ? HIGH : LOW);
      break;

    case 5: // Front and right are clear -> Right Solid, Left Blinks
      digitalWrite(LEDL, ledState ? HIGH : LOW);
      digitalWrite(LEDR, HIGH);
      break;

    case 6: // Left and right are clear -> Alternating Blinking
      digitalWrite(LEDL, ledState ? HIGH : LOW);
      digitalWrite(LEDR, ledState ? LOW : HIGH); // Inverse of Left
      break;

    case 7: // No way out -> Rapid Blinking (100ms)
      digitalWrite(LEDL, fastLedState ? HIGH : LOW);
      digitalWrite(LEDR, fastLedState ? HIGH : LOW);
      break;

    default:
      digitalWrite(LEDL, LOW);
      digitalWrite(LEDR, LOW);
      break;
  }
}
