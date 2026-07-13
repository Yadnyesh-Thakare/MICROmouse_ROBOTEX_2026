/*
 * Micromouse: Enhanced Floodfill Robot (Optimized for Narrow Corridors)
 * Modified with 31-second timer and mode selection for wall following
 * Tailored for ESP32 with TB6612FNG, tripel VL53 sensors, and N20 encoders.
 * Optimized for 18cm corridors with 15cm robot width.
 *
 * IMPORTANT: You MUST calibrate the values in the "Robot Physical Constants"
 * and "Sensor Calibration" sections for your robot to work correctly.
 */
#include <Arduino.h>
#include <Wire.h> // For I2C communication
#include <SparkFun_TB6612.h> // For the motor driver
#include <queue>
#include <math.h> // For sigmoid function (exp)
#include <M26_PINS.h> // For the defined pins
#include <Adafruit_VL53L0X.h>
#include <ESP32Encoder.h>


// =================================================================
// ================= TIMING AND MODE SELECTION ====================
// =================================================================

// Timer constants
const unsigned long WALL_FOLLOW_TIMER_DURATION = 5000; // 31 = 31000 seconds in milliseconds
unsigned long wallFollowStartTime = 0;
bool timerActive = false;

// Wall follow mode selection
enum WallFollowMode 
{
    ONLY_RIGHT_WALL, // Continue with right wall follow only
    SWITCH_TO_LEFT_WALL, // Switch to left wall follow after timer
    LEFT_WALL_ONLY // Only left wall follow (direct selection)
};
WallFollowMode selectedMode = ONLY_RIGHT_WALL;

// =================================================================
// =================================================================

// Global variables to hold sensor readings
unsigned int proxL;
unsigned int proxR;
float frontVal;

// --- Hardware Objects ---
Motor motorLeft = Motor(AIN1, AIN2, PWMA, 1, STBY_PIN);
Motor motorRight = Motor(BIN1, BIN2, PWMB, 1, STBY_PIN);

ESP32Encoder leftEncoderCount;
ESP32Encoder rightEncoderCount;

// Create data structures to hold the measurements
Adafruit_VL53L0X sensorCenter = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLeft   = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight  = Adafruit_VL53L0X();


// --- Robot Physical Constants ---
const float WHEEL_DIAMETER_CM = 5.0;
const float WHEEL_SEPARATION_CM = 10.0;
const int ENCODER_CPR = 280;
const float CM_PER_TICK = (PI * WHEEL_DIAMETER_CM) / ENCODER_CPR;
#define CELL_DISTANCE_CM 18.0

// --- Movement & Control Parameters (ENHANCED) ---
#define TURN_DEGREES_90 95
#define TURN_DEGREES_180 195
#define FAST_TURN_DEGREES_90 105
#define FAST_TURN_DEGREES_180 189

const int BASE_SPEED = 130;
const int TURN_SPEED = 100;
const int FAST_TURN_SPEED = 75;

// --- OPTIMIZED ENCODER PID PARAMETERS ---
float encoderKp = 0.7, encoderKi = 0.005, encoderKd = 0.15;
float encoderError = 0, encoderPrev = 0, encoderInt = 0;
float encoderPIDValue = 0;

// --- OPTIMIZED WALL PID PARAMETERS ---
// float wallKp = 1.5, wallKi = 0.02, wallKd = 12.0; // Single wall

float wallKp = 1.5, wallKi = 0.01, wallKd = 8.0; 
float wallKpDual = 1.8, wallKiDual = 0.01, wallKdDual = 8.0; // Dual wall
float wallError = 0, wallPrev = 0, wallInt = 0;
float wallPIDValue = 0;

#define MAX_INTEGRAL 25 // Reduced for better stability

// MPU6050 GYROSCOPE FUNCTIONS
float gyroBiasZ = 0.0;  // stored in dps
float yaw = 0.0;
unsigned long lastUpdate = 0;

// --- Enhanced Sigmoidal Speed Parameters ---
struct SpeedProfile 
{
    int startSpeed;
    int endSpeed; 
    float sigmoidScale;
    float transitionPoint; // Where to switch from accel to decel
    float smoothingFactor; // Additional smoothing
};

// Exploration mode (safer for narrow corridors)
SpeedProfile explorationProfile = 
{
    80, // startSpeed - Reduced for tight spaces
    50, // endSpeed - Lower for precise stopping
    8.0, // sigmoidScale - Moderate transition steepness
    0.55, // transitionPoint - Start slowing at 60% of distance
    0.3 // smoothingFactor - Additional velocity smoothing
};

// Fast run modes
SpeedProfile fastProfile; // Global variable to hold the selected fast profile

// Profile A (for BTNL press)
SpeedProfile fastProfileA = 
{
    150, // startSpeed - Reduced from 160 for safety in narrow corridors
    110, // endSpeed - Still controlled ending
    25.0, // sigmoidScale - Sharper transitions
    0.33, // transitionPoint - Later deceleration
    0.4 // smoothingFactor - Less smoothing for speed
};

// Profile B (for BTNR press)
SpeedProfile fastProfileB = 
{
    120, // startSpeed - Reduced from 160 for safety in narrow corridors
    100, // endSpeed - Still controlled ending
    20.0, // sigmoidScale - Sharper transitions
    0.33, // transitionPoint - Later deceleration
    0.4 // smoothingFactor - Less smoothing for speed
};


// --- Sensor Calibration ---
const int LEFT_WALL_THRESHOLD = 135;
const int RIGHT_WALL_THRESHOLD = 135;
const int FRONT_WALL_THRESHOLD = 135;
int RIGHT_WALL_TARGET = 95;
int LEFT_WALL_TARGET = 95;

// =================================================================
// ============== CONFIGURABLE START & GOAL SETTINGS ===============
// =================================================================
// --- SET YOUR START POSITION AND HEADING HERE ---
#define START_X 0
#define START_Y 0
#define START_HEADING NORTH // Can be NORTH, EAST, SOUTH, or WEST

// --- SET YOUR GOAL CELLS HERE ---
// For an 8x8 maze, a common goal is the center: {{3,3}, {3,4}, {4,3}, {4,4}}.
struct Point { int x; int y; };
const Point goalCells[] = {{15, 15}};
const int numGoalCells = sizeof(goalCells) / sizeof(Point);

// --- Global State & Maze Variables ---
const int GRID_SIZE = 15;
int distMap[GRID_SIZE][GRID_SIZE];
uint8_t wallsMap[GRID_SIZE][GRID_SIZE];
bool visited[GRID_SIZE][GRID_SIZE];

#define WALL_N 1
#define WALL_E 2
#define WALL_S 4
#define WALL_W 8

enum Heading { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

int robotX = 0, robotY = 0;
Heading robotHeading = EAST;

// Modified Program States to include timed wall following
enum ProgramState 
{
    FOLLOW_LEFT, // Continuous left wall follow 
    EXPLORING, 
    AWAITING_FAST_RUN, 
    FAST_RUN, 
    FINISHED 
};
ProgramState currentState = FOLLOW_LEFT;


// Button debouncing variables
unsigned long lastButton1Press = 0;
unsigned long lastButton2Press = 0;
const unsigned long DEBOUNCE_DELAY = 10; // 250ms debounce delay

long duration;
int WF_FRONT_WALL_THRESHOLD = 135; // mm threshold for front block

// --- Global readings (APDS mapped 0..500; ultrasonic cm as float)
// --- Right-wall PID (single loop)
// --- Targets and thresholds
int WALL_TARGET = 95; // Increased for better wall tracking
#define WALL_THRESHOLD 135 // proxR > this => right wall present

float WF_wallKp = 1.5, WF_wallKi = 0.01, WF_wallKd = 8.0;
#define WF_MAX_INTEGRAL 25

// --- Motion base
int WF_BASE_SPEED = 140; // Slightly reduced for stability

// --- Right-wall reacquire arc
int ARC_SPEED_OUTER = 160; // Reduced difference for stability
int ARC_SPEED_INNER = 69; // Closer speeds = smoother arc
int PID_ARC_CLAMP = 25; // Reduced clamp for precision

// --- Spin Parameters (NEW) ---
int SPIN_SPEED = 100; // Moderate speed for control
int SPIN_DURATION = 340; // Longer duration for proper turn

// --- FSM states (minimal)
enum TurnState 
{
    NONE,
    LEFT_FIND_ALIGN,
    ARC_RIGHT_FIND
};
TurnState turnState = NONE;
unsigned long stateStart = 0;
unsigned long leftAlignStart = 0;
bool leftAlignStable = false;

// --- Improved PID weight distribution ---
const float encoderWeight = 0.35; // Reduced encoder influence
const float wallWeight = 0.65; // Increased wall influence

// --- Function Prototypes ---
void computeFloodfill(int gx, int gy);
void computeFloodfillVisited(int gx, int gy);
void runExploration();
void performFastRun();
void pathReturnToStart();
void attachHardware();
void updateYAW();
bool nextCellTowardsGoal(int &nx, int &ny);
bool nextCellTowardsGoalFinal(int &nx, int &ny);
void senseWallsAtCurrentCell();
void executeMoveTo(int nx, int ny);
void executeMoveToFast(int nx, int ny);
void moveForwardOneCell();
void moveForwardOneCellFast();
void turn(float degrees, int speed);
bool inBounds(int x, int y);
void stopMotors();
void readAllSensors();
float readFrontSensor();
void IRAM_ATTR isrLeftEncoder();
void IRAM_ATTR isrRightEncoder();
void wallPID(unsigned int proxL, unsigned int proxR);
void encoderPID();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void WF_wallPID_R(unsigned int proxR_);
void WF_wallPID_L(unsigned int proxL_);
void followRightWall();
void followLeftWall();
void handleTimedWallFollow();
bool checkButton(int buttonPin, unsigned long &lastPressTime);
void selectFastRunMode();

// ------------------- Setup -------------------
void setup() 
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("Micromouse Hardware Mode Initializing...");
    
    attachHardware();
    
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            wallsMap[y][x] = 0;
            distMap[y][x] = 10000;
            visited[y][x] = false;
        }
    }
    
    for (int y = 0; y < GRID_SIZE; y++) {
        wallsMap[y][0] |= WALL_W;
        wallsMap[y][GRID_SIZE - 1] |= WALL_E;
    }
    for (int x = 0; x < GRID_SIZE; x++) {
        wallsMap[0][x] |= WALL_S;
        wallsMap[GRID_SIZE - 1][x] |= WALL_N;
    }
    
    robotX = START_X;
    robotY = START_Y;
    robotHeading = START_HEADING;
    visited[robotY][robotX] = true;
    
    computeFloodfill(goalCells[0].x, goalCells[0].y);
    
    pinMode(BTNL, INPUT_PULLDOWN);
    pinMode(BTNR, INPUT_PULLDOWN);
    
    Serial.println("=== ROBOT MODE SELECTION ===");
    Serial.println("Button 1: Start Right Wall Follow (with 31s timer)");
    Serial.println("Button 2: Start Exploration Mode");
    Serial.println("Waiting for button press...");
    
    // Initial mode selection
    while(1) {
        if(checkButton(BTNL, lastButton1Press)) 
        {
            digitalWrite(LEDR, HIGH);
            digitalWrite(LEDL, HIGH);
            delay(500);
            digitalWrite(LEDR, LOW);
            digitalWrite(LEDL, LOW);
            
            // Select wall follow mode after timer expires
            followRightWall();
            
            currentState = FOLLOW_LEFT; //! can be set as Right or Left wall follow 
            
            Serial.println("Started: 31-second Right or Left Wall Follow with timer");
            break;
        }
        if(checkButton(BTNR, lastButton2Press)) 
        {
            digitalWrite(LEDR, HIGH);
            digitalWrite(LEDL, HIGH);
            delay(500);
            digitalWrite(LEDR, LOW);
            digitalWrite(LEDL, LOW);
            
            // Check for fast run mode selection before starting exploration
            selectFastRunMode();
            
            currentState = EXPLORING;
            Serial.println("Started: Exploration Mode");
            break;
        }
    }
}

// ------------------- Main loop -------------------
void loop() 
{   
    senseWallsAtCurrentCell();
    updateYAW();
    readAllSensors();
    moveForwardOneCell();
    
}

// ================= NEW TIMER AND BUTTON FUNCTIONS =================



bool checkButton(int buttonPin, unsigned long &lastPressTime) 
{
    unsigned long currentTime = millis();
    
    if (digitalRead(buttonPin) == HIGH && (currentTime - lastPressTime) > DEBOUNCE_DELAY) 
    {
        lastPressTime = currentTime;
        return true;
    }
    return false;
}


// Function to select the fast run speed profile
void selectFastRunMode() 
{
    Serial.println("\n=== SELECT FAST RUN PROFILE ===");
    Serial.println("Button 1: Normal Fast Run Profile");
    Serial.println("Button 2: Slower, More Controlled Fast Run Profile");
    Serial.println("Waiting for selection...");
    
    lastButton1Press = 0;
    lastButton2Press = 0;
    delay(500);
    
    while(1) {
        if(checkButton(BTNL, lastButton1Press)) {
            fastProfile = fastProfileA;
            Serial.println("Selected: Normal Fast Run Profile.");
            digitalWrite(LEDR, HIGH);
            delay(300);
            digitalWrite(LEDR, LOW);
            break;
        }
        if(checkButton(BTNR, lastButton2Press)) {
            fastProfile = fastProfileB;
            Serial.println("Selected: Slower Fast Run Profile.");
            digitalWrite(LEDL, HIGH);
            delay(300);
            digitalWrite(LEDL, LOW);
            delay(200);
            digitalWrite(LEDL, HIGH);
            delay(300);
            digitalWrite(LEDL, LOW);
            break;
        }
    }
    delay(500);
}

// =================================================================
// ================= HARDWARE-SPECIFIC FUNCTIONS ===================
// =================================================================


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

    //Serial.print("Gyro Z: ");
    //Serial.print(gyroZ);
    //Serial.print(" dps   Yaw: ");
    //Serial.println(yaw);
  }
}

void attachHardware() 
{
    Wire.begin();

    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, HIGH);

    pinMode(LEDR, OUTPUT);
    pinMode(LEDL, OUTPUT);
    
    pinMode(XSHUT_CENTER, OUTPUT);
    pinMode(XSHUT_LEFT,   OUTPUT);
    pinMode(XSHUT_RIGHT,  OUTPUT);

    initSensors();
    WKnC_MPU();
    
    leftEncoderCount.attachHalfQuad(ENCL_A, ENCL_B);
    rightEncoderCount.attachHalfQuad(ENCR_A, ENCR_B);

    leftEncoderCount.clearCount();
    rightEncoderCount.clearCount();

    Serial.println("All sensors initialised.");
    Serial.println("Hardware attached.");
}


void stopMotors() 
{
    motorLeft.brake();
    motorRight.brake();
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) 
{
    motorLeft.drive(leftSpeed);
    motorRight.drive(rightSpeed);
}

void readAllSensors() 
{
    VL53L0X_RangingMeasurementData_t mLeft, mCenter, mRight;

    sensorLeft.rangingTest(&mLeft,     false);
    sensorCenter.rangingTest(&mCenter, false);
    sensorRight.rangingTest(&mRight,   false);

    //* Cap out-of-range readings to safe defaults
    proxL = (mLeft.RangeStatus   != 4) ? mLeft.RangeMilliMeter   : 200;
    frontVal = (mCenter.RangeStatus != 4) ? mCenter.RangeMilliMeter : 800;
    proxR  = (mRight.RangeStatus  != 4) ? mRight.RangeMilliMeter  : 200;
}

void senseWallsAtCurrentCell() 
{
    uint8_t cellWalls = wallsMap[robotY][robotX];
    readAllSensors();
    
    bool leftWall = proxL < LEFT_WALL_THRESHOLD;
    bool rightWall = proxR < RIGHT_WALL_THRESHOLD;
    bool frontWall = frontVal < FRONT_WALL_THRESHOLD;
    
    digitalWrite(LEDR, rightWall ? HIGH : LOW);
    digitalWrite(LEDL, frontWall ? HIGH : LOW);
    
    Serial.print("Sensing at ("); Serial.print(robotX); Serial.print(","); Serial.print(robotY); Serial.println("):");
    if(leftWall) Serial.println(" -> Left Wall");
    if(rightWall) Serial.println(" -> Right Wall");
    if(frontWall) Serial.println(" -> Front Wall");
    
    if (leftWall) 
    { 
        if (robotHeading == NORTH) cellWalls |= WALL_W; 
        else if (robotHeading == EAST) cellWalls |= WALL_N; 
        else if (robotHeading == SOUTH) cellWalls |= WALL_E; 
        else cellWalls |= WALL_S; 
    }
    if (rightWall) 
    { 
        if (robotHeading == NORTH) cellWalls |= WALL_E; 
        else if (robotHeading == EAST) cellWalls |= WALL_S; 
        else if (robotHeading == SOUTH) cellWalls |= WALL_W; 
        else cellWalls |= WALL_N; 
    }
    if (frontWall) 
    { 
        if (robotHeading == NORTH) cellWalls |= WALL_N; 
        else if (robotHeading == EAST) cellWalls |= WALL_E; 
        else if (robotHeading == SOUTH) cellWalls |= WALL_S; 
        else cellWalls |= WALL_W; 
    }
    
    wallsMap[robotY][robotX] = cellWalls;
    
    if ((cellWalls & WALL_N) && inBounds(robotX, robotY + 1)) wallsMap[robotY + 1][robotX] |= WALL_S;
    if ((cellWalls & WALL_E) && inBounds(robotX + 1, robotY)) wallsMap[robotY][robotX + 1] |= WALL_W;
    if ((cellWalls & WALL_S) && inBounds(robotX, robotY - 1)) wallsMap[robotY - 1][robotX] |= WALL_N;
    if ((cellWalls & WALL_W) && inBounds(robotX - 1, robotY)) wallsMap[robotY][robotX - 1] |= WALL_E;
}


void followRightWall()
{
    readAllSensors();
    bool hasRightWall = proxR < WALL_THRESHOLD;
    bool hasLeftWall = proxL < WALL_THRESHOLD;
    bool frontBlocked = frontVal < WF_FRONT_WALL_THRESHOLD;
    
    digitalWrite(LEDR, hasRightWall ? HIGH : LOW);
    digitalWrite(LEDL, hasLeftWall ? HIGH : LOW);
    
    if (!hasRightWall) 
    {
        WF_wallPID_R(0); // Target 0 when no wall to guide toward right
        float corr = constrain(wallPIDValue, -PID_ARC_CLAMP, PID_ARC_CLAMP);
        int leftSpeed = constrain(ARC_SPEED_OUTER - (int)corr, -255, 255);
        int rightSpeed = constrain(ARC_SPEED_INNER + (int)corr, -255, 255);
        setMotorSpeeds(leftSpeed, rightSpeed);
    }
    else if (frontBlocked) 
    { 
        // IMPROVED SPIN: Proper duration and speed
        setMotorSpeeds(-SPIN_SPEED, SPIN_SPEED);
        // delay(10);
    }
    else 
    {
        // NORMAL WALL FOLLOWING: More precise PID
        WF_wallPID_R(proxR);
        int leftSpeed = constrain(WF_BASE_SPEED - (int)wallPIDValue, -255, 255);
        int rightSpeed = constrain(WF_BASE_SPEED + (int)wallPIDValue, -255, 255);
        setMotorSpeeds(leftSpeed, rightSpeed);
    }
}

void followLeftWall()
{
    readAllSensors();
    bool hasRightWall = proxR < WALL_THRESHOLD;
    bool hasLeftWall = proxL < WALL_THRESHOLD;
    bool frontBlocked = frontVal < WF_FRONT_WALL_THRESHOLD;
    
    digitalWrite(LEDR, hasRightWall ? HIGH : LOW);
    digitalWrite(LEDL, hasLeftWall ? HIGH : LOW);
    
    if (!hasLeftWall) 
    {
        // Arc LEFT to find the left wall (right motor faster)
        WF_wallPID_L(0); // Target 0 when no wall to guide toward left
        float corr = constrain(wallPIDValue, -PID_ARC_CLAMP, PID_ARC_CLAMP);
        int leftSpeed = constrain(ARC_SPEED_INNER - (int)corr, -255, 255);
        int rightSpeed = constrain(ARC_SPEED_OUTER + (int)corr, -255, 255);
        setMotorSpeeds(leftSpeed, rightSpeed);
    }
    else if (frontBlocked) 
    { 
        // Turn RIGHT when front is blocked (left wall following)
        setMotorSpeeds(SPIN_SPEED, -SPIN_SPEED);
        // delay(10);
    }
    else 
     {
        // NORMAL WALL FOLLOWING: Follow left wall with PID
        WF_wallPID_L(proxL);
        int leftSpeed = constrain(WF_BASE_SPEED - (int)wallPIDValue, -255, 255);
        int rightSpeed = constrain(WF_BASE_SPEED + (int)wallPIDValue, -255, 255);
        setMotorSpeeds(leftSpeed, rightSpeed);
    }
}

void WF_wallPID_R(unsigned int proxR_) 
{
    wallError =  WALL_TARGET - (float)proxR_;
    
    // Integral with windup protection
    wallInt += wallError;
    wallInt = constrain(wallInt, -WF_MAX_INTEGRAL, WF_MAX_INTEGRAL);
    
    float wallDeriv = wallError - wallPrev;
    wallPrev = wallError;
    wallPIDValue = WF_wallKp * wallError + WF_wallKi * wallInt + WF_wallKd * wallDeriv;
}

void WF_wallPID_L(unsigned int proxL_) 
{
    wallError = - (float)proxL_ - WALL_TARGET; 
    
    // Integral with windup protection
    wallInt += wallError;
    wallInt = constrain(wallInt, -WF_MAX_INTEGRAL, WF_MAX_INTEGRAL);
    
    float wallDeriv = wallError - wallPrev;
    wallPrev = wallError;
    wallPIDValue = WF_wallKp * wallError + WF_wallKi * wallInt + WF_wallKd * wallDeriv;
}

// Enhanced Wall PID with adaptive parameters
void wallPID(unsigned int proxL, unsigned int proxR) {
    bool hasBothWalls = (proxL > LEFT_WALL_THRESHOLD && proxR > RIGHT_WALL_THRESHOLD);
    
    // Use different parameters for single vs dual wall scenarios
    float currentKp = hasBothWalls ? wallKpDual : wallKp;
    float currentKi = hasBothWalls ? wallKiDual : wallKi; 
    float currentKd = hasBothWalls ? wallKdDual : wallKd;
    
    wallError = -(int)proxR + (int)proxL;
    wallInt += wallError;
    wallInt = constrain(wallInt, -MAX_INTEGRAL, MAX_INTEGRAL);
    
    float wallDeriv = wallError - wallPrev;
    wallPrev = wallError;
    
    wallPIDValue = currentKp * wallError + currentKi * wallInt + currentKd * wallDeriv;
}

// Enhanced Encoder PID with integral windup protection
void encoderPID() 
{
    encoderError = leftEncoderCount.getCount() - rightEncoderCount.getCount();
    
    // Integral windup protection
    if (abs(encoderPIDValue) < 100) { // Only accumulate when output isn't saturated
        encoderInt += encoderError;
        encoderInt = constrain(encoderInt, -MAX_INTEGRAL, MAX_INTEGRAL);
    }
    
    float encoderDeriv = encoderError - encoderPrev;
    encoderPrev = encoderError;
    
    encoderPIDValue = encoderKp * encoderError + encoderKi * encoderInt + encoderKd * encoderDeriv;
}

void moveForwardOneCell() 
{
    long targetTicks = CELL_DISTANCE_CM / CM_PER_TICK;
    
    leftEncoderCount.clearCount();
    rightEncoderCount.clearCount();
    
    // Use exploration profile for narrow corridors
    SpeedProfile profile = explorationProfile;
    
    // Dynamic speed variables
    int currentSpeed = profile.startSpeed;
    int previousSpeed = profile.startSpeed;
    
    while (true) {
        long avgTicks = (leftEncoderCount.getCount() + rightEncoderCount.getCount())/2;
        if (avgTicks >= targetTicks) break;
        
        float progress = (float)avgTicks / (float)targetTicks;
        
        // --- ENHANCED DUAL-PHASE SIGMOIDAL CONTROL ---
        int dynamicSpeed;
        
        if (progress <= profile.transitionPoint) {
            // Acceleration phase - smooth startup
            float accelProgress = progress / profile.transitionPoint;
            float sigmoidInput = profile.sigmoidScale * (accelProgress - 0.5);
            float sigmoidOutput = 1.0 / (1.0 + exp(-sigmoidInput));
            dynamicSpeed = profile.endSpeed + (int)((profile.startSpeed - profile.endSpeed) * sigmoidOutput);
        } else {
            // Deceleration phase - smooth stopping
            float decelProgress = (progress - profile.transitionPoint) / (1.0 - profile.transitionPoint);
            
            // Enhanced S-curve deceleration
            float sigmoidInput = profile.sigmoidScale * (0.5 - decelProgress);
            float sigmoidOutput = 1.0 / (1.0 + exp(-sigmoidInput));
            dynamicSpeed = profile.endSpeed + (int)((profile.startSpeed - profile.endSpeed) * sigmoidOutput);
        }
        
        // --- VELOCITY SMOOTHING FOR NARROW CORRIDORS ---
        currentSpeed = (int)(profile.smoothingFactor * previousSpeed + 
                             (1.0 - profile.smoothingFactor) * dynamicSpeed);
        currentSpeed = constrain(currentSpeed, profile.endSpeed, profile.startSpeed);
        previousSpeed = currentSpeed;
        
        // Early stopping for front obstacles (critical in narrow spaces)
        readAllSensors();
        if (frontVal < 4) break; // Tighter stopping distance
        
        // --- ENHANCED WALL PID FOR NARROW CORRIDORS ---
        wallPIDValue = 0;
        bool hasLeftWall = proxL > LEFT_WALL_THRESHOLD;
        bool hasRightWall = proxR > RIGHT_WALL_THRESHOLD;
        
        // Critical: Both walls present in narrow corridor
        if (hasLeftWall && hasRightWall) {
            digitalWrite(LEDL, HIGH); 
            digitalWrite(LEDR, HIGH);
            
            // Enhanced dual-wall PID for tight spaces
            wallError = -(int)proxR + (int)proxL;
            
            // Aggressive correction for narrow corridors
            float narrowKp = 1.2; // Higher response for tight spaces
            float narrowKi = 0.015; // Moderate integral
            float narrowKd = 10.0; // Strong damping
            
            wallInt += wallError;
            wallInt = constrain(wallInt, -15, 15); // Tighter integral limits
            
            float wallDeriv = wallError - wallPrev;
            wallPrev = wallError;
            
            wallPIDValue = narrowKp * wallError + narrowKi * wallInt + narrowKd * wallDeriv;
            
        } else if (hasRightWall) {
            digitalWrite(LEDL, LOW); 
            digitalWrite(LEDR, HIGH);
            wallError = (float)proxR - RIGHT_WALL_TARGET;
            wallInt += wallError; 
            float wallDeriv = wallError - wallPrev; 
            wallPrev = wallError;
            wallPIDValue = 1.8 * wallError + 0.025 * wallInt + 15.0 * wallDeriv;
            
        } else if (hasLeftWall) {
            digitalWrite(LEDL, HIGH); 
            digitalWrite(LEDR, LOW);
            wallError = LEFT_WALL_TARGET - (float)proxL;
            wallInt += wallError; 
            float wallDeriv = wallError - wallPrev; 
            wallPrev = wallError;
            wallPIDValue = 1.8 * wallError + 0.025 * wallInt + 15.0 * wallDeriv;
        } else {
            digitalWrite(LEDL, LOW); 
            digitalWrite(LEDR, LOW);
        }
        
        encoderPID();
        
        // --- ADJUSTED CONTROL WEIGHTS FOR NARROW SPACES ---
        const float encoderWeight = 0.25; // Reduced encoder influence
        const float wallWeight = 0.75; // Increased wall priority
        float totalCorrection = (encoderWeight * encoderPIDValue) + (wallWeight * wallPIDValue);
        
        // Tighter correction limits for narrow corridors
        totalCorrection = constrain(totalCorrection, -40, 40);
        
        int leftSpeed = constrain(currentSpeed + totalCorrection, -255, 255);
        int rightSpeed = constrain(currentSpeed - totalCorrection, -255, 255);
        setMotorSpeeds(leftSpeed, rightSpeed);
    }
    
    stopMotors();
    delay(5); // Slightly longer settling time
}

void moveForwardOneCellFast() 
{
    long targetTicks = CELL_DISTANCE_CM / CM_PER_TICK;
    
    leftEncoderCount.clearCount();
    rightEncoderCount.clearCount();

    // Use the globally selected fast run profile
    SpeedProfile profile = fastProfile;
    int currentSpeed = profile.startSpeed;
    int previousSpeed = profile.startSpeed;

    encoderInt = 0; encoderPrev = 0; wallInt = 0; wallPrev = 0;

    while (true) 
    {
        long avgTicks = (leftEncoderCount.getCount() + rightEncoderCount.getCount()) / 2;
        if (avgTicks >= targetTicks) break;

        float progress = (float)avgTicks / (float)targetTicks;

        // ---- Sigmoidal Speed Profile (accel/decel) ----
        int dynamicSpeed;
        if (progress <= profile.transitionPoint) 
        {
            // Acceleration phase
            float accelProgress = progress / profile.transitionPoint;
            float sigmoidInput = profile.sigmoidScale * (accelProgress - 0.5);
            float sigmoidOutput = 1.0 / (1.0 + exp(-sigmoidInput));
            dynamicSpeed = profile.endSpeed + (int)((profile.startSpeed - profile.endSpeed) * sigmoidOutput);
        } 
        else 
        {
            // Deceleration phase
            float decelProgress = (progress - profile.transitionPoint) / (1.0 - profile.transitionPoint);
            float sigmoidInput = profile.sigmoidScale * (0.5 - decelProgress);
            float sigmoidOutput = 1.0 / (1.0 + exp(-sigmoidInput));
            dynamicSpeed = profile.endSpeed + (int)((profile.startSpeed - profile.endSpeed) * sigmoidOutput);
        }

        // --- Velocity Smoothing ---
        currentSpeed = (int)(profile.smoothingFactor * previousSpeed + 
                             (1.0 - profile.smoothingFactor) * dynamicSpeed);
        currentSpeed = constrain(currentSpeed, profile.endSpeed, profile.startSpeed);
        previousSpeed = currentSpeed;

        readAllSensors();
        if (frontVal < 8) break; // Slightly longer stop for higher speed

        wallPIDValue = 0;
        bool hasLeftWall = proxL > LEFT_WALL_THRESHOLD;
        bool hasRightWall = proxR > RIGHT_WALL_THRESHOLD;

        // --- Adaptive Dual Wall PID (fast mode) ---
        if (hasLeftWall && hasRightWall) 
        {
            float narrowKp = 2.0; // stronger correction for speed
            float narrowKi = 0.03;
            float narrowKd = 10.0;

            wallError = -(int)proxR + (int)proxL;
            wallInt += wallError;
            wallInt = constrain(wallInt, -15, 15);

            float wallDeriv = wallError - wallPrev;
            wallPrev = wallError;

            wallPIDValue = narrowKp * wallError + narrowKi * wallInt + narrowKd * wallDeriv;
        } 
        else if (hasRightWall) 
        {
            wallError = RIGHT_WALL_TARGET - (float)proxR;
            wallInt += wallError; 
            float wallDeriv = wallError - wallPrev; 
            wallPrev = wallError;
            wallPIDValue = 2.1 * wallError + 0.02 * wallInt + 13.0 * wallDeriv;
        } 
        else if (hasLeftWall) 
        {
            wallError = (float)proxL - LEFT_WALL_TARGET;
            wallInt += wallError; 
            float wallDeriv = wallError - wallPrev; 
            wallPrev = wallError;
            wallPIDValue = 2.1 * wallError + 0.02 * wallInt + 13.0 * wallDeriv;
        }

        encoderPID();

        // --- Adjusted Correction Weights ---
        const float encoderWeight = 0.27;
        const float wallWeight = 0.73;
        float totalCorrection = (encoderWeight * encoderPIDValue) + (wallWeight * wallPIDValue);

        // Tighter correction limits to avoid oscillations
        totalCorrection = constrain(totalCorrection, -32, 32);

        int leftSpeed = constrain(currentSpeed + totalCorrection, -255, 255);
        int rightSpeed = constrain(currentSpeed - totalCorrection, -255, 255);
        setMotorSpeeds(leftSpeed, rightSpeed);
    }

    stopMotors();
    delay(7); // Slightly longer settling for high speed
}

void turn(float degrees, int speed) {
    float distance_per_wheel = (abs(degrees) / 360.0) * (PI * WHEEL_SEPARATION_CM);
    long targetTicks = distance_per_wheel / CM_PER_TICK;
    
    leftEncoderCount.clearCount();
    rightEncoderCount.clearCount();
    
    if (degrees > 0) setMotorSpeeds(speed, -speed);
    else setMotorSpeeds(-speed, speed);
    
    while ((abs(leftEncoderCount.getCount()) + abs(rightEncoderCount.getCount())) / 2 < targetTicks) 
    {
        delay(5);
    }
    
    stopMotors();
}

void executeMoveTo(int nx, int ny) {
    Heading targetHeading = robotHeading;
    if (nx > robotX) targetHeading = EAST;
    else if (nx < robotX) targetHeading = WEST;
    else if (ny > robotY) targetHeading = NORTH;
    else if (ny < robotY) targetHeading = SOUTH;
    
    int delta = (targetHeading - robotHeading + 4) % 4;
    if (delta == 1) { // Turn Right
        turn(TURN_DEGREES_90, TURN_SPEED);
        stopMotors();
        robotHeading = (Heading)((robotHeading + 1) % 4);
    } else if (delta == 2) { // Turn Around
        turn(TURN_DEGREES_180, TURN_SPEED);
        stopMotors();
        robotHeading = (Heading)((robotHeading + 2) % 4);
    } else if (delta == 3) { // Turn Left
        turn(-TURN_DEGREES_90, TURN_SPEED);
        stopMotors();
        robotHeading = (Heading)((robotHeading + 3) % 4);
    }
    
    moveForwardOneCell();
    robotX = nx;
    robotY = ny;
    visited[robotY][robotX] = true;
}

void executeMoveToFast(int nx, int ny) {
    Heading targetHeading = robotHeading;
    if (nx > robotX) targetHeading = EAST;
    else if (nx < robotX) targetHeading = WEST;
    else if (ny > robotY) targetHeading = NORTH;
    else if (ny < robotY) targetHeading = SOUTH;

    int delta = (targetHeading - robotHeading + 4) % 4;
    if (delta == 1) { // Turn Right
        turn(FAST_TURN_DEGREES_90, FAST_TURN_SPEED);
        stopMotors();
        robotHeading = (Heading)((robotHeading + 1) % 4);
    } else if (delta == 2) { // Turn Around
        turn(FAST_TURN_DEGREES_180, FAST_TURN_SPEED);
        robotHeading = (Heading)((robotHeading + 2) % 4);
        stopMotors();
    } else if (delta == 3) { // Left
        turn(-FAST_TURN_DEGREES_90, FAST_TURN_SPEED);
        robotHeading = (Heading)((robotHeading + 3) % 4);
        stopMotors();
    }
    
    moveForwardOneCellFast();

    robotX = nx;
    robotY = ny;
}

// =================================================================
// ================= ALGORITHM & UTILITY FUNCTIONS =================
// =================================================================

//? void IRAM_ATTR isrLeftEncoder() { encoderCountLeft++; }
//? void IRAM_ATTR isrRightEncoder() { encoderCountRight++; }

bool inBounds(int x, int y) { 
    return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE; 
}

void computeFloodfill(int gx, int gy) {
    for (int y = 0; y < GRID_SIZE; y++)
        for (int x = 0; x < GRID_SIZE; x++) 
            distMap[y][x] = 10000;
    
    std::queue<Point> q;
    if (inBounds(gx, gy)) {
        distMap[gy][gx] = 0;
        q.push({gx, gy});
    }
    
    while (!q.empty()) {
        Point p = q.front(); 
        q.pop();
        int d = distMap[p.y][p.x];
        
        if (!(wallsMap[p.y][p.x] & WALL_N) && inBounds(p.x, p.y + 1) && distMap[p.y + 1][p.x] > d + 1) 
        { 
            distMap[p.y + 1][p.x] = d + 1; 
            q.push({p.x, p.y + 1}); 
        }
        if (!(wallsMap[p.y][p.x] & WALL_E) && inBounds(p.x + 1, p.y) && distMap[p.y][p.x + 1] > d + 1) 
        { 
            distMap[p.y][p.x + 1] = d + 1; 
            q.push({p.x + 1, p.y}); 
        }
        if (!(wallsMap[p.y][p.x] & WALL_S) && inBounds(p.x, p.y - 1) && distMap[p.y - 1][p.x] > d + 1) 
        { 
            distMap[p.y - 1][p.x] = d + 1; 
            q.push({p.x, p.y - 1}); 
        }
        if (!(wallsMap[p.y][p.x] & WALL_W) && inBounds(p.x - 1, p.y) && distMap[p.y][p.x - 1] > d + 1) 
        { 
            distMap[p.y][p.x - 1] = d + 1; 
            q.push({p.x - 1, p.y}); 
        }
    }
}

bool nextCellTowardsGoal(int &nx, int &ny) {
    int cx = robotX;
    int cy = robotY;
    int currentDist = distMap[cy][cx];

    if (currentDist == 10000) return false; // Nowhere to go

    // --- MODIFIED PART ---
    // Original Order: N, E, S, W
    // New Priority Order: E, N, W, S (to prioritize the right/East cell)
    int dx[] = {1, 0, -1, 0};         // E, N, W, S
    int dy[] = {0, 1, 0, -1};
    uint8_t wallMasks[] = {WALL_E, WALL_N, WALL_W, WALL_S};
    // --- END MODIFIED PART ---


    int bestDist = currentDist;
    int bestDirection = -1;

    // Check all 4 neighbors to find the one with the lowest distance
    for (int i = 0; i < 4; i++) {
        int nnx = cx + dx[i];
        int nny = cy + dy[i];

        if (inBounds(nnx, nny) && !(wallsMap[cy][cx] & wallMasks[i]) && distMap[nny][nnx] < bestDist) {
            bestDist = distMap[nny][nnx];
            bestDirection = i; // This will correspond to the new E, N, W, S order
        }
    }

    if (bestDirection != -1) {
        // We need to use the original dx/dy arrays to get the correct coordinates
        // since bestDirection is just an index (0 to 3) from our custom order.
        nx = cx + dx[bestDirection];
        ny = cy + dy[bestDirection];
        return true;
    }

    return false; // No valid path forward was found
}

void pathReturnToStart() {
    Serial.println("Pathfinding to Start...");
    
    while (!(robotX == START_X && robotY == START_Y)) {
        // SENSE FIRST - just like exploration does
        senseWallsAtCurrentCell();
        
        // COMPUTE PATH - with fresh sensor data
        computeFloodfill(START_X, START_Y);
        
        int nx, ny;
        if (nextCellTowardsGoal(nx, ny)) {
            // Path found, execute move (same as exploration)
            executeMoveTo(nx, ny);
        } else {
            // SAME ERROR HANDLING AS EXPLORATION
            Serial.println("Warning: Stuck on return path! Re-flooding and retrying...");
            computeFloodfill(START_X, START_Y);
            
            if(nextCellTowardsGoal(nx, ny)) {
                executeMoveTo(nx, ny);
            } else {
                // Last resort - turn around like exploration would
                Serial.println("Still stuck! Turning around and re-evaluating.");
                
                // Turn around 180 degrees
                turn(TURN_DEGREES_180, TURN_SPEED);
                robotHeading = (Heading)((robotHeading + 2) % 4);
                
                // Sense again after turning (like exploration)
                senseWallsAtCurrentCell();
                computeFloodfill(START_X, START_Y);
                
                // Try once more
                if (!nextCellTowardsGoal(nx, ny)) {
                    Serial.println("Error: Still stuck after turning around! Aborting.");
                    currentState = AWAITING_FAST_RUN;
                    return;
                } else {
                    executeMoveTo(nx, ny);
                }
            }
        }
    }
    
    Serial.println("Returned to start successfully!");
    
    // Final orientation (same as before)
    int delta = (START_HEADING - robotHeading + 4) % 4;
    if (delta == 1) turn(TURN_DEGREES_90, TURN_SPEED);
    else if (delta == 2) turn(TURN_DEGREES_180, TURN_SPEED);
    else if (delta == 3) turn(-TURN_DEGREES_90, TURN_SPEED);
    robotHeading = START_HEADING;
    currentState = AWAITING_FAST_RUN;
}

void runExploration() {
    senseWallsAtCurrentCell();
    computeFloodfill(goalCells[0].x, goalCells[0].y);
    
    bool isAtGoal = false;
    for (int i = 0; i < numGoalCells; i++) {
        if (robotX == goalCells[i].x && robotY == goalCells[i].y) {
            isAtGoal = true;
            break;
        }
    }
    
    if (isAtGoal) {
        stopMotors();
        Serial.println("Goal cell reached during exploration!");
        delay(1000);
        
        while(1){
            if(checkButton(BTNL, lastButton1Press)){
                digitalWrite(LEDR, HIGH);
                digitalWrite(LEDL, HIGH);
                delay(1000);
                digitalWrite(LEDR, LOW);
                digitalWrite(LEDL, LOW);
                currentState = AWAITING_FAST_RUN;
                break;
            }
        }
        return;
    }
    
    int nx, ny;
    if (nextCellTowardsGoal(nx, ny)) {
        executeMoveTo(nx, ny);
    } else {
        Serial.println("Still stuck! Returning to wall follow mode.");
        currentState = FOLLOW_LEFT;
    }
}

void performFastRun() {
    robotX = START_X;
    robotY = START_Y;
    robotHeading = START_HEADING;

    Serial.println("Executing fastest path to goal...");
    computeFloodfillVisited(goalCells[0].x, goalCells[0].y);

    while (!(robotX == goalCells[0].x && robotY == goalCells[0].y)) {
        int nx, ny;
        if (nextCellTowardsGoalFinal(nx, ny)) {
            executeMoveToFast(nx, ny);
        } else {
            Serial.println("Error: Stuck during fast run!");
            currentState = FOLLOW_LEFT;
            break; 
        }
    }
    
    Serial.println("Fast run complete!");
}

bool nextCellTowardsGoalFinal(int &nx, int &ny) {
    int cx = robotX;
    int cy = robotY;
    int currentDist = distMap[cy][cx];

    if (currentDist == 10000) return false; // Nowhere to go

    // --- MODIFIED PART ---
    // New Priority Order: E, N, W, S (to prioritize the right/East cell)
    int dx[] = {1, 0, -1, 0};        // E, N, W, S
    int dy[] = {0, 1, 0, -1};
    uint8_t wallMasks[] = {WALL_E, WALL_N, WALL_W, WALL_S};
    // --- END MODIFIED PART ---

    int bestDist = currentDist;
    int bestDirection = -1;

    // Check all 4 neighbors to find the one with the lowest distance
    for (int i = 0; i < 4; i++) {
        int nnx = cx + dx[i];
        int nny = cy + dy[i];

        // The logic for the final run, including the 'visited' check, remains the same.
        if (inBounds(nnx, nny) && !(wallsMap[cy][cx] & wallMasks[i]) &&
            distMap[nny][nnx] < bestDist && visited[nny][nnx]) {
            bestDist = distMap[nny][nnx];
            bestDirection = i;
        }
    }

    if (bestDirection != -1) {
        nx = cx + dx[bestDirection];
        ny = cy + dy[bestDirection];
        return true;
    }

    return false; // No valid, visited path forward was found
}

void computeFloodfillVisited(int gx, int gy) {
    // Reset the distance map
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            distMap[y][x] = 10000;
        }
    }

    std::queue<Point> q;

    // Only start the floodfill if the goal cell is visited
    if (inBounds(gx, gy) && visited[gy][gx]) {
        distMap[gy][gx] = 0;
        q.push({gx, gy});
    }

    while (!q.empty()) {
        Point p = q.front();
        q.pop();
        int d = distMap[p.y][p.x];

        // Check each neighbor and propagate if the path is valid AND the cell has been visited
        // North
        if (!(wallsMap[p.y][p.x] & WALL_N) && inBounds(p.x, p.y + 1) && distMap[p.y + 1][p.x] > d + 1 && visited[p.y + 1][p.x]) 
        {
            distMap[p.y + 1][p.x] = d + 1;
            q.push({p.x, p.y + 1});
        }
        // East
        if (!(wallsMap[p.y][p.x] & WALL_E) && inBounds(p.x + 1, p.y) && distMap[p.y][p.x + 1] > d + 1 && visited[p.y][p.x + 1]) 
        {
            distMap[p.y][p.x + 1] = d + 1;
            q.push({p.x + 1, p.y});
        }
        // South
        if (!(wallsMap[p.y][p.x] & WALL_S) && inBounds(p.x, p.y - 1) && distMap[p.y - 1][p.x] > d + 1 && visited[p.y - 1][p.x]) 
        {
            distMap[p.y - 1][p.x] = d + 1;
            q.push({p.x, p.y - 1});
        }
        // West
        if (!(wallsMap[p.y][p.x] & WALL_W) && inBounds(p.x - 1, p.y) && distMap[p.y][p.x - 1] > d + 1 && visited[p.y][p.x - 1]) 
        {
            distMap[p.y][p.x - 1] = d + 1;
            q.push({p.x - 1, p.y});
        }
    }
}