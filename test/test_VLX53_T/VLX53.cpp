#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// 1. Hardware Pin Definitions
#define XSHUT_CENTER 27
#define XSHUT_LEFT   13
#define XSHUT_RIGHT  33

// Default ESP32 Hardware I2C Pins
#define I2C_SDA      21
#define I2C_SCL      22

// 2. Custom Unique I2C Addresses 
#define ADDRESS_CENTER 0x30
#define ADDRESS_LEFT   0x31
#define ADDRESS_RIGHT  0x32

// Create sensor objects
Adafruit_VL53L0X sensorCenter = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLeft   = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight  = Adafruit_VL53L0X();

// Function to reset and sequentially assign addresses
void initializeSensorBus() {
  // Set all XSHUT pins to output
  pinMode(XSHUT_CENTER, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  
  // Hard reset: Pull all XSHUT pins LOW to shut down all sensors
  digitalWrite(XSHUT_CENTER, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(20); // Give them time to completely shut off
  
  // --- Step A: Wake up and configure CENTER ---
  digitalWrite(XSHUT_CENTER, HIGH);
  delay(15);
  if (!sensorCenter.begin(ADDRESS_CENTER)) {
    Serial.println("FATAL ERROR: Center VL53L0X failed to initialize!");
    while(1) { delay(10); } // Lock execution if critical hardware is missing
  }
  
  // --- Step B: Wake up and configure LEFT ---
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(15);
  if (!sensorLeft.begin(ADDRESS_LEFT)) {
    Serial.println("FATAL ERROR: Left VL53L0X failed to initialize!");
    while(1) { delay(10); }
  }
  
  // --- Step C: Wake up and configure RIGHT ---
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(15);
  if (!sensorRight.begin(ADDRESS_RIGHT)) {
    Serial.println("FATAL ERROR: Right VL53L0X failed to initialize!");
    while(1) { delay(10); }
  }
}

void setup() {
  // Start Serial communication
  Serial.begin(115200);
  while (!Serial) { delay(1); } // Wait for serial monitor
  
  Serial.println("\n--- Starting Multi-ToF Sensor Initialization ---");
  
  // Force start the ESP32 hardware I2C peripheral
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Assign IDs and clear the default address (0x29) bottleneck
  initializeSensorBus();
  
  Serial.println("Success: All sensors initialized and assigned distinct I2C targets!\n");
}

void loop() {
  // Create data structures to hold measurements
  VL53L0X_RangingMeasurementData_t measureLeft;
  VL53L0X_RangingMeasurementData_t measureCenter;
  VL53L0X_RangingMeasurementData_t measureRight;
  
  // Sequential non-overlapping reads with recovery windows
  sensorLeft.rangingTest(&measureLeft, false);
  delay(10); // 10ms grace period prevents ESP32 Wire.cpp Error 263 bus lockups
  
  sensorCenter.rangingTest(&measureCenter, false);
  delay(10);
  
  sensorRight.rangingTest(&measureRight, false);
  delay(10);
  
  // --- Output Processing ---
  
  // Print Left Sensor Status
  if (measureLeft.RangeStatus != 4) { 
    Serial.print("Left: "); 
    Serial.print(measureLeft.RangeMilliMeter); 
    Serial.print(" mm\t|\t");                                  
  } else {
    Serial.print("Left: OOR \t|\t"); // Out of Range
  }
  
  // Print Center Sensor Status
  if (measureCenter.RangeStatus != 4) {
    Serial.print("Center: "); 
    Serial.print(measureCenter.RangeMilliMeter); 
    Serial.print(" mm\t|\t");
  } else {
    Serial.print("Center: OOR \t|\t");
  }
  
  // Print Right Sensor Status
  if (measureRight.RangeStatus != 4) {
    Serial.print("Right: "); 
    Serial.print(measureRight.RangeMilliMeter); 
    Serial.println(" mm");
  } else {
    Serial.println("Right: OOR ");
  }
  
  // Master pacing delay for loop stability
  delay(100);
}
