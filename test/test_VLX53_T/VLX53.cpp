#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Define XSHUT pins according to your setup
#define XSHUT_CENTER 27
#define XSHUT_LEFT   13
#define XSHUT_RIGHT  33

// Define new I2C Addresses for the sensors 
// (The default is 0x29, so we assign them new ones)
#define ADDRESS_CENTER 0x30
#define ADDRESS_LEFT   0x31
#define ADDRESS_RIGHT  0x32

// Create sensor objects
Adafruit_VL53L0X sensorCenter = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLeft   = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight  = Adafruit_VL53L0X();

void setID() {
  // 1. Set all XSHUT pins to output
  pinMode(XSHUT_CENTER, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  
  // 2. Pull all XSHUT pins LOW to reset and turn off all sensors
  digitalWrite(XSHUT_CENTER, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);
  
  // 3. Wake up Center sensor and change its address
  digitalWrite(XSHUT_CENTER, HIGH);
  delay(10);
  if (!sensorCenter.begin(ADDRESS_CENTER)) {
    Serial.println("Failed to boot Center VL53L0X!");
    while(1); // Freeze if it fails
  }
  
  // // 4. Wake up Left sensor and change its   
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  if (!sensorLeft.begin(ADDRESS_LEFT)) {
    Serial.println("Failed to boot Left VL53L0X!");
    while(1);
  }
  
  // 5. Wake up Right sensor and change its address
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  if (!sensorRight.begin(ADDRESS_RIGHT)) {
    Serial.println("Failed to boot Right VL53L0X!");
    while(1);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Wait until serial port opens 
  while (!Serial) { delay(1); }
  
  Serial.println("Starting multi-VL53L0X initialization...");
  
  // Call the function to assign unique addresses
  setID();
  
  Serial.println("All sensors initialized successfully!\n");
}

void loop() {
  // Create data structures to hold the measurements
  VL53L0X_RangingMeasurementData_t measureCenter;
  VL53L0X_RangingMeasurementData_t measureLeft;
  VL53L0X_RangingMeasurementData_t measureRight;
  
  // Read the sensors (passing 'false' to disable debug printouts)
  sensorLeft.rangingTest(&measureLeft, false);
  sensorCenter.rangingTest(&measureCenter, false);
  sensorRight.rangingTest(&measureRight, false);
  
  // --- Print Left Sensor ---
  if (measureLeft.RangeStatus != 4) {  // Status 4 means Out of Range (OOR)
    Serial.print("Left: "); 
    Serial.print(measureLeft.RangeMilliMeter); 
    Serial.print(" mm\t|\t");                                  
  } else {
    Serial.print("Left: OOR \t|\t");
  }
  
  // --- Print Center Sensor ---
  if (measureCenter.RangeStatus != 4) {
    Serial.print("Center: "); 
    Serial.print(measureCenter.RangeMilliMeter); 
    Serial.print(" mm\t|\t");
  } else {
    Serial.print("Center: OOR \t|\t");
  }
  
  // --- Print Right Sensor ---
  if (measureRight.RangeStatus != 4) {
    Serial.print("Right: "); 
    Serial.print(measureRight.RangeMilliMeter); 
    Serial.println(" mm");
  } else {
    Serial.println("Right: OOR ");
  }
  
  // Small delay before the next read
  delay(100);
}