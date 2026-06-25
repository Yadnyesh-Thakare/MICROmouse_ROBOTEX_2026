#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#define ENCL_A 19
#define ENCL_B 23
#define ENCR_A 2
#define ENCR_B 15

#define XSHUT_CENTER 27
#define XSHUT_LEFT   13
#define XSHUT_RIGHT  33

#define ADDRESS_CENTER 0x30
#define ADDRESS_LEFT   0x31
#define ADDRESS_RIGHT  0x32

const int WallLostThreshold = 125;
const int obstacleThreshold  = 95;

int leftDist_mm = 1000, centerDist_mm = 1000, rightDist_mm = 1000;
volatile long leftEncoderCount = 0, rightEncoderCount = 0;

VL53L0X sensorLeft, sensorCenter, sensorRight;

void IRAM_ATTR leftEncoderISR()  { leftEncoderCount  += (digitalRead(ENCL_B) == LOW) ? 1 : -1; }
void IRAM_ATTR rightEncoderISR() { rightEncoderCount += (digitalRead(ENCR_B) == LOW) ? 1 : -1; }

void initSensors() {
  pinMode(XSHUT_CENTER, OUTPUT); pinMode(XSHUT_LEFT, OUTPUT); pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_CENTER, LOW); digitalWrite(XSHUT_LEFT, LOW); digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  auto initIndiv = [](VL53L0X& s, int pin, uint8_t addr) {
    digitalWrite(pin, HIGH); delay(10);
    s.setAddress(addr);
    if (!s.init()) { Serial.printf("Sensor %X failed!\n", addr); while(1); }
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

int wallCase() {
  int mask = ((leftDist_mm > WallLostThreshold) << 2) | ((centerDist_mm > obstacleThreshold) << 1) | (rightDist_mm > WallLostThreshold);
  const int caseMap[8] = { 7, 3, 2, 5, 1, 6, 4, 0 };
  return caseMap[mask];
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  
  pinMode(ENCL_A, INPUT_PULLUP); pinMode(ENCL_B, INPUT_PULLUP);
  pinMode(ENCR_A, INPUT_PULLUP); pinMode(ENCR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCL_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCR_A), rightEncoderISR, RISING);
  
  initSensors();
}

void loop() {
  // Grab fresh background cache data instantly
  readSensors();
  int currentCase = wallCase();

  // Print telemetry updates every 100 milliseconds
  static unsigned long lastPrintTime = 0;
  unsigned long currentMillis = millis();

  
  if (currentMillis - lastPrintTime >= 100) { 
    lastPrintTime = currentMillis;
    Serial.printf("L_Enc: %ld | R_Enc: %ld || Dist L:%d C:%d R:%d -> CASE: %d\n", 
                  leftEncoderCount, rightEncoderCount, leftDist_mm, centerDist_mm, rightDist_mm, currentCase);
  }

  // Small delay to let the core process OS level background tasks cleanly
  delay(1); 
}