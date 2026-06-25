#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDR 0x68

float gyroBiasZ = 0.0;  // stored in dps
float yaw = 0.0;
unsigned long lastUpdate = 0;


void wakeMPU() {
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

void calibrateGyro() {
  Serial.println("Keep robot completely still...");
  delay(1000);

  float sum = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, 2, true);
    sum += (int16_t)(Wire.read() << 8 | Wire.read()) / 65.5;
    delay(10);
  }

  gyroBiasZ = sum / 200.0;
  Serial.print("Gyro Bias Z = ");
  Serial.println(gyroBiasZ);
}

void updateYAW() {
  if (micros() - lastUpdate >= 1000) {

    float dt = (micros() - lastUpdate) / 1000000.0;
    lastUpdate = micros();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);

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

void setup() {
  Serial.begin(115200);
  Wire.begin();               // change to Wire.begin(SDA, SCL) if needed
  Wire.setClock(400000);

  wakeMPU();
  calibrateGyro();

  yaw = 0;
  lastUpdate = micros();
  Serial.println("Yaw tracking started");
}

void loop() {
  updateYAW();
}
