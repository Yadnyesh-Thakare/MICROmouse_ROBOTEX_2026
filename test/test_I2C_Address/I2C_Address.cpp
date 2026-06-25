#include <Arduino.h>
#include <Wire.h>

void setup() {
  // Set the baud rate to match the Serial Monitor
  Serial.begin(115200);
  while (!Serial); // Wait for the serial monitor to open

  Serial.println("\nI2C Scanner initialized");
  
  // Initialize the I2C bus (Defaults to SDA=21, SCL=22 on most ESP32s)
  Wire.begin(); 
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

  // Loop through all possible I2C addresses (1 to 127)
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of the Write.endTransmission 
    // to see if a device acknowledged the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }

  // Wait 5 seconds before scanning again
  delay(5000); 
}