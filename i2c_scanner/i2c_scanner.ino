// OLED I2C Address Scanner

#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  while (!Serial) {
    // Wait for serial monitor to open
    delay(10);
  }
  
  Serial.println("\nOLED I2C Scanner");
  Serial.println("Scanning for OLED displays at addresses 0x3C and 0x3D...");
  
  // Check for OLED at address 0x3C
  Wire.beginTransmission(0x3C);
  byte error1 = Wire.endTransmission();
  
  // Check for OLED at address 0x3D
  Wire.beginTransmission(0x3D);
  byte error2 = Wire.endTransmission();
  
  if (error1 == 0) {
    Serial.println("OLED found at address 0x3C (most common)");
  }
  
  if (error2 == 0) {
    Serial.println("OLED found at address 0x3D");
  }
  
  if (error1 != 0 && error2 != 0) {
    Serial.println("No OLED display found at standard addresses.");
    Serial.println("Performing full I2C scan...");
    fullScan();
  }
}

void loop() {
  // Nothing to do here
}

void fullScan() {
  byte error, address;
  int devicesFound = 0;
  
  Serial.println("Scanning all I2C addresses...");
  
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println();
      devicesFound++;
    }
  }
  
  if (devicesFound == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.print("Found ");
    Serial.print(devicesFound);
    Serial.println(" device(s)");
  }
}