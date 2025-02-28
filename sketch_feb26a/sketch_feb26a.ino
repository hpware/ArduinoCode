// Device 2 (立原c
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
bool blinkState;


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);  
  Wire.begin(); 

  Serial.println("Initializing MPU...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (true);
  }
  Serial.println("MPU6050 connection successful");
  mpu.setXAccelOffset(0); 
  mpu.setYAccelOffset(0); 
  mpu.setZAccelOffset(0); 
  mpu.setXGyroOffset(0);  
  mpu.setYGyroOffset(0);  
  mpu.setZGyroOffset(0);  
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("Accel: "); Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t"); Serial.print(az); Serial.print("\t");
  Serial.print("Gyro: "); Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t"); Serial.println(gz);
  Serial.print(String(Serial2.available()));
  while (Serial2.available()) {
    String receivedData = Serial2.readStringUntil('\n');
    Serial.print(String(receivedData.length()) + "\n");
    if (receivedData.length() > 0 ) {
      Serial.print("Data: " + receivedData + "\n");
    }

    int separatorIndex = receivedData.indexOf(',');
    if (separatorIndex != -1) {
      String objectType = receivedData.substring(0, separatorIndex);
      String direction = receivedData.substring(separatorIndex + 1);

      Serial.print("Detected ");
      if (objectType == "1") Serial.print("1 - Person");
      else if (objectType == "2") Serial.print("2 - Bus");
      else if (objectType == "3") Serial.print("3 - Truck");
      else if (objectType == "4") Serial.print("4 - Car");
      else if (objectType == "5") Serial.print("5 - Motorbike");
      else if (objectType == "6") Serial.print("6 - Traffic Light");
      else if (objectType == "7") Serial.print("7 - Cat");
      else if (objectType == "8") Serial.print("8 - Dog");
      else Serial.print("Unknown object");
      Serial.println(". Direction: " + direction);
    }
  }

  blinkState = !blinkState;

  delay(200);  
}