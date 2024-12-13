/*****************************************************************
File:         readAccelerationAndGyroscopeAndTemperature.ino
Description:  repeatedly obtain the 3 axis Acceleration(unit: g),
              3 axis Gyroscope(unit: °/s) and Temperature(unit: ℃) 
              through IIC and display the value in the serial port.
******************************************************************/
#include "BMS56M605.h"
BMS56M605 Mpu(8);//default select pin8 as intpin

void setup() {
  Mpu.begin();
  Serial.begin(9600);
}

void loop() {
  Mpu.getEvent();
  Serial.print("Temp = ");
  Serial.print(Mpu.temperature);
  Serial.println(" ℃");
  Serial.print("ax = ")  ;
  Serial.print(Mpu.accX);
  Serial.print("  ay = ");
  Serial.print(Mpu.accY);
  Serial.print("  az = ");
  Serial.print(Mpu.accZ);
  Serial.println("   g");
  Serial.print("gx = ");
  Serial.print(Mpu.gyroX);
  Serial.print("  gy = ");
  Serial.print(Mpu.gyroY);
  Serial.print("  gz = ");
  Serial.print(Mpu.gyroZ);
  Serial.println(" °/s");
  Serial.println();
  delay(1000);

}
