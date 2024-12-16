#include <BMS56M605.h>
#include "values.h"

BMS56M605 mpu(8);

void setup() {
  mpu.begin();
  Serial.begin(9600);
}

void loop() {
  event();
}


void event() {
    mpu.getEvent();
    gx = mpu.gyroX;
    gy = mpu.gyroY;
    gz = mpu.gyroZ;
    //if (gx < -100 || gy < -100 || gz < -100) {
    //    Serial.println("You got hit by a car!");
    //}
    if (gx < -100 && gy < -100) {
        Serial.println("Oops! You just fell down on the floor! -x & -y");
    }
    if (gx > 100 && gy > 100) {
        Serial.println("Oops! You just fell down on the floor! x & y");
    }
    if (gx < -100 && gz < -100) {
        Serial.println("Oops! You just fell down on the floor! -x & -z");
    }
    if (gx > 100 && gz > 100) {
        Serial.println("Oops! You just fell down on the floor! x & z");
    }
    if (gz < -100 && gy < -100) {
        Serial.println("Oops! You just fell down on the floor! -y & -z");
    }
    if (gy > 100 && gz > 100) {
        Serial.println("Oops! You just fell down on the floor! y & z");
    }
    //if (gy < -100) {
    //    Serial.println("Oops! You just fell down on the floor! y");
    //}
    //if (gz < -100) {
    //    Serial.println("Oops! You just fell down on the floor! z");
    //}
    delay(100);
}