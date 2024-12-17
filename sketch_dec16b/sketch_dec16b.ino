#include <BMS56M605.h>
#include "values.h"

BMS56M605 mpu(8);


void setup() {
  mpu.begin();
  Serial.begin(9600);
  Serial.println("a \n");
  pinMode(LED_BUILTIN, OUTPUT);

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
        btn1();
    }
    if (gx < -100 && gz < -100) {
        Serial.println("Oops! You just fell down on the floor! -x & -z");
        btn1();

    }
    if (gx > 100 && gz > 100) {
        Serial.println("Oops! You just fell down on the floor! x & z");
        btn1();
    }
    if (gz < -100 && gy < -100) {
        Serial.println("Oops! You just fell down on the floor! -y & -z");
        btn1();
    }
    if (gy > 100 && gz > 100) {
        Serial.println("Oops! You just fell down on the floor! y & z");
        btn1();
    }
    //if (gy < -100) {
    //    Serial.println("Oops! You just fell down on the floor! y");
    //}
    //if (gz < -100) {
    //    Serial.println("Oops! You just fell down on the floor! z");
    //}
    delay(100);
}
void btn1() {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(10);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(10);  
}