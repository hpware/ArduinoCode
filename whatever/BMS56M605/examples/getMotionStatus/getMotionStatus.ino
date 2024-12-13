/*****************************************************************
File:         getMotionStatus.ino
Description:  Device default select pin8 as intpin,and when no 
              interrupt occurs INT is Low level.The example shows
              Motion Detection.
******************************************************************/
#include "BMS56M605.h"
BMS56M605 Mpu(8);//default select pin8 as intpin
uint16_t cnt = 0;
  
void setup() {
  Mpu.begin();
  Serial.begin(9600);
  Mpu.setINT(MOTION_MODE,true);
  Mpu.setInterruptPinPolarity(ACTIVE_LOW);//when no interrupt occurs INT is Low level
  Mpu.setMotionThreshold(1);  //1mg
  Mpu.setMotionDuration(30);  //30ms
}

void loop() {
  if(Mpu.getINT() == 0)
  {
    cnt++;
    if(cnt >= 0xffff) cnt = 0;
    Serial.print("Motion Detected!  x");
    Serial.println(cnt);
    Serial.println();
  }
  

}
