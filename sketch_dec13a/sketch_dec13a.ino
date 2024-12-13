#include "MPU.h"
void setup() {
  MPU_init(I2C_TypeDef *I2Cx, uint8_t mpu_address);
  Serial.begin(9600);
}

void loop() {
  Serial.println(MPU_ACCEL_X_REG);
  Serial.println(MPU_ACCEL_Y_REG);
  Serial.println(MPU_ACCEL_Z_REG);
}
