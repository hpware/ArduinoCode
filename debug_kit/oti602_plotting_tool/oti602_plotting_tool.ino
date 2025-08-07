#include <Wire.h>

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define OTI602_ADDR 0x10
const bool debug = false;
float oti602AmbientTemp = 0.0;
float oti602ObjectTemp = 0.0;
const float tempChangeThreshold = 0.5;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
}

void loop() {
  if (readOTI602Temperatures(&oti602AmbientTemp, &oti602ObjectTemp)) {
    Serial.print("Ambient:");
    Serial.print(oti602AmbientTemp, 2);
    Serial.print(",");
    Serial.print("Object:");
    Serial.print(oti602ObjectTemp, 2);
    Serial.println("");
    delay(300);
  }
}

bool readOTI602Temperatures(float *ambientTemp, float *objectTemp) {
  byte data[6];
  Wire.beginTransmission(OTI602_ADDR);
  Wire.write(0x80);
  byte error = Wire.endTransmission();

  if (error != 0) {
    if (debug) {
      Serial.print("發送讀取指令錯誤: ");
      Serial.println(error);
    }
    return false;
  }

  byte bytesReceived = Wire.requestFrom(OTI602_ADDR, 6);

  if (debug) {
    Serial.print("接收到的數據量: ");
    Serial.println(bytesReceived);
  }

  if (bytesReceived != 6) {
    return false;
  }

  for (int i = 0; i < 6; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
      if (debug) {
        Serial.print("數據[");
        Serial.print(i);
        Serial.print("]: 0x");
        Serial.println(data[i], HEX);
      }
    } else {
      if (debug) {
        Serial.println("讀取數據時發生錯誤");
      }
      return false;
    }
  }

  int32_t rawAmbient = data[0] + (data[1] << 8) + (data[2] << 16);
  if (data[2] >= 0x80) {
    rawAmbient -= 0x1000000;
  }
  *ambientTemp = rawAmbient / 200.0f;

  int32_t rawObject = data[3] + (data[4] << 8) + (data[5] << 16);
  if (data[5] >= 0x80) {
    rawObject -= 0x1000000;
  }
  *objectTemp = rawObject / 200.0f;

  return true;
}