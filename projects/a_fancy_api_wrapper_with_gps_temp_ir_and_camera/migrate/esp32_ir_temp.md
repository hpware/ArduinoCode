# This is the original ir temp function
```cpp
float oti602AmbientTemp = 0.0;
float oti602ObjectTemp = 0.0;
float prevOti602JbjectTemp = NAN;
const float tempChangeThreshold = 0.5;
void ReadOTI602Temp() {
  if (readOTI602Temperatures(&oti602AmbientTemp, &oti602ObjectTemp)) {
    Serial.print("OTI602 Sensor -> Ambient: ");
    Serial.print(oti602AmbientTemp, 2);
    Serial.print(" *C, Object: ");
    Serial.print(oti602ObjectTemp, 2);
    Serial.println(" *C");
    if (!isnan(prevOti602JbjectTemp)) {
      float tempDelta = abs(oti602ObjectTemp - prevOti602JbjectTemp);
      Serial.println(tempDelta);
      if (tempDelta > tempChangeThreshold) {
        Serial.println("!!!!!!!!!!!!!!!");
        H87_Serial.println("true");
      } else { 
        H87_Serial.println("false");
      }
    }
    prevOti602JbjectTemp = oti602ObjectTemp;
  } else {
    Serial.println("Failed to read from OTI602 sensor!");
    // Optionally set temps to NaN or a specific error value
    // 如果沒有把 OTI602 設成 NAN
    oti602AmbientTemp = NAN;
    oti602ObjectTemp = NAN;
    prevOti602JbjectTemp = NAN;
  }
}

bool readOTI602Temperatures(float *ambientTemp, float *objectTemp) {
  byte data[6];
  
  // 步驟1和2: 發送寫地址和讀取指令
  Wire.beginTransmission(OTI602_ADDR);
  Wire.write(0x80);  // 讀取指令
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print("發送讀取指令錯誤: ");
    Serial.println(error);
    return false;
  }
  
  // 步驟3和4: 讀取數據
  // Arduino的Wire庫在requestFrom中自動處理重複開始條件和讀取位址
  byte bytesReceived = Wire.requestFrom(OTI602_ADDR, 6);
  
  Serial.print("接收到的數據量: ");
  Serial.println(bytesReceived);
  
  if (bytesReceived != 6) {
    return false;
  }
  
  // 讀取6個位元組
  for (int i = 0; i < 6; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
      Serial.print("數據[");
      Serial.print(i);
      Serial.print("]: 0x");
      Serial.println(data[i], HEX);
    } else {
      Serial.println("讀取數據時發生錯誤");
      return false;
    }
  }
  
  // 計算環境溫度 (前3個位元組)
  int32_t rawAmbient = data[0] + (data[1] << 8) + (data[2] << 16);
  if (data[2] >= 0x80) {
    rawAmbient -= 0x1000000;  // 處理負溫度
  }
  *ambientTemp = rawAmbient / 200.0f;
  
  // 計算物體溫度 (後3個位元組)
  int32_t rawObject = data[3] + (data[4] << 8) + (data[5] << 16);
  if (data[5] >= 0x80) {
    rawObject -= 0x1000000;  // 處理負溫度
  }
  *objectTemp = rawObject / 200.0f;
  
  return true;
}

```