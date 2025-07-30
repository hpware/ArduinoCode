#include <HardwareSerial.h>

// 設定紅外線 PIN 與 地址
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define OTI602_ADDR 0x10

// 下方資料不要改!!!!
// 資料
String h87data = "";

// INIT Hardware
HardwareSerial H87_Serial(2); // 8735 連接

// Setup
void setup() {
  Serial.begin(115200);
  H87_Serial.begin(115200, SERIAL_8N1, 26, 27);
  Serial.println("Setup");
}

// Keep loop empty. And do not use it to do anything, as it will go wrong.
void loop() {
  // Read Hub 8735 serial data
  if (H87_Serial.available()) {
    Serial.println("Reading HUB 8735 Data!");
    String data = H87_Serial.readStringUntil('\n');
    Serial.print("Hub 8735 data: ");
    h87data = data;
    Serial.println(h87data);
  }
  delay(100);
}