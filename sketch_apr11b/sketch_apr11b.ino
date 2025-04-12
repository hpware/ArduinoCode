#include <HardwareSerial.h>

#define RXD2 27  // ESP32 RX 接 HUB8735 TX
#define TXD2 26  // ESP32 TX 接 HUB8735 RX

HardwareSerial mySerial(2); // 使用 Serial2 接 HUB8735

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, TXD2, RXD2);
  delay(1000);

  Serial.println("準備接收來自 HUB8735 的 UART 資料...");
}

void loop() {
  if (mySerial.available()) {
    String receivedData = mySerial.readStringUntil('\n');  // 一次讀一筆直到換行
    Serial.print("來自 HUB8735: ");
    Serial.println(receivedData);
  }

  delay(10); // 小延遲，避免 CPU 過載
}
