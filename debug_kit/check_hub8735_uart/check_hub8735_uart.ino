#include <HardwareSerial.h>

#define RXD2 27  // ESP32 RX 接 HUB8735 TX (15)
#define TXD2 5  // ESP32 TX 接 HUB8735 RX (8)

HardwareSerial h87(1);

void setup() {
  Serial.begin(115200);
  h87.begin(115200, SERIAL_8N1, TXD2, RXD2);
  delay(1000);
}

void loop() {
  if (h87.available()) {
    String receivedData = h87.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(receivedData);
  }

  delay(10);
}
