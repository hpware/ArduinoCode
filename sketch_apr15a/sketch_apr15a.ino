#include <HardwareSerial.h>

HardwareSerial H87_Serial(2);
void setup() {
    Serial.begin(115200);
    H87_Serial.begin(115200, SERIAL_8N1, 26, 27); 
}

void loop() {
  if (H87_Serial.available()) {
    Serial.println(H87_Serial.readString());
  }
}
