#include <ESP.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(ESP.getFlashChipSize());
}

void loop() {
  // Do Nothing Here
}