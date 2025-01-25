#include <WiFi.h>

const char* ssid = "hel";
const char* password = "1234567890";
WifiServer server(80);

String header;
void setup() {
  Wifi.begin(ssid, password);
  server.begin();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
    Serial.println(WiFi.localIP());
}

void loop() {
  // put your main code here, to run repeatedly:

}
