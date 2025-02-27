// DEVICE 1 (ESP32 Super Mini)
#define RX1 0
#define TX1 1
      
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX1, TX1); // Correct pins already
  Serial.println("Device 1 (Super Mini) started.");
}

void loop() {
  String dataToSend = "Data_from_Device1:" + String(millis());
  Serial1.println(dataToSend);
  Serial.println("Sent: " + dataToSend);
  delay(1000);
}
