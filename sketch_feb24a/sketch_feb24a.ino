// DEVICE 2 (ESP32-WROOM-32D)
#define RX2 16
#define TX2 17

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2); // Using Serial2
  Serial.println("Device 2 (ESP32-WROOM-32D) started.");
}

void loop() {
  while (Serial2.available()) {
    String receivedData = Serial2.readStringUntil('\n');
    if (receivedData.length() > 0) {
      Serial.print("Received: ");
      Serial.println(receivedData);
    }
  }
}