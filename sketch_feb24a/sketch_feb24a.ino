
// DEVICE 2 (ESP32-WROOM-32D)
#define RX2 16
#define TX2 17

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RX2, TX2); // Using Serial2
  Serial.println("Device 2 (ESP32-WROOM-32D) started.");
}

void loop() {
  if (Serial2.available()) { // Using Serial2
    String receivedData = Serial2.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(receivedData);
  }
}