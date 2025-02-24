// DEVICE 1 (ESP32 Super Mini)
#define RX1 0
#define TX1 1
      
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, RX1, TX1); // Correct pins already
  Serial.println("Device 1 (Super Mini) started.");
}

void loop() {
  Serial1.println("Net");
  Serial.println("data");
  delay(1000);
}
