// DEVICE 2
#define RX1 0
#define TX1 2

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, RX1, TX1);
}
void loop() {
  if (Serial1.available()) {
    String receivedData = Serial1.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(receivedData);
  }
}