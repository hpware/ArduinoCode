// DEVICE 1
#define RX1 0
#define TX1 1

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, RX1, TX1);
}
void loop() {
  Serial1.println("Net");
}