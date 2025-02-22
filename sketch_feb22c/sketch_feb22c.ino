const int beeppin = 3;
const int balsepin = 9;

void setup() {
  pinMode(balsepin, INPUT);
  Serial.begin(115200);
  pinMode(beeppin, OUTPUT);
  beep(50); 
  beep(50);
  delay(1000);
}

void loop() {
  int sensorState = digitalRead(balsepin);
  Serial.println(sensorState);
  delay(100);
}
void beep(unsigned char delayms) {
  analogWrite(beeppin, 20);
  delay(delayms);
  analogWrite(beeppin, 0);
  delay(delayms);
}
