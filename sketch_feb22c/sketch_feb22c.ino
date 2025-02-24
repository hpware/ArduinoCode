const int beeppin = 0;
const int balsepin = 1;
const int balsepind = 2;

void setup() {
  Serial.begin(460800);
  pinMode(beeppin, OUTPUT);
  pinMode(balsepin, INPUT);
  pinMode(balsepind, INPUT);
  beep(50); 
  beep(50);
  delay(1000);
}

void loop() {
  int sensorState = analogRead(balsepin);
  int sensorStated = digitalRead(balsepind);
  Serial.print("Analog: ");
  Serial.print(sensorState);
  Serial.print("  Digital: ");
  Serial.println(sensorStated);
  if (sensorStated) {
    beep(100);
  }
}
void beep(unsigned char delayms) {
  analogWrite(beeppin, 20);
  delay(delayms);
  analogWrite(beeppin, 0);
  delay(delayms);
}
