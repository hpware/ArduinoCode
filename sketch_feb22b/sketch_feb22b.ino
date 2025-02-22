int ledpin = 2;
int beeppin = 3;

void setup() { 
  initBeep();
  pinMode(ledpin, OUTPUT);
}

void initBeep() {
  pinMode(beeppin, OUTPUT);
  beep(50);
  beep(50);
  delay(1000);
}

void loop() {
  analogWrite(ledpin, 100);
  delay(500);
  analogWrite(ledpin, 0);
  delay(500);
}

void beep(unsigned char delayms) {
  analogWrite(beeppin, 20);
  delay(delayms);
  analogWrite(beeppin, 0);
  delay(delayms);
}