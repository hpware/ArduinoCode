int ledpin = 2;
int ledpin2 = 15;
int beeppin = 3;

void setup() { 
  pinMode(ledpin, OUTPUT);
  pinMode(ledpin2, OUTPUT);
  pinMode(beeppin, OUTPUT);
  beep(50); 
  beep(50);
  delay(1000);
}


void loop() {
  led(ledpin2);
}

void beep(unsigned char delayms) {
  analogWrite(beeppin, 20);
  delay(delayms);
  analogWrite(beeppin, 0);
  delay(delayms);
}

void led(unsigned char pin) {
  analogWrite(pin, 100);
  delay(500);
  analogWrite(pin, 0);
  delay(500);
}  