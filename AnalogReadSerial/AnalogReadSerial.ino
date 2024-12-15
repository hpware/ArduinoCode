/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogReadSerial
*/

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A4);
  // print out the value you read:
  Serial.println("a4: ");
  Serial.print(sensorValue);
  Serial.println();
  // read the input on analog pin 0:
  int sensorValue2 = analogRead(A5);
  // print out the value you read:
  Serial.println("a5: ");
  Serial.print(sensorValue2);
  delay(1000);  // delay in between reads for stability
}
