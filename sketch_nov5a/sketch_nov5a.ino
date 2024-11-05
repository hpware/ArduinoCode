#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Int
Adafruit_SSD1306 display(128, 64, &Wire, -1);


/*
  Button

  Turns on and off a light emitting diode(LED) connected to digital pin 13,
  when pressing a pushbutton attached to pin 2.

  The circuit:
  - LED attached from pin 13 to ground through 220 ohm resistor
  - pushbutton attached to pin 2 from +5V
  - 10K resistor attached to pin 2 from ground

  - Note: on most Arduinos there is already an LED on the board
    attached to pin 13.

  created 2005
  by DojoDave <http://www.0j0.org>
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Button
*/

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;  // the number of the pushbutton pin

// variables will change:
int buttonState = 0;  // variable for reading the pushbutton status
int Count = 0;

void setup() {
  // initialize the LED pin as an output:
    // initialize the pushbutton pin as an input:
    Serial.begin(9600);
  pinMode(buttonPin, INPUT);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
  display.setTextSize(1);
    display.setTextColor(WHITE);
  display.setCursor(0, 0);
}

void loop() {
  display.clearDisplay();
    display.setCursor(0, 0);
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    display.println("Button Clicked");
    Serial.println("Button Clicked");
    } else {
    // turn LED off:
    Serial.println("Button UnClicked");
    display.println("Button Unclicked");
    }
    display.display();
    }