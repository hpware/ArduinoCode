#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Int
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// Menu
int Current_Menu = 1;
int Current_Choice = 1;
int Forward_Menu = 0;
int initMenu_Choice = 1;
// Btn1
const int buttonPin1 = 2;
int NextButtonState = 0;
//Button2
const int buttonPin2 = 4;
int BackButtonState = 0;
// Btn3
const int buttonPin3 = 7;
int UnNextButtonState = 0;

// Btn4
const int buttonPin4 = 3;
int GoButtonState = 0;
