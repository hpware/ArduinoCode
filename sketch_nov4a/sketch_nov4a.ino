#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Int
Adafruit_SSD1306 display(128, 64, &Wire, -1);


// Menu
int Current_Menu = 0;
int Current_Choice = 1;
// Btn1
const int buttonPin1 = 2;
int button1State = 0;
// Btn2
const int buttonPin2 = 4;
int button2State = 0;

void setup() {
  Serial.begin(9600);
  initDisplay();
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  initMenu();
}

void loop() {
  btnload();
  btnif();
  debug();
}

// Start Popup Load
void initDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("XXXXX", 0, 0, &x1, &y1, &w, &h);
  int16_t x = (128 - w) / 2;
  int16_t y = (64 - h - 13) / 2;
  display.setCursor(x, y);
  display.println("CYIVS");
  display.setCursor(x + 5, y + 15);
  display.println("C2XX");
  display.display();
  delay(1000);
  display.clearDisplay();
}


// Start Menu
void initMenu() {
    display.clearDisplay();
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds("XXXX", 0, 0, &x1, &y1, &w, &h);
    int16_t x = (128 - w) / 2;
    display.setCursor(x, 0);
    display.println("Menu");
    display.setCursor(0, 15);
    display.println("TIME");
    display.setCursor(0, 30);
    display.println("BLE");
    display.setCursor(0, 45);
    display.println("EEPROM");
    display.setCursor(70, 45);
    display.println("LIGHT");
    display.display();
}

// Buttons 
void btnload() {
  btn1();
  btn2();
}

void btnif() {
  if (button1State = 1) {
    Serial.println(button1State);
    if (Current_Choice = 4) {
      Current_Choice = 1;
    } else {
        Current_Choice = Current_Choice + 1;
    }
  } else if (button2State = 1) {
    Serial.println(button2State);
    if (Current_Choice = 1) {
      Current_Choice = 4;
    } else {
        Current_Choice = Current_Choice - 1;
    }
  }
}

void btn1() {
  button1State = digitalRead(buttonPin1);
}

void btn2() {
  button2State = digitalRead(buttonPin2);
}

// Debug 

void debug() {
}