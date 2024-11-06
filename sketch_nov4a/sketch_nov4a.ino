#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Int
Adafruit_SSD1306 display(128, 64, &Wire, -1);


// Menu
int Current_Menu = 1;
int Current_Choice = 1;
// Btn1
const int buttonPin1 = 2;
int button1State = 0;

void setup() {
  Serial.begin(9600);
  initDisplay();
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
 pinMode(buttonPin1, INPUT);
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
  Current_Menu = 1;
}

// Buttons 
void btnload() {
  btn1();
}

void btnif() {
  if (button1State == 1) {
    if (Current_Choice == 4) {
      Current_Choice = 1;
    } else {
        Current_Choice++;
    }
    MenuDash();
  } else {

  }
}

void btn1() {
  button1State = digitalRead(buttonPin1);
  delay(50);
}

// Debug 

void debug() {

}

String getmsg() {
  while(!Serial.available());
  return Serial.readString();
}

// Menu Dash
void MenuDash() {
  display.clearDisplay();
  if (Current_Menu == 1) {
    initMenu(); 
  } else if (Current_Menu == 2) {
    timeMenu();
  } else if (Current_Menu == 3) {
    BLEMenu();
  } else if (Current_Menu == 4) {
    EEPROMMenu();
  } else if (Current_Menu == 5) {
    LightMenu();
  }
}

// Menu

// Start Menu
void initMenu() {
    display.clearDisplay();
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds("XXXX", 0, 0, &x1, &y1, &w, &h);
    int16_t x = (128 - w) / 2;
    display.setCursor(x, 0);
    display.println("Menu");
    if (Current_Choice == 1) {
    display.setCursor(0, 15);
    display.println("> TIME");
    display.setCursor(0, 30);
    display.println("BLE");
    display.setCursor(0, 45);
    display.println("EEPROM");
    display.setCursor(70, 45);
    display.println("LIGHT");
    } else if (Current_Choice == 2) {
    display.setCursor(0, 15);
    display.println("TIME");
    display.setCursor(0, 30);
    display.println("> BLE");
    display.setCursor(0, 45);
    display.println("EEPROM");
    display.setCursor(70, 45);
    display.println("LIGHT");
    } else if (Current_Choice == 3) {
    display.setCursor(0, 15);
    display.println("TIME");
    display.setCursor(0, 30);
    display.println("BLE");
    display.setCursor(0, 45);
    display.println("> EEPROM");
    display.setCursor(70, 45);
    display.println("LIGHT");
    } else if (Current_Choice == 4) {
    display.setCursor(0, 15);
    display.println("TIME");
    display.setCursor(0, 30);
    display.println("BLE");
    display.setCursor(0, 45);
    display.println("EEPROM");
    display.setCursor(70, 45);
    display.println("> LIGHT");
    }
    display.display();
}

// Time Menu
void timeMenu() {
  display.clearDisplay();
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("XXXX", 0, 0, &x1, &y1, &w, &h);
  int16_t x = (128 - w) / 2;
  display.setCursor(x, 0);
  display.println("Time");
  if (Current_Choice == 1) {
    display.setCursor(0, 15);
    display.println("> Set Time");
    display.setCursor(0, 30);
    display.println("Get Time");
    display.setCursor(0, 45);
    display.println("Set Time");
    display.setCursor(70, 45);
    display.println("Set Time");
  }
  display.display();
}

// BLE Menu
void BLEMenu() {
  display.clearDisplay();
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("XXXX", 0, 0, &x1, &y1, &w, &h);
  int16_t x = (128 - w) / 2;
  display.setCursor(x, 0);
  display.println("BLE");
  if (Current_Choice = 1) {
    display.setCursor(0, 15);
    display.println("> Scan");
    display.setCursor(0, 30);
    display.println("Connect");
    display.setCursor(0, 45);
    display.println("Disconnect");
    display.setCursor(70, 45);
    display.println("Scan");
  }
  display.display();
}

// EEPROM Menu
void EEPROMMenu() {
  display.clearDisplay();
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("XXXX", 0, 0, &x1, &y1, &w, &h);
  int16_t x = (128 - w) / 2;
  display.setCursor(x, 0);
  display.println("EEPROM");
  if (Current_Choice = 1) {
    display.setCursor(0, 15);
    display.println("> Read");
    display.setCursor(0, 30);
    display.println("Write");
    display.setCursor(0, 45);
    display.println("Erase");
    display.setCursor(70, 45);
    display.println("Read");
  }
  display.display();
}

// Light Menu
void LightMenu() {
  display.clearDisplay();
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("XXXX", 0, 0, &x1, &y1, &w, &h);
  int16_t x = (128 - w) / 2;
  display.setCursor(x, 0);
  display.println("Light");
  if (Current_Choice = 1) {
    display.setCursor(0, 15);
    display.println("> On");
    display.setCursor(0, 30);
    display.println("Off");
    display.setCursor(0, 45);
    display.println("Toggle");
    display.setCursor(70, 45);
    display.println("On");
  }
  display.display();
}