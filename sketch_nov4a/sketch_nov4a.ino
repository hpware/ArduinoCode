#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "values.h"


void setup() {
  Serial.begin(9600);
  Serial.println("Setting Up...");
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
}

// Start Popup Load
void initDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
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
  btn2();
  btn3();
  btn4();
  delay(100);
}

void btnif() {
  if (NextButtonState == 1) {
    if (Current_Choice == 4 && Current_Menu == 1 || Current_Choice == 6 && Current_Menu == 5 || Current_Choice == 3 && Current_Menu == 3 || Current_Choice == 4 && Current_Menu == 2) {
      
    } else {
        Current_Choice++;
    }
    MenuDash();
  } if (BackButtonState == 1) {
    if (Current_Menu == 1) {
      initMenu_Choice = Current_Choice;
    }
    if (Current_Menu != 1) {
    Current_Choice = 1;
        if (Current_Menu < 10) {
            Current_Menu = 1;
        } else if (20 > Current_Menu > 10) {
            Current_Menu = 2;
        }
        MenuDash();
    }
  } if (UnNextButtonState == 1) {
    if (Current_Choice == 1) {
    } else {
        Current_Choice--;
    }
    MenuDash();
  } if (GoButtonState == 1) {
      Current_Choice = 1;
      // Init Menu
    if (Forward_Menu == 1) {
      Current_Menu = 2;
    } else if (Forward_Menu == 2) {
      Current_Menu = 3;
    } else if (Forward_Menu == 3) {
      Current_Menu = 4;
    } else if (Forward_Menu == 4) {
      Current_Menu = 5;
    } // TimeMenu
    else if (Forward_Menu == 11) {
      Current_Menu = 11;
    } else if (Forward_Menu == 12) {
      Current_Menu = 12;
    } else if (Forward_Menu == 13) {
      Current_Menu = 13;
    } else if (Forward_Menu == 14) {
      Current_Menu = 14;
    }
    MenuDash();
  }
}

void btn1() {
  NextButtonState = digitalRead(buttonPin1);
}

void btn2() {
  BackButtonState = digitalRead(buttonPin2);
}

void btn3() {
  UnNextButtonState = digitalRead(buttonPin3);
}

void btn4() {
  GoButtonState = digitalRead(buttonPin4);
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
  } // Time SubMenu 
  else if (Current_Menu == 11) {
    timemenu11();
  } else if (Current_Menu == 12) {
    timemenu12();
  } else if (Current_Menu == 13) {
    timemenu13();
  } else if (Current_Menu == 14) {
    timemenu14();
  }
}

// Menus

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
    Forward_Menu = 1;
    display.setCursor(0, 15);
    display.println("> TIME");
    display.setCursor(0, 30);
    display.println("BLE");
    display.setCursor(0, 45);
    display.println("EEPROM");
    display.setCursor(70, 45);
    display.println("LIGHT");
    } else if (Current_Choice == 2) {
    Forward_Menu = 2;
    display.setCursor(0, 15);
    display.println("TIME");
    display.setCursor(0, 30);
    display.println("> BLE");
    display.setCursor(0, 45);
    display.println("EEPROM");
    display.setCursor(70, 45);
    display.println("LIGHT");
    } else if (Current_Choice == 3) {
    Forward_Menu = 3;
    display.setCursor(0, 15);
    display.println("TIME");
    display.setCursor(0, 30);
    display.println("BLE");
    display.setCursor(0, 45);
    display.println("> EEPROM");
    display.setCursor(70, 45);
    display.println("LIGHT");
    } else if (Current_Choice == 4) {
    Forward_Menu = 4;
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
    Forward_Menu = 11;
    display.setCursor(0, 15);
    display.println("> Set Time");
    display.setCursor(0, 30);
    display.println("Get Time");
    display.setCursor(0, 45);
    display.println("Set Time");
    display.setCursor(70, 45);
    display.println("Set Time");
  } else if (Current_Choice == 2) {
    Forward_Menu = 12;
    display.setCursor(0, 15);
    display.println("Set Time");
    display.setCursor(0, 30);
    display.println("> Get Time");
    display.setCursor(0, 45);
    display.println("Set Time");
    display.setCursor(70, 45);
    display.println("Set Time");
  } else if (Current_Choice == 3) {
    Forward_Menu = 13;
    display.setCursor(0, 15);
    display.println("Set Time");
    display.setCursor(0, 30);
    display.println("Get Time");
    display.setCursor(0, 45);
    display.println("> Set Time");
    display.setCursor(70, 45);
    display.println("Set Time");
  } else if (Current_Choice == 4) {
    Forward_Menu = 14;
    display.setCursor(0, 15);
    display.println("Set Time");
    display.setCursor(0, 30);
    display.println("Get Time");
    display.setCursor(0, 45);
    display.println("Set Time");
    display.setCursor(70, 45);
    display.println("> Set Time");
  }
  display.display();
}
// SubMenus
void timemenu11() {
  display.clearDisplay();
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("XXXX", 0, 0, &x1, &y1, &w, &h);
  int16_t x = (128 - w) / 2;
  display.setCursor(x, 0);
  display.println("Time");
  display.display();
}

void timemenu12() {
  display.clearDisplay();
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("XXXX", 0, 0, &x1, &y1, &w, &h);
  int16_t x = (128 - w) / 2;
  display.setCursor(x, 0);
  display.println("Time");
  display.display();
}

void timemenu13() {
  display.clearDisplay();
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("XXXX", 0, 0, &x1, &y1, &w, &h);
  int16_t x = (128 - w) / 2;
  display.setCursor(x, 0);
  display.println("Time");
  display.display();
}

void timemenu14() {
  display.clearDisplay();
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("XXXX", 0, 0, &x1, &y1, &w, &h);
  int16_t x = (128 - w) / 2;
  display.setCursor(x, 0);
  display.println("Time");
  display.display();
}

// BLE Menu
void BLEMenu() {
  display.clearDisplay();
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("XXX", 0, 0, &x1, &y1, &w, &h);
  int16_t x = (128 - w) / 2;
  display.setCursor(x, 0);
  display.println("BLE");
  if (Current_Choice == 1) {
    display.setCursor(0, 15);
    display.println("> Connect");
    display.setCursor(0, 30);
    display.println("Change Time");
    display.setCursor(0, 45);
    display.println("Change EEPROM");
  } else if (Current_Choice == 2) {
    display.setCursor(0, 15);
    display.println("Connect");
    display.setCursor(0, 30);
    display.println("> Change Time");
    display.setCursor(0, 45);
    display.println("Change EEPROM");
  } else if (Current_Choice == 3) { 
    display.setCursor(0, 15);
    display.println("Connect");
    display.setCursor(0, 30);
    display.println("Change Time");
    display.setCursor(0, 45);
    display.println("> Change EEPROM");
  }
  display.display();
}

// EEPROM Menu
void EEPROMMenu() {
  display.clearDisplay();
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("XXXXXX", 0, 0, &x1, &y1, &w, &h);
  int16_t x = (128 - w) / 2;
  display.setCursor(x, 0);
  display.println("EEPROM");
  if (Current_Choice == 1) {
    display.setCursor(0, 15);
    display.println("> Read");
    display.setCursor(0, 30);
    display.println("Write");
    display.setCursor(0, 45);
    display.println("Erase");
    display.setCursor(70, 45);
    display.println("Read");
  } else if (Current_Choice == 2) {
    display.setCursor(0, 15);
    display.println("Read");
    display.setCursor(0, 30);
    display.println("> Write");
    display.setCursor(0, 45);
    display.println("Erase");
    display.setCursor(70, 45);
    display.println("Read");
  } else if (Current_Choice == 3) {
    display.setCursor(0, 15);
    display.println("Read");
    display.setCursor(0, 30);
    display.println("Write");
    display.setCursor(0, 45);
    display.println("> Erase");
    display.setCursor(70, 45);
    display.println("Read");
  } else if (Current_Choice == 4) {
    display.setCursor(0, 15);
    display.println("Read");
    display.setCursor(0, 30);
    display.println("Write");
    display.setCursor(0, 45);
    display.println("Erase");
    display.setCursor(70, 45);
    display.println("> Read");  
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
  if (Current_Choice == 1) {
    display.setCursor(0, 15);
    display.println("> L1");
    display.setCursor(25, 15);
    display.println("  L2");
    display.setCursor(50, 15);
    display.println("  L3");
    display.setCursor(0, 30);
    display.println("  L4");
    display.setCursor(25, 30);
    display.println("  L5");
    display.setCursor(50, 30);
    display.println("  L6");
  } else if (Current_Choice == 2) {
    display.setCursor(0, 15);
    display.println("  L1");
    display.setCursor(25, 15);
    display.println("> L2");
    display.setCursor(50, 15);
    display.println("  L3");
    display.setCursor(0, 30);
    display.println("  L4");
    display.setCursor(25, 30);
    display.println("  L5");
    display.setCursor(50, 30);
    display.println("  L6");
  } else if (Current_Choice == 3) {
    display.setCursor(0, 15);
    display.println("  L1");
    display.setCursor(25, 15);
    display.println("  L2");
    display.setCursor(50, 15);
    display.println("> L3");
    display.setCursor(0, 30);
    display.println("  L4");
    display.setCursor(25, 30);
    display.println("  L5");
    display.setCursor(50, 30);
    display.println("  L6");
  } else if (Current_Choice == 4) {
    display.setCursor(0, 15);
    display.println("  L1");
    display.setCursor(25, 15);
    display.println("  L2");
    display.setCursor(50, 15);
    display.println("  L3");
    display.setCursor(0, 30);
    display.println("> L4");
    display.setCursor(25, 30);
    display.println("  L5");
    display.setCursor(50, 30);
    display.println("  L6");
  } else if (Current_Choice == 5) {
    display.setCursor(0, 15);
    display.println("  L1");
    display.setCursor(25, 15);
    display.println("  L2");
    display.setCursor(50, 15);
    display.println("  L3");
    display.setCursor(0, 30);
    display.println("  L4");
    display.setCursor(25, 30);
    display.println("> L5");
    display.setCursor(50, 30);
    display.println("  L6");
  } else if (Current_Choice == 6) {
    display.setCursor(0, 15);
    display.println("  L1");
    display.setCursor(25, 15);
    display.println("  L2");
    display.setCursor(50, 15);
    display.println("  L3");
    display.setCursor(0, 30);
    display.println("  L4");
    display.setCursor(25, 30);
    display.println("  L5");
    display.setCursor(50, 30);
    display.println("> L6");
  }
  display.display();

    
  }
