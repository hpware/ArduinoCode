#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(128, 64, &Wire, -1);


void setup() {
  initDisplay();
  delay(2000);
}

void loop() {
  
}

void initDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("XXXXX", 0, 0, &x1, &y1, &w, &h);
  int16_t x = (128 - w) / 2;
  int16_t y = (64 - h) / 2;
  display.setCursor(x, y);
  display.println("CYIVS");
  display.setCursor(x + 5, y + 15);
  display.println("C2XX");
  display.display();
}