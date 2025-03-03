#include <Wire.h>
#include <U8g2lib.h>

// Using the constructor for SSD1327 I2C
// Format: U8G2_SSD1327_MIDAS_128X128_F_HW_I2C u8g2(rotation, [reset [, clock, data]])
U8G2_SSD1327_MIDAS_128X128_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void screeninit() {
  Serial.println("Initializing SSD1327 display with U8g2...");
  
  // Begin Wire for I2C communication
  Wire.begin(sda, scl);
  
  // Initialize display
  display.begin();
  
  Serial.println("SSD1327 initialized successfully!");
  
  // Clear the display buffer
  display.clearBuffer();
  
  // Set font
  display.setFont(u8g2_font_profont12_tr);
  
  // Draw text
  display.drawStr(0, 12, "SSD1327 Initialized");
  
  // Send buffer to display
  display.sendBuffer();
  
  delay(2000);
}
