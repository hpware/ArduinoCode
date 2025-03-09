// This file is mostly using bootstrap icons turned into OLED displayable content via Claude 3.5 Sonnet on Github Copilot

// Checkmark SVG
void checkmark(int xOffset, int yOffset, int scale) {
    display.drawCircle(xOffset + 8*scale, yOffset + 8*scale, 7*scale, WHITE);
    display.drawLine(xOffset + 5*scale, yOffset + 8*scale, xOffset + 7*scale, yOffset + 10*scale, WHITE);
    display.drawLine(xOffset + 7*scale, yOffset + 10*scale, xOffset + 11*scale, yOffset + 6*scale, WHITE);
    display.drawCircle(xOffset + 8*scale, yOffset + 8*scale, 6*scale, WHITE);
}

// ! mark SVG
void emark(int xOffset, int yOffset, int scale) {
  display.drawLine(xOffset + 5*scale, xOffset + 0*scale, xOffset + 11*scale, xOffset + 0*scale, WHITE); 
  display.drawLine(xOffset + 11*scale, xOffset + 0*scale, xOffset + 16*scale, xOffset + 5*scale, WHITE); 
  display.drawLine(xOffset + 16*scale, xOffset + 5*scale, xOffset + 16*scale, xOffset + 11*scale, WHITE); 
  display.drawLine(xOffset + 16*scale, xOffset + 11*scale, xOffset + 11*scale, xOffset + 16*scale, WHITE); 
  display.drawLine(xOffset + 11*scale, xOffset + 16*scale, xOffset + 5*scale, xOffset + 16*scale, WHITE);
  display.drawLine(xOffset + 5*scale, xOffset + 16*scale, xOffset + 0*scale, xOffset + 11*scale, WHITE);
  display.drawLine(xOffset + 0*scale, xOffset + 11*scale, xOffset + 0*scale, xOffset + 5*scale, WHITE);
  display.drawLine(xOffset + 0*scale, xOffset + 5*scale, xOffset + 5*scale, xOffset + 0*scale, WHITE); 
  display.fillCircle(xOffset + 8*scale, xOffset + 11*scale, scale, WHITE);
  display.fillRect(xOffset + 7*scale, xOffset + 5*scale, 2*scale, 4*scale, WHITE);
}
// Down mark SVG
void downsvg(int xOffset, int yOffset, int scale) {
    int x1 = xOffset + 2*scale;   
    int y1 = yOffset + 4*scale;
    int x2 = xOffset + 8*scale;   
    int y2 = yOffset + 11*scale;
    int x3 = xOffset + 14*scale;  
    int y3 = yOffset + 4*scale;
    display.fillTriangle(x1, y1, x2, y2, x3, y3, WHITE);
}