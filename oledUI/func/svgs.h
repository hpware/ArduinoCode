// Checkmark SVG
void checkmark() {

}

// ! mark SVG
void emark() {
  // Scale factor for 128x64 OLED display
  int scale = 2;
  int xOffset = 48; // Center on typical 128px wide display
  int yOffset = 16; // Center on typical 64px high display
  
  // Draw octagon outline
  display.drawLine(xOffset + 5*scale, xOffset + 0*scale, xOffset + 11*scale, xOffset + 0*scale, WHITE); // top
  display.drawLine(xOffset + 11*scale, xOffset + 0*scale, xOffset + 16*scale, xOffset + 5*scale, WHITE); // top-right
  display.drawLine(xOffset + 16*scale, xOffset + 5*scale, xOffset + 16*scale, xOffset + 11*scale, WHITE); // right
  display.drawLine(xOffset + 16*scale, xOffset + 11*scale, xOffset + 11*scale, xOffset + 16*scale, WHITE); // bottom-right
  display.drawLine(xOffset + 11*scale, xOffset + 16*scale, xOffset + 5*scale, xOffset + 16*scale, WHITE); // bottom
  display.drawLine(xOffset + 5*scale, xOffset + 16*scale, xOffset + 0*scale, xOffset + 11*scale, WHITE); // bottom-left
  display.drawLine(xOffset + 0*scale, xOffset + 11*scale, xOffset + 0*scale, xOffset + 5*scale, WHITE); // left
  display.drawLine(xOffset + 0*scale, xOffset + 5*scale, xOffset + 5*scale, xOffset + 0*scale, WHITE); // top-left
  
  // Draw exclamation mark - dot
  display.fillCircle(xOffset + 8*scale, xOffset + 11*scale, scale, WHITE);
  
  // Draw exclamation mark - line
  display.fillRect(xOffset + 7*scale, xOffset + 5*scale, 2*scale, 4*scale, WHITE);
}
// Down mark SVG
void downsvg() {
    
}