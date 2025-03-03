// This file is mostly using bootstrap icons turned into OLED displayable content

#ifndef SVGS_H
#define SVGS_H

// Checkmark SVG
void checkmark(int xOffset, int yOffset) {
    int scale = 1;
    display.drawCircle(xOffset + 8*scale, yOffset + 8*scale, 7*scale);
    
    display.drawLine(xOffset + 5*scale, yOffset + 8*scale, xOffset + 7*scale, yOffset + 10*scale);
    display.drawLine(xOffset + 7*scale, yOffset + 10*scale, xOffset + 11*scale, yOffset + 6*scale);
}

// ! mark SVG
void emark(int xOffset, int yOffset) {
  int scale = 1;
  display.drawLine(xOffset + 5*scale, yOffset + 0*scale, xOffset + 11*scale, yOffset + 0*scale);
  display.drawLine(xOffset + 8*scale, yOffset + 2*scale, xOffset + 8*scale, yOffset + 9*scale);
  display.drawPixel(xOffset + 8*scale, yOffset + 11*scale);
}

// Down mark SVG
void downsvg(int x, int y) {
    int width = 8;
    int height = 8;
    
    
    int x1 = x + width / 2;
    int y1 = y + height - 2;
    int x2 = x + 2;
    int y2 = y + 2;
    int x3 = x + width - 2;
    int y3 = y + 2;
    
    display.drawTriangle(x1, y1, x2, y2, x3, y3);
    display.drawTriangle(x1, y1-1, x2+1, y2+1, x3-1, y3+1); 
}

#endif 