// Design: https://i.imgur.com/58N0Im3.png

// Import SVGs
#include "../func/svgs.h"
// values within the component.
int timex = 10;
int timey = 12; 
bool ampmchangeb = false;
int refresh = 0;String hourStr = "12";
String minuteStr = "00";
bool initialized = false;
String data = "No Data";
bool dataFail = false;
void displayindex() {
    display.clearBuffer();
    display.setFont(u8g2_font_profont12_tr);
    String ampmchange = "";
    display.setCursor(1, 12);
    char dataBuffer[50];
    data.toCharArray(dataBuffer, 50);
    display.drawStr(1, 12, dataBuffer);  
    char timeBuffer[10];
    String timeStr = hourStr + ":" + minuteStr;
    timeStr.toCharArray(timeBuffer, 10);
    display.drawStr(timex, timey, timeBuffer); 
    if (ampmchangeb == true) {
        char ampmBuffer[10];
        ampmchange.toCharArray(ampmBuffer, 10);
        display.drawStr(40, 40, ampmBuffer);     
    }
    if (!dataFail) {
        display.drawStr(70, 60, "Internet:");
        checkmark(70,60);
    } else {
        display.drawStr(70, 60, "Internet");
        emark(70, 60);
        display.drawStr(90, 60, "錯誤");
    }
    if (!initialized) {
        display.drawStr(77, 33, "投餌");
        downsvg(77,33);
        display.drawStr(77, 45, "重新連線");
        downsvg(77,45);
        display.drawStr(77, 57, "更新資料");
        downsvg(77,57);
    }
    display.sendBuffer();
    if (refresh == 3600) {
        refresh = 0;
    } else {
        refresh += 1;
    }
}