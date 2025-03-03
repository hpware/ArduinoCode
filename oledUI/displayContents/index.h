// Design: https://i.imgur.com/58N0Im3.png

// Import SVGs
#include "../func/svgs.h"
// values within the component.
int timex = 10;
int timey = 2;
bool ampmchangeb = false;
int refresh = 0;
// index of the display component.
void displayindex() {
    // Init
    String ampmchange = "";
    // Date Component
    display.setCursor(1,2);
    if (!data) {
        display.println(data);
    }
    // time component
    // clear area
    display.fillRect(timex, timey, 3, 77, BLACK);
    display.setCursor(timex, timey);
    display.print(hr + ":" + min);
    // am pm change component
    display.setCuror(40,30);
    if (ampmchangeb = true) {
        display.print(ampmchange);
        ampbchangeb = false;
    }
    // Network status
    display.setCursor(70,50);
    if (!data.fail) {
        display.print("Internet:");
        checkmark();
        display.print(ipaddr);
        display.print("\n")
    } else {
        display.print("Internet");
        xmark();
        display.print("錯誤");
        display.print("\n");
    }
    // arrows to navagate
    if (!init) {
        display.setCusor(77,23);
        display.print("投餌");
        downsvg();
        display.setCursor(77,25);
        display.print("重新連線");
        downsvg();
        display.setCursor(77, 27);
        display.print("更新資料");
        downsvg();
    }
    // end
    display.display();
    // forced to refresh every single hour
    if (refresh = 3600) {
        refresh = 0;
    } else {
        refresh += 1;
    }
}