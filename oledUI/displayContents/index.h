// OG Design: https://i.imgur.com/58N0Im3.png

// Import SVGs
#include "../func/svgs.h"

bool initialized = false;
int refresh = 0;
String dataStr = "No data"; 
bool dataFail = false; 
// DATE STUFF
String date = "2024-12-1";
int hourVal = 00;
int minuteVal = 00;
int secVal = 00;
String dataminVal ="";
String datahrVal = "";
// OTHER DATA
float phData = 0;
float tempData = 0;

void displayindex() {
    //display.fillRect(10,1,3,77,BLACK);
    display.setCursor(10,1);
    display.print(date);
    display.setTextSize(1);
    display.setCursor(6, 10);
    // Time Component
    // Second
    if (secVal == 60 || !initialized) {
        secVal = 0;
        minuteVal += 1;
        if (!initialized) {
            minuteVal -= 1;
        }
        if (String(minuteVal).length() == 1) {
            dataminVal = "0" + String(minuteVal);
        } else {
            dataminVal = String(minuteVal);
        }
        if (String(hourVal).length() == 1) {
            datahrVal = "0" + String(hourVal);
        } else {
            datahrVal = String(hourVal);
        }
        display.fillRect(6, 10, 120, 20, BLACK);
        display.print(datahrVal + ":" + dataminVal); 
    } else {
        secVal += 1;
    }
    // Minute
    if (minuteVal == 60) {
        minuteVal = 00;
    }
    // Hour 
    if (hourVal == 24) {
        hourVal = 00;
    }
    display.setTextSize(1); 
    display.setCursor(6,28);
    display.print("WiFi:");
    if (!dataFail) {
        display.print(ssid);
        display.print("\n");
    } else {
        display.print("Error");
        display.print("\n");
    }
    display.setCursor(6,37);
    display.print("IP:");
    if (!dataFail) { 
        display.print(ipAddr);
        //checkmark(20,28,1);
        display.print("\n");
    } else {
        //emark(80,2,2);
        display.print("Error");
        display.print("\n");
    }
    /**if (!initialized) { 
        display.setCursor(23,44); 
        display.print("Feed");
        downsvg(25, 50, 1);
        display.setCursor(56,44);
        disdplay.print("RST");
        downsvg(55, 50, 1);
        display.setCursor(82, 44);
        display.print("UDT");
        downsvg(81,50,1);
        initialized = true;
    }*/
    if (lastPH != phData || !initialized) {
        display.setCursor(70,4);
        display.print(phData);
        display.print(" ph");
        phData = lastPH;
    }
    if (tempData != lastTemp || !initialized) {
        display.setCursor(70,15);
        display.print(tempData);
        display.print(" C");
        tempData = lastTemp;
    }
    display.display();
    if (refresh == 3600) { 
        refresh = 0;
        initialized = false;
    } else {
        refresh += 1;
    }
}