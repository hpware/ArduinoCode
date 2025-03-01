#include "../values.h"
#include <Arduino.h>
#include "func/buttoninit.h"
#include "func/screeninit.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


void init() {
    Serial.begin(9600);
    buttoninit();
    screeninit();
}