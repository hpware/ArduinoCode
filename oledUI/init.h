#include "values.h"
#include <Arduino.h>
#include "func/buttoninit.h"
#include "func/screeninit.h"



void init() {
    Serial.begin(9600);
    buttoninit();
    screeninit();
}