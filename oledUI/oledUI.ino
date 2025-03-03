#include <Arduino.h>
#include "init.h"
#include "values.h"
#include "displayContents/index.h"

void setup() {
  init();
}

void loop() {
  displayindex();
  delay(1000);
}
