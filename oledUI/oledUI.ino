#include <Arduino.h>
#include "init.h"
#include "values.h"
#include "displayContents/index.h"
#include "func/mqtt2-ph.h"

void setup() {
  init();
}

void loop() {
  displayindex();
  delay(1000);
  readPhTemp();
}
