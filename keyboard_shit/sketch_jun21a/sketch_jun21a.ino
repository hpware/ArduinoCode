#include <Keyboard.h>

// Define the text you want the Arduino to type.
// You can make this as long as you want!

void setup() {
  Keyboard.begin();
}

void loop() {
  Keyboard.println("  cccc");
  delay("100");
}