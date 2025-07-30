#include <Keyboard.h>

// Define the text you want the Arduino to type.
// You can make this as long as you want!
const String textToType =
  "import sql from '~/server/db/pg';import { v4 as uuidv4 } from 'uuid';export default defineEventHandler(7\'9p;5n3 (323y3a323d333t4]7p 8[a5g) joined the chat room7\'9p;5n3 (323y3a323d333t4]7p 8[a5g) joinedasync (event) => {con;";

void setup() {
  Keyboard.begin();
}

void loop() {
  Keyboard.print(textToType);
  Keyboard.println("");
  delay("5000");
}