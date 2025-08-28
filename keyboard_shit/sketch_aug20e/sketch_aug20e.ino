#include "Keyboard.h"
#include "Mouse.h"

void setup() {
  // Initialize Mouse control
  Mouse.begin();
  // Initialize random seed (important for true randomness)
  randomSeed(analogRead(0)); // Use an unconnected analog pin for a more random seed
                               // Or use micros() if analogRead(0) is not available/suitable:
                               // randomSeed(micros());
}

void loop() {
  // --- Parameters for smooth random movement ---
  int maxDistance = 100; // Maximum distance to move in one direction (pixels)
  int minDistance = 20;  // Minimum distance to move (prevents tiny, unnoticeable moves)
  int steps = 15;        // Number of small steps for each movement
  int delayMs = 15;      // Delay between each step (milliseconds)
  int pauseBetweenMoves = 200; // Pause after a complete random movement (milliseconds)

  // 1. Determine a random direction and distance
  int xMove = 0;
  int yMove = 0;
  int currentDistance = random(minDistance, maxDistance + 1); // +1 because random(min, max) is exclusive of max

  // Choose a random direction: 0=Up, 1=Down, 2=Left, 3=Right
  int direction = random(4); // Generates 0, 1, 2, or 3

  switch (direction) {
    case 0: // Up
      yMove = -currentDistance;
      break;
    case 1: // Down
      yMove = currentDistance;
      break;
    case 2: // Left
      xMove = -currentDistance;
      break;
    case 3: // Right
      xMove = currentDistance;
      break;
  }

  // 2. Perform the smooth random movement
  // We'll calculate the step size for x and y independently
  // If xMove is 0, xStep will be 0, and vice-versa
  float xStep = (float)xMove / steps;
  float yStep = (float)yMove / steps;

  for (int i = 0; i < steps; i++) {
    // Mouse.move expects integer values, so we cast to int
    // This can lead to small rounding errors over many steps, but usually imperceptible
    Mouse.move((int)xStep, (int)yStep);
    delay(delayMs);
  }

  // 3. Optional: Add a random click sometimes
  /*if (random(10) < 2) { // 20% chance to click (2 out of 10)
    Mouse.click(MOUSE_LEFT);
    delay(50); // Small delay after click
  }*/

  delay(pauseBetweenMoves); // Pause before the next random movement
}