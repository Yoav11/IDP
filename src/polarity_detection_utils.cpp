#include <polarity_detection_utils.h>

// Set up the Green LED output reader
void polatity_detection_setup() {
  pinMode(polarity_detection_pin, INPUT);
}

// Reads the Green LED output and returns whether the mine is the correct way up or not
bool correct_way_up() {
  delay(200);
  int state = analogRead(polarity_detection_pin);
  if (state < 150) {
    return false;
  } else {
    return true;
  }
}
