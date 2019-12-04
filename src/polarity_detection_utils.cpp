#include <polarity_detection_utils.h>

void polatity_detection_setup() {
  pinMode(polarity_detection_pin, INPUT);
}

bool correct_way_up() {
  delay(200);
  int state = analogRead(polarity_detection_pin);
  if (state < 150) {
    return false;
  } else {
    return true;
  }
}
