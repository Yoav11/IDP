#include "Arduino.h"
#include <ultrasound_utils.h>

void setup() {
  ultrasound_setup(trigPinFront, echoPinFront);
  Serial.begin(9600); // Starts the serial communication
}

void loop() {
}
