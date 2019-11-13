#include "Arduino.h"
#include <ultrasound_utils.h>

const int trigPin = 9;
const int echoPin = 10;

void setup() {
  ultrasound_setup(trigPin, echoPin);
  Serial.begin(9600); // Starts the serial communication
}

void loop() {
}
