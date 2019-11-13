#include "Arduino.h"
#include <ultrasound_utils.h>

const int trigPin = 9;
const int echoPin = 10;

void setup() {
  // put your setup code here, to run once:
  ultrasound_setup(trigPin, echoPin);
  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  // put your main code here, to run repeatedly:
test_distance_sensor(trigPin, echoPin, 20);
}
