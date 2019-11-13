#include "Arduino.h"

void ultrasound_setup(int trigPin, int echoPin);
int get_distance(int trigPin, int echoPin);
void test_distance_sensor(int trigPin, int echoPin, int frequency);
