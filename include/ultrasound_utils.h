#include "Arduino.h"

const int trigPinFront = 9;
const int echoPinFront = 10;
const int trigPinLeft = 11;
const int echoPinLeft = 12;

void ultrasound_setup(int trigPin, int echoPin);
int get_distance(int trigPin, int echoPin);
void test_distance_sensor(int trigPin, int echoPin, int frequency);
void change_direction(int final_bearing);
