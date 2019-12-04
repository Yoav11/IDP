#include "Arduino.h"

const int trigPinFront = 2;
const int echoPinFront = 3;
const int trigPinLeft = 10;
const int echoPinLeft = 11;
const int trigPinBack = 4;
const int echoPinBack = 5;

void ultrasound_setup();
int get_distance(int trigPin, int echoPin);
void test_distance_sensor(int trigPin, int echoPin, int frequency);
void change_direction(int final_bearing);
