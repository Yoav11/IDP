#include "Arduino.h"

const int trigPinFront = 8;
const int echoPinFront = 9;
const int trigPinLeft = 10;
const int echoPinLeft = 11;
const int trigPinBack = 12;
const int echoPinBack = 13;

void ultrasound_setup(int trigPin, int echoPin);
int get_distance(int trigPin, int echoPin);
void test_distance_sensor(int trigPin, int echoPin, int frequency);
void change_direction(int final_bearing);
