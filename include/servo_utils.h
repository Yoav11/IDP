  #include <Servo.h>
#include <Arduino.h>

const int servo_pin = 6;

void servo_setup();
bool move_servo(int angle);
bool raise_servo();
bool lower_servo();
