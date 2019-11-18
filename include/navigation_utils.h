#include <Arduino.h>

int bearing;
bool move_forward_till_is_on = false;

void move_forward_till(float desired_d, float speed);
void move_forward(float speed);
void face_north();
void face_south();
void face_west();
void face_east();
void return_to_base(float turnSpeed, float speed);
