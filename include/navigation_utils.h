#include <Arduino.h>

bool move_forward_till_on();
void set_move_forward_till(bool on);
void move_forward_till(float desired_d, float speed, bool is_forward);
void move_forward(float speed);
void change_direction(int final_bearing);
void return_to_base(bool stopped);
int detected_mine(int trigPinLeft, int echoPinLeft);
