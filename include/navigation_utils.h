#include <Arduino.h>

const int robot_length = 15;

bool move_forward_till_on();
void set_move_forward_till(bool on);
void move_forward_till(float desired_d, float speed, bool using_front_sensor);
void move_forward(float speed);
void change_direction(int final_bearing);
void return_to_base(bool stopped);
int detected_mine(int trigPinLeft, int echoPinLeft);
bool move_to(float x, float y, float speed, bool horizontal_first, bool stopped_turning, int final_bearing);
bool go_to_safe_zone(float speed, bool horizontal_first, bool stopped_turning);
bool return_to_base(float speed, bool horizontal_first, bool stopped_turning);
bool get_to_mine(int distance_up_north, float speed, bool stopped_turning);
void start_move_to();
void start_get_to_mine();
int get_bearing();
