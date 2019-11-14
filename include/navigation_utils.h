#include <Arduino.h>

int bearing;

// Remove this after testing
void motor_begin();
void motor_run(int motor_index, float speed);
void motor_stop();
void motor_turn(float angle);
// ------------
