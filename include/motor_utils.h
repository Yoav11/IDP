/*
helper functions for interfacing with motors
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <utility/Adafruit_MS_PWMServoDriver.h>

void motor_begin();
void motor_run(int motor_index, uint16_t speed, uint16_t direction); // motor_index is either 1 or 2, speed is between 0 and 255, direction is either FORWARD or BACKWARD
void motor_stop();
void motor_turn(float angle);
bool stop_ticker();
void square_test();
bool close_gripper();
