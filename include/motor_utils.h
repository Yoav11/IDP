/*
helper functions for interfacing with motors
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <utility/Adafruit_MS_PWMServoDriver.h>

void motor_begin();
void motor_run(int motor_index, uint16_t speed, uint16_t direction);
void motor_stop();
void motor_turn(float angle);
bool stop_ticker();
void square_test();
