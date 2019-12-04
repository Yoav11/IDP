#include <servo_utils.h>

Servo myservo;

float move_servo_timer;
float raise_servo_timer;
float lower_servo_timer;
bool move_servo_first = true;
bool raise_servo_first = true;
bool lower_servo_first = true;

void servo_setup() {
  myservo.attach(servo_pin);
  move_servo(0);
}

// Moves the servo. angle is between and including 20 and 168
// 20 is up, 168 is down
bool move_servo(int angle) {
  int actual_angle = angle;

  while (actual_angle < 0) {
    actual_angle += 180;
  }
  while (actual_angle > 180) {
    actual_angle -= 180;
  }

  if (actual_angle < 20) {
    actual_angle = 20;
  } else if (actual_angle > 165) {
    actual_angle =  165;
  }

  if (move_servo_first) {
    move_servo(actual_angle);
    move_servo_timer = millis();
    move_servo_first = false;
  }

  if (millis() - raise_servo_timer > 2000) {
    move_servo_first = true;
    return true;
  }
  return false;

  // myservo.write(actual_angle);
}

bool raise_servo() {
  if (raise_servo_first) {
    move_servo(0);
    raise_servo_timer = millis();
    raise_servo_first = false;
  }

  if (millis() - raise_servo_timer > 2000) {
    raise_servo_first = true;
    return true;
  }
  return false;
}

bool lower_servo() {
  if (lower_servo_first) {
    move_servo(180);
    lower_servo_timer = millis();
    lower_servo_first = false;
  }

  if (millis() - lower_servo_timer > 2000) {
    lower_servo_first = true;
    return true;
  }
  return false;
}
