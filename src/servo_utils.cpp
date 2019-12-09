#include <servo_utils.h>

Servo myservo;

float servo_timer;
bool servo_first = true;

// sets up the servo connection
void servo_setup() {
  myservo.attach(servo_pin);
  move_servo(0);
}

// Moves the servo. angle is between and including 20 and 165
// 20 is up, 168 is down
void move_servo(int angle) {
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

  myservo.write(actual_angle);
}

// call this in a loop to raise servo. It returns true when it is complete
bool raise_servo() {
  if (servo_first) {
    move_servo(0);
    servo_timer = millis();
    servo_first = false;
  }

  if (millis() - servo_timer > 2000) {
    servo_first = true;
    return true;
  }
  return false;
}

// call this in a loop to lower servo. It returns true when it is complete
bool lower_servo() {
  if (servo_first) {
    move_servo(180);
    servo_timer = millis();
    servo_first = false;
  }

  if (millis() - servo_timer > 2000) {
    servo_first = true;
    return true;
  }
  return false;
}
