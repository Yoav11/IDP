#include <servo_utils.h>

Servo myservo;

void servo_setup() {
  myservo.attach(servo_pin);
}

// Moves the servo. angle is between 0 and 180
void move_servo(int angle) {
  int actual_angle = angle;

  while (actual_angle < 0) {
    actual_angle += 180;
  }
  while (actual_angle > 180) {
    actual_angle -= 180;
  }

  myservo.write(actual_angle);
}
