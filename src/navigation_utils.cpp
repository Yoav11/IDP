#include <navigation_utils.h>
#include <ultrasound_utils.h>
#include <motor_utils.h>

// Upon calling this function once, the robot moves forward till a desired distance is achieved.
void move_forward_till(float desired_d, float speed) {
  // while (abs(get_distance(trigPinFront, echoPinFront) - desired_d) > 5) {
  //   const uint16_t direction = (get_distance(trigPinFront, echoPinFront) - desired_d > 0)? FORWARD : BACKWARD;
  //   for (int i=0; i<2; i++) {
  //     motor_run(i, speed, direction);
  //   }
  // }
  //
  // while (abs(get_distance(trigPinFront, echoPinFront) - desired_d) > 2) {
  //   const int direction = (get_distance(trigPinFront, echoPinFront) - desired_d) / abs(get_distance(trigPinFront, echoPinFront) - desired_d); // This is either +1 or -1 indicating the direction of travel.
  //   for (int i=0; i<2; i++) {
  //     motor_run(i, direction * speed/2);
  //   }
  // }

  const uint16_t direction = (get_distance(trigPinFront, echoPinFront) > desired_d) ? FORWARD : BACKWARD;
  
  // If this function is called constantly to check the distance, then commment the above and uncomment the below
  if (abs(desired_d - get_distance(trigPinFront, echoPinFront)) > 5) {
    for (int i=0; i<2; i++) {
      motor_run(i, speed, direction);
    }
  } else if (abs(desired_d - get_distance(trigPinFront, echoPinFront)) > 5) {
    for (int i=0; i<2; i++) {
      motor_run(i, speed, direction);
    }
  }




  if (desired_d > get_distance(trigPinFront, echoPinFront) + 5) {
    for (int i=0; i<2; i++) {
      // motor_run(i, speed);
      motor_run(i, speed, direction);
    }
  } else if (desired_d > get_distance(trigPinFront, echoPinFront)) {
    for (int i=0; i<2; i++) {
      motor_run(i, speed/2, direction);
    }
  } else if (desired_d > get_distance(trigPinFront, echoPinFront) + 5) {
    for (int i=0; i<2; i++) {
      motor_run(i, -speed);
    }
  } else {
    for (int i=0; i<2; i++) {
      motor_run(i, -speed/2);
    }
  }
}

void move_forward(float speed) {
  for (int i=0; i<2; i++) {
    motor_run(i, speed);
  }
}

void face_north() {
  motor_stop(); // Stop before turning just in case it hasn't stopped.
  if (bearing >= 0 && bearing <= 180) { // If it's facing to the East ish, turn anti-clockwise to face North.
    motor_turn(0 - bearing);
  } else { // If it's facing to the West ish, turn clockwise to face North.
    motor_turn(360-bearing);
  }
}

void face_south() {
  motor_stop(); // Stop before turning just in case it hasn't stopped.
  motor_turn(180-bearing);
}

void face_east() {
  motor_stop(); // Stop before turning just in case it hasn't stopped.
  if (bearing >= 270 && bearing <= 360) { // If it's facing to the North West ish, turn clockwise to face East.
    motor_turn(450 - bearing);
  } else { // Elswhere, 90-bearing should do the job whichever direction we are turning.
    motor_turn(90 - bearing);
  }
}

void face_west() {
  motor_stop();
  if (bearing >= 0 && bearing <= 90) {
    motor_turn(-90 - bearing);
  } else {
    motor_turn(270 - bearing);
  }
}

// Upon calling this function once, the robot returns to base.
void return_to_base(float turnSpeed, float speed) {
  face_west();

  while (get_distance(trigPinFront, echoPinFront) > 10) {
    move_forward(speed);
  }

  motor_stop();

  if (get_distance(trigPinLeft, echoPinLeft) < 200) {
    face_north();
    move_forward_till(45, speed);
  } else {
    face_south();
    move_forward_till(200, speed);
  }
}
