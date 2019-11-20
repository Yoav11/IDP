#include <navigation_utils.h>
#include <ultrasound_utils.h>
#include <motor_utils.h>
#include <ArduinoSort.h>

int bearing = 90;
bool move_forward_till_is_on = false;
int good_distance_count = 0;
int good_distances[10];

// Returns whether the move_forward_till function should be in use.
bool move_forward_till_on() {
  return move_forward_till_is_on;
}

// Set value for whether move_forward_till function should be in use.
void set_move_forward_till(bool on) {
  move_forward_till_is_on = on;
}

// Converts the speed (-1 to 1) to the speed that can be read by the motors (0 to 255). Only the magnitude counts.
uint16_t convert_to_speed_outOf_255(float speed) {
  return 255.0 * fabs(speed);
}

// Returns the direction of the motor1 from speed (-1 to 1)
uint16_t motor_1_direction(float speed) {
  return (speed > 0) ? FORWARD : BACKWARD;
}

// Returns the direction of the motor2 from speed (-1 to 1)
uint16_t motor_2_direction(float speed) {
  return (motor_1_direction(speed) == FORWARD) ? BACKWARD : FORWARD;
}

// Given speed (-1 to 1) and desired_d (desired distance in cm), this function determines whether it should move and how much power it should give to each motor.
void move_forward_till(float desired_d, float speed) {
  const int distance = get_distance(trigPinFront, echoPinFront);
  const float error = desired_d-distance;
  const uint16_t motor_1_direc = (distance > desired_d) ? FORWARD : BACKWARD;
  const uint16_t motor_2_direc = (motor_1_direc == FORWARD) ? BACKWARD : FORWARD;

  Serial.print("distance: ");
  Serial.println(distance);
  uint16_t speed_outOf_255;
// Above 10cm, the speed is as specified when calling this function. Below 10cm, we switch to manual maneuvring.
  if (abs(error) >= 15) {
    speed_outOf_255 = convert_to_speed_outOf_255(speed);
} else if (abs(error) > 10) {
    speed_outOf_255 = convert_to_speed_outOf_255(0.5);
    Serial.print("Manual maneuvring Phase 1. Error: ");
    Serial.println(error);
} else if (abs(error) > 5) {
    speed_outOf_255 = convert_to_speed_outOf_255(0.4);
    Serial.print("Manual maneuvring Phase 2. Error: ");
    Serial.println(error);
  } else if (abs(error) >  1) {
    speed_outOf_255 = convert_to_speed_outOf_255(0.2);
    Serial.print("Phase 3: Robot pretty close to the desired distance. Error: ");
    Serial.println(error);
  } else if (abs(error) >  0) {
    speed_outOf_255 = convert_to_speed_outOf_255(0.15);
    Serial.print("Phase 4: Robot very close to the desired distance. Error: ");
    Serial.println(error);
  } else {
    motor_stop();
    move_forward_till_is_on = false;
    Serial.println("Phase 5: Robot stopping since we reached the desired distance");
  }

  for (int i=1; i<3; i++) {
    if (move_forward_till_is_on) {
      motor_run(i, speed_outOf_255, (i==1) ? motor_1_direc : motor_2_direc);
      Serial.print("setting speed at: ");
      Serial.println(speed_outOf_255);
    }
  }
}

void move_forward(float speed) {
  for (int i=1; i<3; i++) {
    motor_run(i, convert_to_speed_outOf_255(speed), (i==1) ? motor_1_direction(speed) : motor_2_direction(speed));
  }
}

// This function would turn the robot to face a desired bearing.
void change_direction(int final_bearing) {
  motor_stop();

  int actual_fin_bearing = final_bearing;
  if (final_bearing < 0) {
      while (actual_fin_bearing < 0) {
          actual_fin_bearing += 360;
      }
  } else if (final_bearing > 360) {
      while (actual_fin_bearing > 360) {
          actual_fin_bearing -= 360;
      }
  }

  if (abs(actual_fin_bearing-bearing) < 180) {
    motor_turn(actual_fin_bearing - bearing);
  } else if (bearing >= 0 && bearing <= 180) {
    motor_turn((actual_fin_bearing-360) - bearing);
  } else {
    motor_turn((actual_fin_bearing+360) - bearing);
  }

  bearing = actual_fin_bearing;
}

// This function returns a positive number if a mine is detected (in cm), and a negative number if not detected. Call it in the loop function.
int detected_mine(int trigPinLeft, int echoPinLeft) {
  int distance = get_distance(trigPinLeft, echoPinLeft);
  if (distance < 80) {
    good_distances[good_distance_count] = distance;
    good_distance_count++;
  } else {
    good_distance_count = 0;
    return -1;
  }

  if (good_distance_count == 5) {
    sortArray(good_distances, 10);
    good_distance_count = 0;
    Serial.print("Detected mine at:");
    Serial.println(good_distances[4]);
    return good_distances[4];
  }

  return -2;
}
