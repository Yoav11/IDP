#include <navigation_utils.h>
#include <ultrasound_utils.h>
#include <motor_utils.h>
#include <ArduinoSort.h>

int bearing = 90;

bool move_forward_till_is_on = false;
int move_forward_till_phase_counts[6] = {0, 0, 0, 0, 0, 0};

int good_distance_count = 0;
int good_distances[10];
int return_base_step = 0;
int final_base_turn = false;

int move_to_phase = 0;
bool move_to_is_on = false;
int get_to_mine_phase = 0;
bool get_to_mine_is_on = false;
bool get_to_mine_first_run = true;
bool get_to_mine_go_back_up_first = true;

float current_time_mine = 0;

// Returns whether the move_forward_till function should be in use.
bool move_forward_till_on() {
  return move_forward_till_is_on;
}

// Set value for whether move_forward_till function should be in use.
void set_move_forward_till(bool on) {
  move_forward_till_is_on = on;
  if (!on) {
      motor_stop();
  }
}

void start_move_to() {
  move_to_is_on = true;
  move_to_phase = 0;
}
void stop_move_to() {
  move_to_is_on = false;
  move_to_phase = 0;
}
void start_get_to_mine() {
  get_to_mine_is_on = true;
  get_to_mine_phase = 0;
}
void stop_get_to_mine() {
  get_to_mine_is_on = false;
  get_to_mine_phase = 0;
  get_to_mine_first_run = true;
  get_to_mine_go_back_up_first = true;
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

void set_counts_for_move_forward_till(int phase) {
  for (int i=0; i<6; i++) {
    if (i == phase) {
      move_forward_till_phase_counts[i]++;
    } else {
      move_forward_till_phase_counts[i] = 0;
    }
  }
}

void set_counts_for_move_forward_till_to_zero() {
  for (int i=0; i<6; i++) {
    move_forward_till_phase_counts[i] = 0;
  }
}

// Given speed (-1 to 1) and desired_d (desired distance in cm) from whichever sensor we're using to the object (whether using the front sensor or back sensor), this function determines whether it should move and how much power it should give to each motor.
void move_forward_till(float desired_d, float speed, bool using_front_sensor) {
  if (!move_forward_till_on()) {return;}

  // NEEDS CHANGE HERE WHEN WE START USING THE BACK SENSOR
  const int distance = get_distance(using_front_sensor ? trigPinFront : trigPinBack, using_front_sensor ? echoPinFront : echoPinBack);
  const float error = desired_d-distance;
  const uint16_t motor_1_direc = (using_front_sensor ? (distance > desired_d) : (distance < desired_d)) ? FORWARD : BACKWARD;
  const uint16_t motor_2_direc = (motor_1_direc == FORWARD) ? BACKWARD : FORWARD;
  const int n_checks = 6;
  const int n_stop_checks = 2;

  uint16_t speed_outOf_255;

// Above 15cm, the speed is as specified when calling this function. Below 15cm, we switch to manual maneuvring.
  if (desired_d <= 100) { // If desired_d is small enough, go for the accurate control
    if (abs(error) >= 15) {
      set_counts_for_move_forward_till(0);
      if (move_forward_till_phase_counts[0] >= n_checks) {
        speed_outOf_255 = convert_to_speed_outOf_255(speed);
      }
    } else if (abs(error) > 10) {
      set_counts_for_move_forward_till(1);
      if (move_forward_till_phase_counts[1] >= n_checks) {
        speed_outOf_255 = convert_to_speed_outOf_255(0.5);
        // Serial.print("Manual maneuvring Phase 1. Error: ");
        // Serial.println(error);
      }

    } else if (abs(error) > 5) {
      set_counts_for_move_forward_till(2);
      if (move_forward_till_phase_counts[2] >= n_checks) {
        speed_outOf_255 = convert_to_speed_outOf_255(0.4);
        // Serial.print("Manual maneuvring Phase 2. Error: ");
        // Serial.println(error);
      }

    } else if (abs(error) >  1) {
      set_counts_for_move_forward_till(3);
      if (move_forward_till_phase_counts[3] >= n_checks/2) {
        speed_outOf_255 = convert_to_speed_outOf_255(0.2);
        Serial.print("Phase 3: Robot pretty close to the desired distance. Error: ");
        Serial.println(error);
      }
    } else if (abs(error) >  0) {
      set_counts_for_move_forward_till(4);
      if (move_forward_till_phase_counts[4] >= n_checks/2) {
        speed_outOf_255 = convert_to_speed_outOf_255(0.15);
        Serial.print("Phase 4: Robot very close to the desired distance. Error: ");
        Serial.println(error);
      }
    } else {
      set_counts_for_move_forward_till(5);
      if (move_forward_till_phase_counts[5] >= n_stop_checks) {
        motor_stop();
      }
      if (move_forward_till_phase_counts[5] >= n_stop_checks+1) {
        move_forward_till_is_on = false;
        set_counts_for_move_forward_till_to_zero();
        Serial.println("Phase 5: Robot stopping since we reached the desired distance");
      }
    }
  } else { // if desired_d is big, choose the control with a slightly bigger error (~3cm)
    if (abs(error) >= 15) {
      set_counts_for_move_forward_till(0);
      if (move_forward_till_phase_counts[0] >= n_checks) {
        speed_outOf_255 = convert_to_speed_outOf_255(speed);
      }
    } else if (abs(error) > 8) {
      set_counts_for_move_forward_till(1);
      if (move_forward_till_phase_counts[1] >= n_checks) {
        speed_outOf_255 = convert_to_speed_outOf_255(0.5);
      }
    } else if (abs(error) > 3) {
      set_counts_for_move_forward_till(2);
      if (move_forward_till_phase_counts[2] >= n_checks) {
        speed_outOf_255 = convert_to_speed_outOf_255(0.3);
      }
    } else {
      set_counts_for_move_forward_till(3);
      if (move_forward_till_phase_counts[3] >= n_stop_checks) {
        motor_stop();
      }
      if (move_forward_till_phase_counts[3] >= n_stop_checks+1) {
        move_forward_till_is_on = false;
        set_counts_for_move_forward_till_to_zero();
        Serial.println("move_forward_till manual maneuvering phase 2: Robot stopping since we reached a close enough distance");
      }
    }
  }


  for (int i=1; i<3; i++) {
    if (move_forward_till_is_on) {
      motor_run(i, speed_outOf_255, (i==1) ? motor_1_direc : motor_2_direc);
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

  if (abs(actual_fin_bearing-bearing) <= 180) {
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

  if (good_distance_count == 10) {
    sortArray(good_distances, 10);
    good_distance_count = 0;
    return good_distances[4];
  }

  return -2;
}

bool move_to(float x, float y, float speed, bool horizontal_first, bool stopped_turning, int final_bearing) {
  if (!move_to_is_on) {return;}
  switch (move_to_phase) {
    case 0:
      Serial.println("move_to phase 0: face first wall");
      change_direction(horizontal_first ? 270 : 180);
      move_to_phase++;
      break;
    case 1:
      if (!stopped_turning) {break;}

      Serial.println("move_to phase 1: switch to the move to first wall phase");
      move_to_phase++;
      set_move_forward_till(true);
      break;
    case 2:
      if (stopped_turning) {break;}

      // Serial.println("move_to phase 2: go to first wall");
      if (horizontal_first) {
        if (x > (240-robot_length)/2) {
          move_forward_till(240 - robot_length - x, 1.0, false);
        } else {
          move_forward_till(x, 1.0, true);
        }
      } else {
        if (y > (240-robot_length)/2) {
          move_forward_till(240 - robot_length - y, 1.0, false);
        } else {
          move_forward_till(y, 1.0, true);
        }
      }

      if (!move_forward_till_on()) { // when the robot has stopped
        Serial.println("robot has stopped");
        move_to_phase++;
      }
      break;
    case 3: // Face second wall
      Serial.println("move_to phase 3: face second wall");
      change_direction(horizontal_first ? 180 : 270);
      move_to_phase++;
      break;
    case 4: // Change the mode to moving to the west wall phase
      if (!stopped_turning) {break;}

      Serial.println("move_to phase 4: switch to the move to second wall phase");
      move_to_phase++;
      set_move_forward_till(true);
      break;
    case 5:
      if (stopped_turning) {break;}

      Serial.println("go_to_safe_zone phase 2: go to north wall");
      if (horizontal_first) { // hence vertical this time
        if (y > (240-robot_length)/2) { // y is quite big, then use the back sensor
          move_forward_till(240 - robot_length - y, 1.0, false);
        } else { // if y is small, then use the front sensor
          move_forward_till(y, 1.0, true);
        }
      } else { // hence horizontal this time
        if (x > (240-robot_length)/2) { // x is quite big, then use the back sensor
          move_forward_till(240 - robot_length - x, 1.0, false);
        } else { // if y is small, then use the front sensor
          move_forward_till(x, 1.0, true);
        }
      }

      if (!move_forward_till_on()) { // when the robot has stopped
        move_to_phase++;
      }
      break;
    case 6:
      Serial.println("move_to phase 6: turn to the final bearing");
      change_direction(final_bearing);
      move_to_phase++;
      break;
    case 7:
      if (!stopped_turning) {break;}
      Serial.print("Moved to coordinates:");
      Serial.print(x);
      Serial.print(",");
      Serial.println(y);
      stop_move_to();
      return true;
      break;
  }
  return false;
}

bool go_to_safe_zone(float speed, bool horizontal_first, bool stopped_turning) {
  return move_to(10, 200, speed, horizontal_first, stopped_turning, 0);
}

bool return_to_base(float speed, bool horizontal_first, bool stopped_turning) {
  return move_to(10, 20, speed, horizontal_first, stopped_turning, 90);
}

bool get_to_mine(int distance_up_north, float speed, bool stopped_turning) {
  if (!get_to_mine_is_on) {return;}
  int duration = 500; // ms
  switch (get_to_mine_phase) {
    case 0: // back up a bit
        move_forward(-speed);
        if (get_to_mine_first_run){
          Serial.println("get_to_mine phase 0: Backing up a bit");
          current_time_mine = millis();
          get_to_mine_first_run = false;
        }
        if (millis() - current_time_mine > duration) {
          motor_stop();
          get_to_mine_phase++;
        }
        break;
    case 1: // turn north
      Serial.println("get_to_mine phase 1: turning north");
      change_direction(0);
      get_to_mine_phase++;
      break;
    case 2: // switch to "move up north"
      if (!stopped_turning) {break;}
      Serial.println("get_to_mine phase 2: switch to phase 3(the move up north)");
      get_to_mine_phase++;
      set_move_forward_till(true);
      break;
    case 3: // move up north
      if (stopped_turning) {break;}
      move_forward_till(distance_up_north, 1.0, false);
      if (!move_forward_till_on()) {
        Serial.println("get_to_mine phase 3 end: finished moving up north");
        get_to_mine_phase++;
      }
      break;
    case 4: // turn east
      Serial.println("get_to_mine phase 4: turning east");
      change_direction(90);
      get_to_mine_phase++;
      break;
    case 5:
      if (!stopped_turning) {break;}
      get_to_mine_phase++;
      break;
    case 6:
      Serial.println("Got to the mine!");
      stop_get_to_mine();
      return true;
  }
  return false;
}
