#include <Arduino.h>
#include <motor_utils.h>
#include <led_utils.h>
#include <navigation_utils.h>
#include <ultrasound_utils.h>
#include <servo_utils.h>
#include <Ticker.h>
#include <polarity_detection_utils.h>

int old_routine_step;
bool stopped = true;
int phase = 0;
int step = 1;
int detected_distance;
int previous_step = 0;
float current_time;
float distance;
float temp_distance;

int global_timer = 0;

bool got_to_mine = false;
bool got_to_safe_zone = false;
bool got_to_base = false;

bool gripper_closed;
bool servo_lowered = false;
float servo_time;

int robot_bearing;

bool first_mine = true;
int first_mine_step = 0;

Ticker amber_led(blink_builtin, 250);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    amber_led.start();
    polatity_detection_setup();
    Serial.begin(9600);
    motor_begin();
    servo_setup();
    delay(2000);
    ultrasound_setup();
    robot_bearing = get_bearing();
    start_adjust_angle();
}

// void loop() {
//   if (correct_way_up()) {
//     lower_servo();
//   } else {
//     raise_servo();
//   }
// }

// void loop() {
//   stopped = stop_ticker();
//   return_to_base(1.0, true, stopped);
// }

// void loop() {
//   if (lower_servo()) {
//     if (close_gripper(true)) {
//       if (close_gripper(false)) {
//         if (raise_servo()) {
//
//         }
//       }
//     }
//   }
// }

// void loop() {
//   if (lower_servo()) {
//     if (raise_servo()) {
//       return;
//     }
//   }
// }


void loop() {

    // Serial.print("front: ");
    // test_distance_sensor(trigPinFront, echoPinFront, 2);
    // Serial.print("back: ");
    // test_distance_sensor(trigPinBack, echoPinBack, 2);
    // Serial.print("left: ");
    // test_distance_sensor(trigPinLeft, echoPinLeft, 2);

    amber_led.update();

    stopped = stop_ticker();
    temp_distance = detected_mine(trigPinLeft, echoPinLeft);

    if (first_mine) {
      switch (first_mine_step) {
        case 0:
          if (adjust_angle(1.0)) {
            first_mine_step++;
            set_move_forward_till(true);
          }
          break;
        case 1:
          move_forward_till(25, 0.5, false);
          if (!move_forward_till_on()) {
            first_mine_step++;
            start_move_to();
          }
          break;
        case 2:
          if (move_to(30, 106, 1.0, false, stopped, 90)) {
            first_mine_step++;
            step = 6;
            first_mine = false;
          }
          break;
      }
      return;
    }

    switch(step) {
        case 1:
            if (adjust_angle(1.0)) {
              step++;
              // set_move_forward_till(true);
            }
            break;
        case 2:
            change_direction(robot_bearing);
            robot_bearing = get_bearing();
            step++;
            break;
        case 3:
            if (!stopped) {break;}
            step++;
            set_move_forward_till(true);
            break;
        case 4:
            if (temp_distance >= 0){
                distance = temp_distance;
                start_get_to_mine();
                step++;
            }
            else if(move_forward_till_on()) {
                move_forward_till(17, 0.3, true);
            } else {
                robot_bearing -= 90;
                step = 2;
            }
            break;
        case 5:
            got_to_mine = get_to_mine(distance + 30, 0.5, stopped);
            if(got_to_mine) {
                // start_move_to();
                got_to_mine = false;
                step++;
            }
            break;
        case 6:
            if (lower_servo()) {
              step++;
            }
            break;
        case 7:
            gripper_closed = close_gripper(true);
            if(gripper_closed) {
                step++;
                gripper_closed = false;
            }
            break;
        case 8:
            gripper_closed = close_gripper(false);
            if(gripper_closed) {
                step++;
                gripper_closed = false;
            }
            break;
        case 9:
            if (correct_way_up()) {
              move_servo(150);
              step++;
              start_move_to();
            } else {
              if (raise_servo()) {
                step++;
                start_move_to();
              }
            }
            break;
        case 10:
            got_to_safe_zone = go_to_safe_zone(1.0, true, stopped);
            if(got_to_safe_zone) {
                got_to_safe_zone = false;
                step++;
                // start_move_to();
                start_return();
            }
            break;
        case 11:
            if (raise_servo()) {
              if (close_gripper(true)) {
                Serial.println("closed");
                step++;
              }
            }
            break;
        case 12:
            if (lower_servo()) {
              if (close_gripper(false)) {
                Serial.println("opened");
                step++;
              }
            }
            break;
        case 13:
            if (raise_servo()) {
              step++;
              start_return();
            }
        case 14:
            got_to_base = return_to_base(1.0, true, stopped);
            if(got_to_base) {
                step = 1;
                 got_to_base = false;
                robot_bearing = get_bearing();
                start_adjust_angle();
                global_timer++;
                if(global_timer >= 2) {
                    while(1) {}
                }
            }
            break;
    }
}
