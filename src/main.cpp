#include <Arduino.h>
#include <motor_utils.h>
#include <led_utils.h>
#include <navigation_utils.h>
#include <ultrasound_utils.h>
#include <servo_utils.h>

int old_routine_step;
bool stopped = true;
int phase = 0;
int step = 1;
int detected_distance;
int previous_step = 0;
float current_time;
float distance;
float temp_distance;

bool got_to_mine = false;
bool got_to_safe_zone = false;
bool got_to_base = false;

bool gripper_closed;
bool servo_lowered = false;
float servo_time;

int robot_bearing;

void setup() {
    // pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
    motor_begin();
    servo_setup();
    delay(2000);
    ultrasound_setup();
    robot_bearing = get_bearing();
    start_adjust_angle();
}


void loop() {
    stopped = stop_ticker();
    temp_distance = detected_mine(trigPinLeft, echoPinLeft);

    switch(step) {
        case 1:
            if (adjust_angle(1.0)) {
              step++;
              set_move_forward_till(true);
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
        case 4:
            got_to_mine = get_to_mine(distance + 30, 0.5, stopped);
            if(got_to_mine) {
                start_move_to();
                got_to_mine = false;
                step++;
            }
            break;
        case 5:
            gripper_closed = close_gripper();
            if(gripper_closed) {
                step++;
            }
        case 6:
            if (!servo_lowered) {
              servo_time = millis();
              move_servo(180);
              servo_lowered = true;
            }
            if (millis() - servo_time > 1000 && servo_lowered) {
              step++;
            }
            break;
        case 7:
            // gripper time
            delay(1000);
            step++;
            break;
        case 8:
            if (servo_lowered) {
              servo_time = millis();
              move_servo(0);
              servo_lowered = false;
            }
            if (millis() - servo_time > 1000 && !servo_lowered) {
              step++;
            }
            break;
        case 9:
            got_to_safe_zone = go_to_safe_zone(1.0, true, stopped);
            if(got_to_safe_zone) {
                got_to_safe_zone = false;
                step++;
                start_move_to();
            }
            break;
        case 10:
            // gripper time
            delay(1000);
            step++;
            break;
        case 11:
            got_to_base = return_to_base(1.0, true, stopped);
            if(got_to_base) {
                step = 1;
                 got_to_base = false;
                robot_bearing = get_bearing();
                start_adjust_angle();
            }
            break;
    }
}
