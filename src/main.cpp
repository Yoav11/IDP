#include <Arduino.h>
#include <motor_utils.h>
#include <led_utils.h>
#include <navigation_utils.h>
#include <ultrasound_utils.h>

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

int side_phase = 90;

void setup() {
    // pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
    motor_begin();
    delay(2000);
    ultrasound_setup();
}

void loop() {
    stopped = stop_ticker();
    temp_distance = detected_mine(trigPinLeft, echoPinLeft);

    switch(step) {
        case 1:
            change_direction(side_phase);
            step++;
            break;
        case 2:
            if (!stopped) {break;}
            step++;
            set_move_forward_till(true);
            break;
        case 3:
            if (temp_distance >= 0){
                distance = temp_distance;
                start_get_to_mine();
                step++;
            }
            else if(move_forward_till_on()) {
                move_forward_till(20, 0.3, true);
            } else {
                side_phase -= 90;
                step = 1;
            }
            break;
        case 4:
            got_to_mine = get_to_mine(distance + 20, 0.5, stopped);
            if(got_to_mine) {
                start_move_to();
                got_to_mine = false;
                step++;
            }
            break;
        case 5:
            // do gripper stuff
            step++;
            break;
        case 6:
            got_to_safe_zone = go_to_safe_zone(1.0, true, stopped);
            if(got_to_safe_zone) {
                got_to_safe_zone = false;
                step++;
                start_move_to();
            }
            break;
        case 7:
            // do gripper stuff
            step++;
            break;
        case 8:
            got_to_base = return_to_base(1.0, true, stopped);
            if(got_to_base) {
                step = 1;
                got_to_base = false;
                side_phase = 90;
            }
            break;
    }
}
