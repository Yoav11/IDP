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

void setup() {
    // pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
    motor_begin();
    servo_setup();
    delay(2000);
    ultrasound_setup();
    // start_get_to_mine();
    // start_adjust_angle();
}

void loop() {


  /*
    stopped = stop_ticker();
    temp_distance = detected_mine(trigPinLeft, echoPinLeft);

    switch(step) {
        case 1:
            change_direction(90);
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
                step++;
            }
            else if(move_forward_till_on()) {
                move_forward_till(20, 0.2, true);
            } else {
                while(1) {};
            }
            break;
        case 4:
            got_to_mine = get_to_mine(distance + 20, 0.5, stopped);
            if(got_to_mine) {
                start_move_to();
                Serial.println("go to step4");
                got_to_mine = false;
                step++;
            }
            break;
        case 5:
            got_to_safe_zone = go_to_safe_zone(0.8, true, stopped);
            if(got_to_safe_zone) {
                got_to_safe_zone = false;
                step++;
                start_move_to();
            }
            break;
        case 6:
            got_to_base = return_to_base(0.8, true, stopped);
            if(got_to_base) {
                got_to_base = false;
            }
            break;
    }
    */
}
