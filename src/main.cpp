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

bool got_to_mine;
bool got_to_safe_zone;
bool got_to_base;

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

    if(stopped) {
        step = previous_step + 1;
        Serial.println("finished turning");
        set_move_forward_till(true);
    }

    switch(step) {
        case 1:
            Serial.println("turning");
            change_direction(90);
            step = -1;
            previous_step = 1;
            break;
        case 2:
            if (temp_distance >= 0){
                distance = temp_distance;
                set_move_forward_till(false);
                start_get_to_mine();
                step++;
            }
            else if(move_forward_till_on()) {
                move_forward_till(20, 0.8, true);
            } else {
                while(1) {};
            }
            break;
        case 3:
            got_to_mine = get_to_mine(distance, 1.0, stopped);
            if(got_to_mine) {
                Serial.println("go to step4");
                got_to_mine = false;
                step++;
            }
            break;
        case 4:
            got_to_safe_zone = go_to_safe_zone(0.8, true, stopped);
            if(got_to_safe_zone) {
                got_to_safe_zone = false;
                step++;
            }
            break;
        case 5:
            got_to_base = return_to_base(0.8, true, stopped);
            if(got_to_base) {
                got_to_base = false;
            }
            break;
    }
}
