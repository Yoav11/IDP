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


void setup() {
    // pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
    motor_begin();
    delay(2000);
    set_move_forward_till(true);
    ultrasound_setup(trigPinLeft, echoPinLeft);
    ultrasound_setup(trigPinFront, echoPinFront);
}

void loop() {
    stopped = stop_ticker();
    distance = detected_mine(trigPinLeft, echoPinLeft);

    if(stopped) {
        step = previous_step + 1;
        Serial.println("finished turning");
    }

    switch(step) {
        case 1:
            Serial.println("turning");
            change_direction(90);
            step = -1;
            previous_step = 1;
            break;
        case 2:
            if(move_forward_till_on()) {
                Serial.println("moving forward");
                move_forward_till(20, 0.8, true);
            } else if (distance >= 0){
                step++;
            } else {
                step+=2;
            }
            break;
        // case 3:
        //     got_to_mine = get_to_mine(distance, 0.8, stopped)
        //     if(got_to_mine) {
        //         got_to_mine = false;
        //         step++;
        //     }
        //     break;
        // case 4:
        //     got_to_safe_zone = go_to_safe_zone(0.8, true, stopped)
        //     if(got_to_safe_zone) {
        //         got_to_safe_zone = false;
        //         step++;
        //     }
        //     break;
        // case 5:
        //     got_to_base = got_to_base(0.8, true, stopped)
        //     if(got_to_base) {
        //         got_to_base = false;
        //     }
        //     break;
    }
}
