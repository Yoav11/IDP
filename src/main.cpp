#include <Arduino.h>
#include <motor_utils.h>
#include <led_utils.h>
#include <navigation_utils.h>
#include <ultrasound_utils.h>

int old_routine_step;
bool stopped = true;
int phase = 0;
int step = 0;
int detected_distance;
float current_time;
float distance;


void setup() {
    // pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
    motor_begin();
    delay(2000);
    set_move_forward_till(true);
    ultrasound_setup(trigPinFront, echoPinFront);
}

void loop() {
    stopped = stop_ticker();

    detected_distance = detected_mine(trigPinFront, echoPinFront);
    if(stopped) {
        Serial.println("finished turn");
        step = 0;
    }

    switch(step){
        case 0:
            if(detected_distance >= 0) {
                step++;
                Serial.println("mine detected !");
            }
            else if (move_forward_till_on()) {
              move_forward_till(20, 1.0);
            } else {
                step++;
                set_move_forward_till(true);
                Serial.println("start turn");
            }
            break;
        case 1:
            change_direction(0+phase);
            phase-= 90;
            step = -1;
            break;
    }
}
