#include <Arduino.h>
#include <motor_utils.h>
#include <led_utils.h>
#include <navigation_utils.h>
#include <ultrasound_utils.h>

int old_routine_step;
bool stopped = true;
int step = 0;
float current_time;
float distance;


void setup() {
    // pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
    motor_begin();
    delay(1000);
    set_move_forward_till(true);
    ultrasound_setup(trigPinFront, echoPinFront);
}

void loop() {
    stopped = stop_ticker();

    switch(step){
        case 0:
            if (move_forward_till_on()) {
              move_forward_till(5, 1.0);
            } else {
                step++;
            }
            break;
        case 1:
            change_direction(0);
            break;
    }
}
