#include <Arduino.h>
#include <motor_utils.h>
#include <led_utils.h>
#include <navigation_utils.h>
#include <ultrasound_utils.h>

int old_routine_step;
bool stopped = true;


void setup() {
    // pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
    motor_begin();
    delay(3000);

    set_move_forward_till(true);
    ultrasound_setup(trigPinFront, echoPinFront);
}

void loop() {
    // stopped = stop_ticker();
    // if(stopped){
    //     delay(2000);
    //     square_test();
    // }

    if (move_forward_till_on()) {
      move_forward_till(5, 0.2);
    }
    // move_forward(0.5);
}
