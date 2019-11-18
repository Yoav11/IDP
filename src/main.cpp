#include <Arduino.h>
#include <motor_utils.h>
#include <led_utils.h>
#include <Ticker.h>
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
    delay(3000);

    set_move_forward_till(true);
    ultrasound_setup(trigPinFront, echoPinFront);
}

void loop() {
    stopped = stop_ticker();
    if(millis() - current_time >= 50) {
        current_time = millis();
        distance = get_distance(trigPinFront, echoPinFront);
    }

    if(stopped){
        switch(step){
            case 0:
                face_east();
                break;
            case 1:
                move_forward_till(30, 200);
                step++;
                break;
            case 2:
                face_north();
                break;
            case 3:
                move_forward_till(30, 200);
                step++;
                break;
            case 4:
                face_west();
                break;
            case 5:
                move_forward_till(30, 200);
                step++;
                break;
            case 6:
                face_south();
                break;
            case 7:
                move_forward_till(30, 200);
                step++;
                break;
        }
    }
    // move_forward(0.5);
}
