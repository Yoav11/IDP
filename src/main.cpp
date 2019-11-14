#include <Arduino.h>
#include <motor_utils.h>
#include <led_utils.h>

int old_routine_step;
bool stopped = true;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
    motor_begin();
    delay(3000);
}

void loop() {
    stopped = stop_ticker();
    if(stopped){
        delay(2000);
        square_test();
}
