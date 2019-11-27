#include <motor_utils.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *left_motor = AFMS.getMotor(1);
Adafruit_DCMotor *right_motor = AFMS.getMotor(2);
bool has_to_stop = false;
float time_to_stop = -1;
float stop_tick;
int routine_step = -1;
uint16_t old_speeds[2] = {0, 0}; // Stores last speed for each motor
uint16_t old_direcs[2] = {FORWARD, BACKWARD}; // Stores last moving direction for each motor

void motor_begin() {
    AFMS.begin();
}

void motor_run(int motor_index, uint16_t speed, uint16_t direction) {
    Adafruit_DCMotor *motor = motor_index == 1 ? left_motor : right_motor;

    // if (old_speeds[motor_index-1] == speed && old_direcs[motor_index-1] == direction) {
    //   return; // If the new instruction is exactly the same as the last one, don't send any commands and just return.
    // }

    // The below will run if the new instruction is different.
    motor->setSpeed(speed);
    motor->run(direction);
    old_speeds[motor_index-1] = speed;
    old_direcs[motor_index-1] = direction;
}

void motor_stop() {
    left_motor->setSpeed(0);
    right_motor->setSpeed(0);
}

void motor_turn(float angle) {
    if(angle >= 0){
        time_to_stop = map(angle, 0, 360, 0, 12250);
        motor_run(1, 100, FORWARD);
        motor_run(2, 100, FORWARD);
    } else {
        time_to_stop = map(angle, -360, 0, 12250, 0);
        motor_run(1, 100, BACKWARD);
        motor_run(2, 100, BACKWARD);
    }
    has_to_stop = true;
}

bool stop_ticker() {
    if(has_to_stop) {
        stop_tick = millis();
        has_to_stop = false;
    }
    if(millis()-stop_tick >= time_to_stop && time_to_stop >= 0) {
        time_to_stop = -1;
        motor_stop();
        routine_step++;
        return true;
    }
    return false;
}

void square_test() {
    switch(routine_step) {
        case 0:
            motor_turn(90);
            Serial.println(routine_step);
            break;
        case 1:
            motor_run(1, 200, FORWARD);
            motor_run(2, 200, BACKWARD);
            has_to_stop = true;
            time_to_stop = 6000;
            Serial.println(routine_step);
            break;
        case 2:
            motor_turn(-90);
            Serial.println(routine_step);
            break;
        case 3:
            motor_run(1, 200, FORWARD);
            motor_run(2, 200, BACKWARD);
            has_to_stop = true;
            time_to_stop = 6000;
            Serial.println(routine_step);
            break;
        case 4:
            motor_turn(-90);
            Serial.println(routine_step);
            break;
        case 5:
            motor_run(1, 200, FORWARD);
            motor_run(2, 200, BACKWARD);
            has_to_stop = true;
            time_to_stop = 6000;
            Serial.println(routine_step);
            break;
        case 6:
            motor_turn(-90);
            Serial.println(routine_step);
            break;
        case 7:
            motor_run(1, 200, FORWARD);
            motor_run(2, 200, BACKWARD);
            has_to_stop = true;
            time_to_stop = 6000;
            Serial.println(routine_step);
            break;
    }

}
