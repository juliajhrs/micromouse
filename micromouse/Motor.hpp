#pragma once

#include <Arduino.h>
#include "math.h"

namespace mtrn3100 {

class Motor {
public:
    // Constructor
    Motor(uint8_t in1, uint8_t in2) :  pwm_pin_(in1), dir_pin_(in2) {
        pinMode(pwm_pin_, OUTPUT);
        pinMode(dir_pin_, OUTPUT);
    }

    // This function outputs the desired motor direction and the PWM signal. 
    // NOTE: a pwm signal > 255 could cause troubles as such ensure that pwm is clamped between 0 - 255.
    void setPWM(int16_t pwm) {
        if (pwm > 0) {
            digitalWrite(dir_pin_, LOW);
            analogWrite(pwm_pin_, min(pwm, 255));
        } else {
            digitalWrite(dir_pin_, HIGH);
            analogWrite(pwm_pin_, min(-pwm, 255));
        }
    } 

private:
    const uint8_t pwm_pin_;
    const uint8_t dir_pin_;
};

} // namespace mtrn3100