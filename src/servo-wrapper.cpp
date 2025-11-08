#include "servo-wrapper.h"
#include <Servo.h>
ServoWrapper::ServoWrapper(int pin, float multiplier)
    : pin(pin), multiplier(multiplier), lastValue(0.0f) {
    servo = new Servo();
}

void ServoWrapper::begin() {
    servo->attach(pin);
}

void ServoWrapper::drive(float value) {
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;
    servo->writeMicroseconds(1500 + (int)(500.0f * value * multiplier));
    lastValue = value;
}
