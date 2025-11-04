#pragma once
#include <Servo.h>

class ServoWrapper {
    public:
        ServoWrapper(int pin, float multiplier = 1.0f) : pin(pin), multiplier(multiplier) {
            servo = new Servo();
        }

        void begin() {
            servo->attach(pin);
        }

        void drive(float value) {
            if(value > 1.0f) value = 1.0f;
            if(value < -1.0f) value = -1.0f;
            servo->writeMicroseconds(1500 + (int)(500 * value * multiplier));
            lastValue = value;
        }

    private:
        Servo* servo;
        float lastValue;
        float multiplier;
        int pin;
};
