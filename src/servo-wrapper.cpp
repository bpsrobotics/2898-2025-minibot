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
            if(value > 1.0) value = 1.0;
            if(value < -1.0) value = -1.0;
            servo->writeMicroSeconds(1500 + 500 * value * multiplier);
            lastValue = value;
        }

        private:
            Servo* servo;
            float lastValue, multiplier;
            int pin;

}