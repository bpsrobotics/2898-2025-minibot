#ifndef SERVO_WRAPPER_H
#define SERVO_WRAPPER_H

class Servo;

class ServoWrapper {
  public:
    ServoWrapper(int pin, float multiplier = 1.0f);
    void begin();
    void drive(float value);

  private:
    Servo* servo;
    float lastValue;
    float multiplier;
    int pin;
};

#endif // SERVO_WRAPPER_H
