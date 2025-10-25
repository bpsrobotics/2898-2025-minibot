#include <Arduino.h>
#include <Wire.h>
#include <servo-wrapper>
#include <math.h>

#define frontLeftMotorPin 5
#define backLeftMotorPin 6
#define backRightMotorPin 9
#define frontRightMotorPin 10


#define rightStickHorizontalPin 2
#define rightStickVerticalPin 3
#define intakePin 4


#define NOISE_THRESHOLD 50
#define CHANNEL_DEADZONE 50
#define CHANNEL_DEADZONE_CENTER 1500

#define FOUR_MOTORS true

#if FOUR_MOTORS
  ServoWrapper frontLeft(frontLeftMotorPin);
  ServoWrapper frontRight(frontRightMotorPin);
  ServoWrapper backLeft(backLeftMotorPin);
  ServoWrapper backRight(backRightMotorPin);
#else
  ServoWrapper leftMotor(frontLeftMotorPin);
  ServoWrapper rightMotor(frontRightMotorPin);
#endif

struct DriveValues { float left, right; };

struct channelInfo {
  uint8_t pin;
  bool wasOn;
  bool valueChanged;
  unsigned long lastHighTime;
  float value;
};

struct channels{
  volatile channelInfo rightStickHorizontal {rightStickHorizontalPin, false, false, 0, 0.0f};
  volatile channelInfo rightStickVertical {rightStickVerticalPin, false, false, 0, 0.0f};
  volatile channelInfo intake {intakePin, false, false, 0, 0.0f};
};

void updateChannel(volatile channelInfo& channel){
  const bool newState = digitalRead(channel.pin);
  const bool oldState = channel.wasOn;
  channel.valueChanged = false;
  if(newState != oldState) {
    const unsigned long now = micros();
    if (newState){
      channel.lastHighTime = now;
      channel.wasOn = true;
    } else{
      channel.wasOn = false;
      long delta = now - channel.lastHighTime;
      if(delta < NOISE_THRESHOLD) return;
      if(abs(delta - CHANNEL_DEADZONE_CENTER) <= CHANNEL_DEADZONE) delta = 1500;
      channel.value = (float)(delta - 1500) / 500.0f;
    }
  }
}

float sign(float x) {
  return x < 0 ? -1 : 1;
}

DriveValues driveTank(float go, float steer) {
  float denom = max(1, max(abs(go + steer), abs(go - steer)));
  return {
    (go + sign(go) * steer) / denom,
    (go - sign(go) * steer) / denom
  };
}

void setup() {
  Serial.(9600);
  pinMode(rightStickHorizontalPin, INPUT);
  pinMode(rightStickVerticalPin, INPUT);
  pinMode(intakePin, INPUT);
  attachInterrupt(digitalPinToInterrupt(rightStickHorizontalPin), []{updateChannel(channels.rightStickHorizontal); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightStickVerticalPin), []{updateChannel(channels.rightStickVertical); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(intakePin), []{updateChannel(channels.intake); }, CHANGE);

  #if FOUR_MOTORS
    frontLeft.begin();
    frontRight.begin();
    backLeft.begin();
    backRight.begin();
  #else
    leftMotor.begin();
    rightMotor.begin();
  #endif
}

void loop() {
    if (channels.switchA.value > 0.5f) {
      #if FOUR_MOTORS
        frontLeft.drive(0.0f);
        frontRight.drive(0.0f);
        backLeft.drive(0.0f);
        backRight.drive(0.0f);
      #else 
        rightMotor.drive(0.0f);
        leftMotor.drive(0.0f);
      #endif
        delay(2500);
        return;
    }

    vec2 stick = vec2(
        channels.rightStickHorizontal.value,
        channels.rightStickVertical.value
    );
    
    // Move motors
    DriveValues driveValues = driveTank(stick.x, stick.y)

    #if FOUR_MOTORS
      frontLeft.drive(driveValues.left);
      backLeft.drive(driveValues.left);
      frontRight.drive(driveValues.right);
      backRight.drive(driveValues.right);
    #else
      leftMotor.drive(driveValues.left);
      rightMotor.drive(driveValues.right);
    #endif
    // float v = channels.rightStickVertical.value;
    // frontLeft.drive(v);
    // frontRight.drive(v);
    // backLeft.drive(v);
    // backRight.drive(v);
    // Fans
    
    // Debugging assistance
    Serial.println(
        String("RH:") + channels.rightStickHorizontal.value
            + ", RV:" + channels.rightStickVertical.value
            + ", LH:" + channels.leftStickHorizontal.value
            + ", LV:" + channels.leftStickVertical.value
    );
}

