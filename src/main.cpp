#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "servo-wrapper.h"

#define frontLeftMotorPin 5
#define backLeftMotorPin 10
#define backRightMotorPin 4
#define frontRightMotorPin 6


#define rightStickHorizontalPin 9
#define rightStickVerticalPin 8
#define intakePin 10
#define eStopPin 12

#define NOISE_THRESHOLD 50
#define CHANNEL_DEADZONE 50
#define CHANNEL_DEADZONE_CENTER 50

#define FOUR_MOTORS false

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

struct ChannelInfo {
  uint8_t pin;
  bool wasOn;
  bool valueChanged;
  unsigned long lastHighTime;
  unsigned long lastPulseWidth;
  float value;
  bool invert;
};

struct Channels {
  volatile ChannelInfo rightStickHorizontal;
  volatile ChannelInfo rightStickVertical;
  volatile ChannelInfo intake;
  volatile ChannelInfo eStop;
};

// instantiate global channels with initial values
volatile Channels channels = {
  { rightStickHorizontalPin, false, false, 0, 0.0f, false },
  { rightStickVerticalPin, false, false, 0, 0.0f, false },
  { intakePin, false, false, 0, 0.0f, false },
  { eStopPin, false, false, 0, 0.0f, false }
};

void updateChannel(volatile ChannelInfo& channel) {
  // Keep ISR as short as possible: only record timestamps and pulse width
  const bool newState = digitalRead(channel.pin);
  const bool oldState = channel.wasOn;
  if (newState != oldState) {
    const unsigned long now = micros();
    if (newState) {
      // store rising time
      channel.lastHighTime = now;
      channel.wasOn = true;
    } else {
      // falling edge: compute pulse width and mark for main loop
      channel.wasOn = false;
      unsigned long delta = now - channel.lastHighTime;
      if (delta < NOISE_THRESHOLD) return;
      channel.lastPulseWidth = delta;
      channel.valueChanged = true;
    }
  }
}

// Atomically copy pulse data and convert to normalized float value in main loop
void processChannel(volatile ChannelInfo& channel) {
  if (!channel.valueChanged) return;
  // copy multi-byte data atomically
  noInterrupts();
  unsigned long pulse = channel.lastPulseWidth;
  channel.valueChanged = false;
  interrupts();

  long delta = (long)pulse;
  // apply deadzone and snapping
  if (delta >= 2000 - CHANNEL_DEADZONE) delta = 2000;
  if (delta <= 1000 + CHANNEL_DEADZONE) delta = 1000;
  if (abs(delta - 1500) <= CHANNEL_DEADZONE_CENTER) delta = 1500;
  // convert 1000 <= delta <= 2000 to -1.0 <= value <= 1.0
  channel.value = (float)(delta - 1500) / 500.0f;
  if (channel.invert) channel.value = -channel.value;
}

DriveValues driveTank(float go, float steer) {
  // Standard arcade-to-tank mixing:
  // left = go + steer, right = go - steer
  float left = go + steer;
  float right = go - steer;
  // scale both outputs if either exceeds magnitude 1.0
  float denom = fmax(1.0f, fmax(fabs(left), fabs(right)));
  return { left / denom, right / denom };
}

void setup() {
  Serial.begin(9600);
  pinMode(rightStickHorizontalPin, INPUT);
  pinMode(rightStickVerticalPin, INPUT);
  pinMode(intakePin, INPUT);
  pinMode(eStopPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(rightStickHorizontalPin), []{updateChannel(channels.rightStickHorizontal); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightStickVerticalPin), []{updateChannel(channels.rightStickVertical); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(intakePin), []{updateChannel(channels.intake); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(eStopPin), []{updateChannel(channels.eStop); }, CHANGE);

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
  // First handle any newly-measured pulses from ISRs
  processChannel(channels.rightStickHorizontal);
  processChannel(channels.rightStickVertical);
  processChannel(channels.intake);
  processChannel(channels.eStop);

    if (channels.eStop.value > 0.5f) {
      // E-Stop engaged, stop all motors
      #if FOUR_MOTORS
        frontLeft.drive(0.0f);
        backLeft.drive(0.0f);
        frontRight.drive(0.0f);
        backRight.drive(0.0f);
      #else
        leftMotor.drive(0.0f);
        rightMotor.drive(0.0f);
      #endif
      return;
    }

    // read stick values (vertical = forward/back, horizontal = steer)
    float go = channels.rightStickVertical.value;
    float steer = channels.rightStickHorizontal.value;

    // Move motors
    DriveValues driveValues = driveTank(go, steer);

    #if FOUR_MOTORS
      frontLeft.drive(driveValues.left);
      backLeft.drive(driveValues.left);
      frontRight.drive(driveValues.right);
      backRight.drive(driveValues.right);
    #else
      leftMotor.drive(driveValues.left);
      rightMotor.drive(driveValues.right);
    #endif
}
