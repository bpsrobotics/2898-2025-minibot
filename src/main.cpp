#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "servo-wrapper.h"

#define frontLeftMotorPin 6
#define backLeftMotorPin 10 // Not used if using two motors
#define backRightMotorPin 4 // Not used if using two motors
#define frontRightMotorPin 5

#define rightStickHorizontalPin 9
#define rightStickVerticalPin 8
#define intakePin 10
#define eStopPin 11

#define leftPowerAdjPin 12
#define rightPowerAdjPin 13

#define MAX_COMPENSATION 1.0f // maximum fraction to reduce the opposite motor (0.0 - 1.0)

#define NOISE_THRESHOLD 50
#define CHANNEL_DEADZONE 50
#define CHANNEL_DEADZONE_CENTER 50
#define INTAKE_MOVED_TRHESHOLD 0.0f


#define FOUR_MOTORS false

ServoWrapper intake(intakePin);

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
  volatile ChannelInfo leftPowerAdj;
  volatile ChannelInfo rightPowerAdj;
};

// instantiate global channels with initial values
volatile Channels channels = {
  { rightStickHorizontalPin, false, false, 0, 0.0f, false },
  { rightStickVerticalPin, false, false, 0, 0.0f, false },
  { intakePin, false, false, 0, 0.0f, false },
  { eStopPin, false, false, 0, 0.0f, false },
  { leftPowerAdjPin, false, false, 0, 0.0f, false },
  { rightPowerAdjPin, false, false, 0, 0.0f, false }
};

// has the intake moved from the inital position?
volatile bool intakeMoved = false;

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
  pinMode(leftPowerAdjPin, INPUT);
  pinMode(rightPowerAdjPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(rightStickHorizontalPin), []{updateChannel(channels.rightStickHorizontal); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightStickVerticalPin), []{updateChannel(channels.rightStickVertical); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(intakePin), []{updateChannel(channels.intake); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(eStopPin), []{updateChannel(channels.eStop); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftPowerAdjPin), []{updateChannel(channels.leftPowerAdj); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightPowerAdjPin), []{updateChannel(channels.rightPowerAdj); }, CHANGE);

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
  processChannel(channels.leftPowerAdj);
  processChannel(channels.rightPowerAdj);

  

  // Check for emergency stop
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

  //read intake values
  float intakeSpeed = channels.intake.value;

  // if the intake has moved above the threshold, enable intake movement.
  if(intakeMoved == false){
    if(intakeSpeed >= INTAKE_MOVED_TRHESHOLD) intakeMoved = true;
  }


  // read stick values (vertical = forward/back, horizontal = steer)
  float go = channels.rightStickVertical.value;
  float steer = channels.rightStickHorizontal.value;

  // Move motors
  DriveValues driveValues = driveTank(go, steer);
  
  // Apply left/right compensation based on adjustment channels.
  float left = driveValues.left;
  float right = driveValues.right;
  float leftAdjNorm = (channels.leftPowerAdj.value + 1.0f) * 0.5f;
  float rightAdjNorm = (channels.rightPowerAdj.value + 1.0f) * 0.5f;
  
  // If left adjustment requested, make left relatively stronger by reducing right.
  if (leftAdjNorm > 0.01f) {
    float comp = leftAdjNorm * MAX_COMPENSATION; // 0..MAX_COMPENSATION
    right *= (1.0f - comp);
  }
  // If right adjustment requested, make right relatively stronger by reducing left.
  if (rightAdjNorm > 0.01f) {
    float comp = rightAdjNorm * MAX_COMPENSATION;
    left *= (1.0f - comp);
  }
  
  // Apply final drive values to motors

  if(intakeMoved == true){
    intake.drive(intakeSpeed);
  }
  
  #if FOUR_MOTORS
    frontLeft.drive(left);
    backLeft.drive(left);
    frontRight.drive(right);
    backRight.drive(right);
  #else
    leftMotor.drive(left);
    rightMotor.drive(right);
  #endif
}
