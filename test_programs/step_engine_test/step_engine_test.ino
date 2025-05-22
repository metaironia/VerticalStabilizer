#include "AccelStepper.h"

// Define stepper motor connections and motor interface type. 
// Motor interface type must be set to 1 when using a driver:
const int dirPin = 6;
const int stepPin = 5;
const int motorInterfaceType = 1;

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(200);
}

void loop() {
  // Set the target position:
  stepper.moveTo(-1000);
  // Run to target position with set speed and acceleration/deceleration:
  stepper.runToPosition();

  delay(1000);

  // Move back to zero:
  stepper.moveTo(-200);
  stepper.runToPosition();
}