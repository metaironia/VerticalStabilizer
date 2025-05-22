#ifndef STABILIZER_H
#define STABILIZER_H

#define DEBUG_STAB

// Define stepper motor connections and motor interface type. 
// Motor interface type must be set to 1 when using a driver:
const int   DIR_PIN = 6;
const int   STEP_PIN = 5;
const int   MOTOR_INTERFACE_TYPE = 1;

const int   SERVO_PIN = 7; // A descriptive name for D6 pin of Arduino to provide PWM signal

const int   GYRO_SELECT_PIN = 4;
const int   DEFAULT_GYRO_RATE = BMI160_GYRO_RATE_400HZ;

const float SERVO_KP = 1;
const float SERVO_KI = 1;
const float SERVO_KD = 1;

void GyroInterrupt (void);

void PlotAnglePosY (float anglePosY, int currTime);

void PlotPosX (float posX, int currTime);

int ComputePID (float input, float setpoint,
                float kp, float ki, float kd,
                float dt, int minOut, int maxOut);

float ExpRunningAverageAdaptive (float newVal);

void SetInitialGyroOffset();

void SetInitialStepperOffset();

#endif