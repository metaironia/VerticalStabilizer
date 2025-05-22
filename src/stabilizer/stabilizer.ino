#include "BMI160Gen.h"
#include "AccelStepper.h"
#include "Servo.h"

#include "stabilizer.h"

AccelStepper stepper(MOTOR_INTERFACE_TYPE, STEP_PIN, DIR_PIN);
Servo SERVO;

int initialAnglePosY = 0; 
int initialStepperPos = 0;

void GyroInterrupt (void) {

  noInterrupts();

  static unsigned long prevTimeCalled = 0;

  int   anglePosY         = BMI160.getRotationY();
  float filteredAnglePosY = ExpRunningAverageAdaptive (anglePosY);
 
  static int posX         = 0;
  int        accelX       = BMI160.getXAccelOffset();
  
  posX += accelX / (DEFAULT_GYRO_RATE * DEFAULT_GYRO_RATE) / 2;

  int   servoPosToWrite   = ComputePID (filteredAnglePosY, initialAnglePosY,
                                        SERVO_KP, SERVO_KI, SERVO_KD,
                                        1 / DEFAULT_GYRO_RATE, -1000, 1000); 

//  SERVO.write (servoPosToWrite);

#ifdef DEBUG_STAB
  PlotAnglePosY(anglePosY, millis());
  //PlotPosX(posX, millis());
#endif
  //int   stepperPosToWrite = ComputePID ()

  interrupts();
}

void PlotAnglePosY (float anglePosY, int currTime) {

  Serial.print("angle:");
  Serial.print(anglePosY);

  Serial.print(",");

  Serial.print("time:");
  Serial.println(currTime);
}

void PlotPosX (float posX, int currTime) {

  Serial.print("x:");
  Serial.print(posX);

  Serial.print(",");

  Serial.print("time:");
  Serial.println(currTime);
}

int ComputePID (float input, float setpoint, 
                float kp, float ki, float kd,
                float dt, int minOut, int maxOut) {
 
  float        err      = setpoint - input;
  static float integral = 0;
  static float prevErr  = 0;
  
  integral = constrain (integral + (float)err * dt * ki, minOut, maxOut);
 
  float D = (err - prevErr) / dt;
  prevErr = err;
 
  return constrain (err * kp + integral + D * kd, minOut, maxOut);
}

float ExpRunningAverageAdaptive (float newVal) {
  
  static float filVal = 0;
  float k = 0;
  
  k = (abs (newVal - filVal) > 1.5) ? 0.9 : 0.03;

  filVal += (newVal - filVal) * k;
  
  return filVal;
}

void SetInitialGyroOffset() {

  initialAnglePosY = 0;
}

void SetInitialStepperOffset() {

  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed (200);
  stepper.setAcceleration (100);

  stepper.move (300);
  stepper.setCurrentPosition (stepper.currentPosition());
}

void setup() {

  Serial.begin (9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  BMI160.begin (BMI160GenClass::SPI_MODE, GYRO_SELECT_PIN);
  BMI160.setGyroRate (DEFAULT_GYRO_RATE);

//  SERVO.attach (SERVO_PIN);  // Connect D6 of Arduino with PWM signal pin of SERVO motor

  SetInitialGyroOffset();
//  SetInitialStepperOffset();

  attachInterrupt (1, GyroInterrupt, CHANGE);

  //BMI160.attachInterrupt (GyroInterrupt);
}

void loop() {
  while(true);
}