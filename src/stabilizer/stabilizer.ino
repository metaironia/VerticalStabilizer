#include "BMI160Gen.h"
#include "AccelStepper.h"
#include "Servo.h"

#include "stabilizer.h"

AccelStepper stepper(MOTOR_INTERFACE_TYPE, STEP_PIN, DIR_PIN);
Servo SERVO;

int initialAnglePosY = 0; 
int initialStepperPos = 0;

volatile float anglePosY = 0;

void GyroInterrupt (void) {

  noInterrupts();

  int   rawAngleVelocityY       = BMI160.getRotationY();
  float filteredAngleVelocityY  = ExpRunningAverageAdaptive (rawAngleVelocityY);
  float trueAngleVelocityY      = filteredAngleVelocityY / BANANI_V_DEGREE;

  anglePosY += (trueAngleVelocityY / DEFAULT_GYRO_RATE_IN_HZ) * 2; // don't ask me why 2 is here...

#ifdef DEBUG_STAB
  Serial.print("angleVelocityY:");
  Serial.print(trueAngleVelocityY);

  Serial.print(",");

  Serial.print("filteredAngleVelocityY:");
  Serial.print(filteredAngleVelocityY);

  Serial.print(",");

  Serial.print("anglePosY:");
  Serial.println(anglePosY);
#endif

  static int   posX       = 0;
  
  //posX                        += accelX / (DEFAULT_GYRO_RATE * DEFAULT_GYRO_RATE) / 2;

  int        servoPosToWrite    = ComputeServoPID (anglePosY, initialAnglePosY); 

#ifdef DEBUG_STAB
  Serial.print("servoPosToWrite:");
  Serial.println(servoPosToWrite);
#endif

//  static int servoCounter;
//  if (servoCounter < 1000)
//    servoCounter++;
  SERVO.write (servoPosToWrite * 100);

//int   stepperPosToWrite = ComputePID ()

  interrupts();
}

void PlotAnglePosY (float anglePosY, int currTime) {


}
/*
void PlotPosX (float posX, int currTime) {

  Serial.print("x:");
  Serial.print(posX);

  Serial.print(",");

  Serial.print("time:");
  Serial.println(currTime);
}
*/
int ComputeServoPID (float input, float setpoint) {
 
  float        err      = setpoint - input;
  static float integral = 0;
  static float prevErr  = 0;
  
  integral = constrain (integral + (float)err * SERVO_KI / DEFAULT_GYRO_RATE_IN_HZ, -1000, 1000);
 
  float D = (err - prevErr) * DEFAULT_GYRO_RATE_IN_HZ;
  prevErr = err;
 
  return constrain (err * SERVO_KP + integral + D * SERVO_KD, -1000, 1000);
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

  BMI160.autoCalibrateGyroOffset();

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
  BMI160.setAccelRate (DEFAULT_ACCEL_RATE);
  BMI160.setFullScaleGyroRange (DEFAULT_GYRO_RANGE);
  SERVO.attach (SERVO_PIN);  // Connect D6 of Arduino with PWM signal pin of SERVO motor

  SetInitialGyroOffset();
//  SetInitialStepperOffset();

  attachInterrupt (1, GyroInterrupt, CHANGE);

  //BMI160.attachInterrupt (GyroInterrupt);
}

void loop() {
  while(true);
}