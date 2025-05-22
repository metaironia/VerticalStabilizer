#include <BMI160Gen.h>

const int select_pin = 4;
const int i2c_addr = 0x68;

void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  BMI160.begin(BMI160GenClass::SPI_MODE, select_pin);
  //BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
}

void loop() {
  int gx = 0, gy = 0, gz = 0;         // raw gyro values

  // read raw gyro measurements from device
  gx = BMI160.getRotationX();

  // display tab-separated gyro x/y/z values
  Serial.print("g:\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.println();

  delay(50);
}