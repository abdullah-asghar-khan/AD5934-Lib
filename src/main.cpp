#include <Arduino.h>
#include <Wire.h>
#include "i2c_eit.h"

// put function declarations here:
//int myFunction(int, int);

void setup() //setup code
{
  Wire.begin();        //Initialize I2C
  Serial.begin(9600);  // Initialize serial communication with Raspberry Pi
  setupAD5934();       // Setting up the sensor (IC)
}

void loop() //code that repeats
{
  readImpedance();
  delay(1000);// put your main code here, to run repeatedly:
}
