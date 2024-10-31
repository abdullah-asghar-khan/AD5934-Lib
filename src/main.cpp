#include <Arduino.h>
#include <Wire.h>
#include "I2C_EXT.h"
#include "ImpedanceFuncs.h"

// put function declarations here:
//int myFunction(int, int);

void setup() //setup code
{
  Wire.begin();        //Initialize I2C
  //Serial.begin(9600);  // Initialize serial communication with Raspberry Pi
  setupAD5934();       // Setting up the sensor (IC)
}

void loop() //code that repeats
{
  //Start sweeping from standby
  setRegister(AD5934_Address, CTRL_REG, 0xA000, 2);  // Initial sweep command

  readImpedance();

  //Return to standby after finishing
  setRegister(AD5934_Address, CTRL_REG, 0xB000, 2);  // Standby mode command
  //setRegister(AD5934_Address, CTRL_REG, 0xA004, 2);  // Repeat frequency Sweep command
  delay(5000);
}
