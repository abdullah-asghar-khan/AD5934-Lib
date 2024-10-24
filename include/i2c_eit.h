#ifndef I2C_EIT_H
#define I2C_EIT_H

#include <Wire.h> //missing this!!

#define AD5934_Address 0x0D  // I2C address of the AD5934
#define START_FREQ_REG 0x82  // Start frequency register (16-bit)
#define REAL_DATA_REG  0x94  // Real data register (16-bit)
#define IMAG_DATA_REG  0x96  // Imaginary data register (16-bit)
#define CTRL_REG       0x80  // Control register

void setupAD5934();
void readImpedance();
void setRegister(uint8_t deviceAddress, uint16_t registerAddress, long value, int len);
int16_t readRegister(uint8_t deviceAddress, uint16_t registerAddress, int len);

#endif 
