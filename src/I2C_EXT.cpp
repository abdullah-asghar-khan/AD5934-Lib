#include "I2C_EXT.h"
#include <Arduino.h>  // Required for Arduino-specific functions (like delay, Serial)

void setupAD5934() {
    long startFrequency = 0x189CF;  // should be around 12 (0x189CF) - 13 (0x1A9CC) KHz
    setRegister(AD5934_Address, START_FREQ_REG, startFrequency, 3);  // Set start frequency (3 bytes)

    int numIncrements = 511;  // Maximum number of increments (9 bits)
    setRegister(AD5934_Address, NUM_INCR_REG, numIncrements, 2);  // Set number of increments (2 bytes)

    long incrementFrequency = 0x00053;  // Set increment frequency value as 0x00053 - DOUBLECHECK THIS
    setRegister(AD5934_Address, INCR_FREQ_REG, incrementFrequency, 3);  // Set frequency increment (3 bytes)

    // Place AD5934 into Standby Mode
    setRegister(AD5934_Address, CTRL_REG, 0xB000, 2);  // Standby mode command
}

void setRegister(uint8_t deviceAddress, uint16_t address, long value, int len) {
    Wire.beginTransmission(AD5934_Address);
    Wire.write((address >> 8) & 0xFF);  // Send upper byte of 16-bit register address
    Wire.write(address & 0xFF);         // Send lower byte of 16-bit register address

    for (int i = len - 1; i >= 0; i--) {
        Wire.write((byte)((value >> (i * 8)) & 0xFF));  // Write data bytes (1, 2, or 3 bytes)
    }

    byte err = Wire.endTransmission();
    if (err != 0) {
        Serial.println("Error writing to AD5934");
    }
}

int16_t readRegister(uint8_t deviceAddress, uint16_t address, int len) {
    Wire.beginTransmission(AD5934_Address);
    Wire.write((address >> 8) & 0xFF);  // Send upper byte of 16-bit register address
    Wire.write(address & 0xFF);         // Send lower byte of 16-bit register address

    byte err = Wire.endTransmission(false);  // Keep the I2C bus alive
    if (err != 0) {
        Serial.println("Error setting register address");
        return 0;
    }

    Wire.requestFrom(AD5934_Address, len);
    int16_t result = 0;

    if (Wire.available() == len) {
        for (int i = 0; i < len; i++) {
            result = (result << 8) | Wire.read();  // Combine received bytes into a single value
        }
    } else {
        Serial.println("Error: Not enough data received");
    }

    return result;
}
