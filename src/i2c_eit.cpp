#include "i2c_eit.h"
#include <Arduino.h>  // Required for Arduino-specific functions (like delay, Serial)

void setupAD5934() {
    // Example start frequency, adjust as needed (3-byte start frequency value)
    long startFrequency = 0x123456;
    setRegister(START_FREQ_REG, startFrequency, 3);  // Write 3-byte start frequency
}

void setRegister(uint16_t address, long value, int len) {
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

void readImpedance() {
    int16_t real = readRegister(REAL_DATA_REG, 2);  // Read 2 bytes from real part register
    int16_t imag = readRegister(IMAG_DATA_REG, 2);  // Read 2 bytes from imaginary part register

    // Calculate the magnitude and phase of the impedance
    float magnitude = sqrt(pow(real, 2) + pow(imag, 2));
    float phase = atan2(imag, real);

    // Print the results for debugging
    Serial.print("Magnitude: ");
    Serial.println(magnitude);
    Serial.print("Phase: ");
    Serial.println(phase);
}

int16_t readRegister(uint16_t address, int len) {
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
