#include "ImpedanceFuncs.h"
#include <Arduino.h>  // Required for Arduino-specific functions (like delay, Serial)
#include <I2C_EXT.h>

float gain_factor = 0.000026;

void readImpedance() {
    int16_t real = readRegister(AD5934_Address, REAL_DATA_REG, 2);  // Read 2 bytes from real part register 
    int16_t imag = readRegister(AD5934_Address,IMAG_DATA_REG, 2);  // Read 2 bytes from imaginary part register 

    // Calculate the magnitude and phase of the impedance
    float magnitude = sqrt(pow(real, 2) + pow(imag, 2));
    float phase = atan2(imag, real);
    float impedance = 1/(gain_factor * magnitude);

    

    // Print the results for debugging
    Serial.print("Magnitude: ");
    Serial.println(magnitude);
    Serial.print("Phase: ");
    Serial.println(phase);
    Serial.print("Impedance: ");
    Serial.println(impedance);
    
}
