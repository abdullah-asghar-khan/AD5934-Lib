/**
 * @file I2C_EXT.cpp
 * @brief Library code for AD5934
 */

#include "I2C_EXT.h"
#include <Math.h>

/**
 * Request to read a byte from the AD5934.
 *
 * @param address Address of register requesting data from
 * @param value Pointer to a byte where the return value should be stored, or
 *        where the error code will be stored if fail.
 * @return Success or failure
 */
int AD5934::getByte(byte address, byte *value) {
    Wire.beginTransmission(AD5934_ADDR);
    Wire.write(ADDR_PTR);
    Wire.write(address);

    if (byte res = Wire.endTransmission() != I2C_RESULT_SUCCESS) {
        *value = res;
        return false;
    }

    Wire.requestFrom(AD5934_ADDR, 1);
    if (Wire.available()) {
        *value = Wire.read();
        return true;
    } else {
        *value = 0;
        return false;
    }
}

/**
 * Write a byte to a register on the AD5934.
 *
 * @param address The register address to write to
 * @param value The byte to write to the address
 * @return Success or failure of transmission
 */
bool AD5934::sendByte(byte address, byte value) {
    Wire.beginTransmission(AD5934_ADDR);
    Wire.write(address);
    Wire.write(value);

    if (Wire.endTransmission() != I2C_RESULT_SUCCESS) {
        return false;
    } else {
        return true;
    }
}

bool AD5934::setControlMode(byte mode) {
    byte val;
    if (!getByte(CTRL_REG1, &val))
        return false;

    // Clear the top 4 bits (mode bits) and set new mode
    val &= 0x0F;

    // Set the top 4 bits appropriately
    val |= mode;

    return sendByte(CTRL_REG1, val);
}

/**
 * Reset the AD5934.
 *
 * @return Success or failure
 */
bool AD5934::reset() {
    byte val;
    if (!getByte(CTRL_REG2, &val))
        return false;

    val |= CTRL_RESET;
    return sendByte(CTRL_REG2, val);
}

/**
 * Set the external clock as required for AD5934.
 *
 * @return Success or failure
 */
bool AD5934::setExternalClock() {
    return sendByte(CTRL_REG2, CLOCK_EXTERNAL);
}

/**
 * Set the settling time cycles use for frequency sweep.
 *
 * @param time The settling time cycles to set.
 * @return Success or failure
 */
bool AD5934::setSettlingCycles(int time) {
    int cycles;
    byte settleTime[2], rsTime[2], val;

    settleTime[0] = time & 0xFF;
    settleTime[1] = (time >> 8) & 0xFF;

    cycles = (settleTime[0] | (settleTime[1] & 0x1));
    val = (byte)((settleTime[1] & 0x7) >> 1);

    if ((cycles > 0x1FF) || !(val == 0 || val == 1 || val == 3)) {
        return false;
    }

    if (sendByte(NUM_SCYCLES_1, settleTime[1]) && sendByte(NUM_SCYCLES_2, settleTime[0])) {
        if (getByte(NUM_SCYCLES_1, &rsTime[1]) && getByte(NUM_SCYCLES_2, &rsTime[0])) {
            if ((settleTime[0] == rsTime[0]) && (settleTime[1] == rsTime[1])) {
                return true;
            }
        }
    }
    return false;
}

/**
 * Set the start frequency for a frequency sweep.
 *
 * @param start The initial frequency.
 * @return Success or failure
 */
bool AD5934::setStartFrequency(unsigned long start) {
    long freqHex = (start / (16e6 / 16)) * pow(2, 27);
    if (freqHex > 0xFFFFFF) {
        return false;
    }

    byte highByte = (freqHex >> 16) & 0xFF;
    byte midByte = (freqHex >> 8) & 0xFF;
    byte lowByte = freqHex & 0xFF;

    return sendByte(START_FREQ_1, highByte) &&
           sendByte(START_FREQ_2, midByte) &&
           sendByte(START_FREQ_3, lowByte);
}

/**
 * Set the increment frequency for a frequency sweep.
 *
 * @param increment The frequency to increment by, 0x430.
 * @return Success or failure
 */
bool AD5934::setIncrementFrequency(unsigned long increment) {
    long freqHex = (increment / (16e6 / 16)) * pow(2, 27);
    if (freqHex > 0xFFFFFF) {
        return false;
    }

    byte highByte = (freqHex >> 16) & 0xFF;
    byte midByte = (freqHex >> 8) & 0xFF;
    byte lowByte = freqHex & 0xFF;

    return sendByte(INC_FREQ_1, highByte) &&
           sendByte(INC_FREQ_2, midByte) &&
           sendByte(INC_FREQ_3, lowByte);
}

/**
 * Set the number of frequency increments for a frequency sweep.
 *
 * @param num The number of increments.
 * @return Success or failure
 */
bool AD5934::setNumberIncrements(unsigned int num) {
    if (num > 511) {
        return false;
    }

    byte highByte = (num >> 8) & 0xFF;
    byte lowByte = num & 0xFF;

    return sendByte(NUM_INC_1, highByte) &&
           sendByte(NUM_INC_2, lowByte);
}

/**
 * Set the PGA gain factor.
 *
 * @param gain The gain factor to select. Use constants or 1/5.
 * @return Success or failure
 */
bool AD5934::setPGAGain(byte gain) {
    byte val;
    if (!getByte(CTRL_REG1, &val))
        return false;

    val &= 0xFE;

    if (gain == PGA_GAIN_X1 || gain == 1) {
        val |= PGA_GAIN_X1;
        return sendByte(CTRL_REG1, val);
    } else if (gain == PGA_GAIN_X5 || gain == 5) {
        val |= PGA_GAIN_X5;
        return sendByte(CTRL_REG1, val);
    } else {
        return false;
    }
}

/**
 * Read the value of a register.
 *
 * @param reg The address of the register to read.
 * @return The value of the register. Returns 0xFF if can't read it.
 */
byte AD5934::readRegister(byte reg) {
    byte val;
    if (getByte(reg, &val)) {
        return val;
    } else {
        return STATUS_ERROR;
    }
}

bool AD5934::setRange(byte range) {
    byte val;

    if (!getByte(CTRL_REG1, &val))
        return false;

    // Clear bits D9 and D10 (voltage range bits)
    val &= 0xF9;

    // Set range bits based on the selected range
    switch (range) {
        case CTRL_OUTPUT_RANGE_2:
            val |= CTRL_OUTPUT_RANGE_2;
            break;
        case CTRL_OUTPUT_RANGE_3:
            val |= CTRL_OUTPUT_RANGE_3;
            break;
        case CTRL_OUTPUT_RANGE_4:
            val |= CTRL_OUTPUT_RANGE_4;
            break;
        default:
            val |= CTRL_OUTPUT_RANGE_1;
            break;
    }

    return sendByte(CTRL_REG1, val);
}

/**
 * Read the value of the status register.
 *
 * @return The value of the status register. Returns 0xFF if can't read it.
 */
byte AD5934::readStatusRegister() {
    return readRegister(STATUS_REG);
}

/**
 * Read the value of the control register.
 *
 * @return The value of the control register. Returns 0xFFFF if can't read it.
 */
int AD5934::readControlRegister() {
    return ((readRegister(CTRL_REG1) << 8) | readRegister(CTRL_REG2)) & 0xFFFF;
}

/**
 * Get a raw complex number for a specific frequency measurement.
 *
 * @param real Pointer to an int that will contain the real component.
 * @param imag Pointer to an int that will contain the imaginary component.
 * @return Success or failure
 */
bool AD5934::getComplexData(int *real, int *imag) {
    // Wait for a measurement to be available
    while ((readStatusRegister() & STATUS_DATA_VALID) != STATUS_DATA_VALID);

    byte realComp[2];
    byte imagComp[2];
    if (getByte(REAL_DATA_1, &realComp[0]) &&
        getByte(REAL_DATA_2, &realComp[1]) &&
        getByte(IMAG_DATA_1, &imagComp[0]) &&
        getByte(IMAG_DATA_2, &imagComp[1])) {
        // Combine the two separate bytes into a single 16-bit value and store
        // them at the locations specified.
        *real = (int16_t)(((realComp[0] << 8) | realComp[1]) & 0xFFFF);
        *imag = (int16_t)(((imagComp[0] << 8) | imagComp[1]) & 0xFFFF);
        return true;
    } else {
        *real = -1;
        *imag = -1;
        return false;
    }
}

/**
 * Set the power level of the AD5934.
 *
 * @param level The power level to choose.
 * @return Success or failure
 */
bool AD5934::setPowerMode(byte level) {
    switch (level) {
        case POWER_ON:
            return setControlMode(CTRL_NO_OPERATION);
        case POWER_STANDBY:
            return setControlMode(CTRL_STANDBY_MODE);
        case POWER_DOWN:
            return setControlMode(CTRL_POWER_DOWN_MODE);
        default:
            return false;
    }
}

bool AD5934::frequencySweep(int real[], int imag[], int n) {
    // Initialize sweep
    if (!(setPowerMode(POWER_STANDBY) &&
          setControlMode(CTRL_INIT_START_FREQ) &&
          setControlMode(CTRL_START_FREQ_SWEEP))) {
        return false;
    }

    // Perform sweep
    int i = 0;
    while ((readStatusRegister() & STATUS_SWEEP_DONE) != STATUS_SWEEP_DONE) {
        if (i >= n) return false;

        // Wait for valid data
        while ((readStatusRegister() & STATUS_DATA_VALID) != STATUS_DATA_VALID) {
            delay(10); // Prevent overloading the I2C bus
        }

        // Retrieve impedance data
        if (!getComplexData(&real[i], &imag[i])) return false;



        // Increment frequency
        if (!setControlMode(CTRL_INCREMENT_FREQ)) return false;

        i++;
    }
    delay(50); // Allow the device to stabilize
    return setPowerMode(POWER_STANDBY); // Put device into standby
}


/**
 * Computes the gain factor and phase for each point in a frequency sweep.
 *
 * @param gain An array of appropriate size to hold the gain factors
 * @param phase An array of appropriate size to hold phase data.
 * @param ref The known reference resistance.
 * @param n Length of the array (or the number of discrete measurements)
 * @return Success or failure
 */
bool AD5934::calibrate(double gain[], int phase[], int ref, int n) {
    int *real = new int[n];
    int *imag = new int[n];

    if (!frequencySweep(real, imag, n)) {
        delete[] real;
        delete[] imag;
        return false;
    }

    for (int i = 0; i < n; i++) {
        gain[i] = (double)(1.0 / ref) / sqrt(pow(real[i], 2) + pow(imag[i], 2));
        phase[i] = atan2(imag[i], real[i]) * (180.0 / M_PI);  // Convert to degrees if needed

    }

    delete[] real;
    delete[] imag;
    return true;
}

/**
 * Computes the gain factor and phase for each point in a frequency sweep.
 * Also provides the caller with the real and imaginary data.
 *
 * @param gain An array of appropriate size to hold the gain factors
 * @param phase An array of appropriate size to hold the phase data
 * @param real An array of appropriate size to hold the real data
 * @param imag An array of appropriate size to hold the imaginary data.
 * @param ref The known reference resistance.
 * @param n Length of the array (or the number of discrete measurements)
 * @return Success or failure
 */
bool AD5934::calibrate(double gain[], int phase[], int real[], int imag[], int ref, int n) {
    if (!frequencySweep(real, imag, n)) {
        return false;
    }

    for (int i = 0; i < n; i++) {
        gain[i] = (double)(1.0 / ref) / sqrt(pow(real[i], 2) + pow(imag[i], 2));
        phase[i] = atan2(imag[i], real[i]) * (180.0 / M_PI);  // Always convert phase to degrees
    }

    return true;
}

bool AD5934::setupAD5934(AD5934 &ad5934) {
    if (!(setExternalClock() &&
          setStartFrequency(START_FREQUENCY) &&
          setNumberIncrements(NUM_INCREMENTS) &&
          setIncrementFrequency(STEP_SIZE) &&
          ad5934.setRange(CTRL_OUTPUT_RANGE_1))) {  // Using the instance here
        return false;
    }

    // Configure settling cycles
    if (!ad5934.setSettlingCycles(SETTLING_CYCLES)) {  // Using the instance here
        return false;
    } 

    // Set device to Standby Mode and initialize start frequency
    if (!(setControlMode(CTRL_STANDBY_MODE) &&
          setControlMode(CTRL_INIT_START_FREQ))) {
        return false;
    }

    return true;
}
