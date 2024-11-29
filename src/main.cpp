#include <Arduino.h>
#include <Wire.h>
#include <Wire.h>                          // For I2C communication
#include <Adafruit_Sensor.h>               // For the unified sensor interface
#include <Adafruit_BME680.h>               // For the BME680 sensor
// #include <Adafruit_GFX.h>               // for a display that requires GFX library
#include "I2C_EXT.h"



AD5934 ad5934; // Instance of the class for calling non-static methods
Adafruit_BME680 bme; // global instance for the BME680 sensor

bool buttonPress = true; // Button press simulator - Initialize as true to avoid immediate trigger

void setup() {
    Wire.begin();
    Serial.begin(9600);
    delay(20000);  // Give time for Serial monitor to initialize. adjust per your needs
    Serial.println("Initializing AD5934...");

    if (!AD5934::setupAD5934(ad5934)) {
    Serial.println("Error: AD5934 setup failed");
    }

    Serial.println("Initializing BME680...");
    if (!setupBME680(bme)) {
        Serial.println("Error: BME680 setup failed.");
    }

    buttonPress = false; // Button press simulator - Set to false after setup is complete
}

void loop() {
    if (!buttonPress) {   
    Serial.println("Starting impedance measurement...");

    // Define arrays to store the real and imaginary parts of impedance
    int realData[NUM_INCREMENTS];
    int imagData[NUM_INCREMENTS];
    float magnitudeData[NUM_INCREMENTS];
    float phaseData[NUM_INCREMENTS];

    // Perform the frequency sweep
    if (!ad5934.frequencySweep(realData, imagData, NUM_INCREMENTS)) {
        return;
    }

    // Process and save the sweep data
    for (int i = 0; i < NUM_INCREMENTS; i++) {
        // Calculate magnitude and phase
        magnitudeData[i] = sqrt(pow(realData[i], 2) + pow(imagData[i], 2));
        phaseData[i] = atan2(imagData[i], realData[i]) * (180.0 / PI); // Phase in degrees //

        //Print the values for debugging
        //Serial.print("Point ");             //Comment out if need be
        //Serial.print(i + 1);                //Comment out if need be
        //Serial.print(" - Magnitude: ");     //Comment out if need be
        Serial.print(magnitudeData[i]);
        //Serial.print(", Phase: ");          //Comment out if need be
        Serial.println(phaseData[i]);
    }

    // Immediately after impedance sweep, read and display temperature and humidity
    Serial.println("Reading temperature and humidity...");
    readSensorData(bme); // Call the function to handle temperature and humidity readings

    // Prevent re-triggering
    buttonPress = true;
    }

    // Delay between measurements
    delay(30000); //adjust value per your needs

}
