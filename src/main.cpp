#include <Arduino.h>
#include <Wire.h>                          // For I2C communication
#include <Adafruit_Sensor.h>               // For the unified sensor interface
#include <Adafruit_BME680.h>               // For the BME680 sensor
//#include <Adafruit_GFX.h>               // for a display that requires GFX library
#include "I2C_EXT.h"
#include "TCA9548.h"                       //Multiplexer TCA9548a library

//Multiplexer instance
TCA9548 multiplexer(0x70); // Address for TCA9548 (default)

//AD5934 instances
AD5934 ad5934; //IC instance

//BME680 instance
Adafruit_BME680 bme; 

// Arrays to store data for all devices
float magnitudeData[3][NUM_INCREMENTS]; // 3 devices, NUM_INCREMENTS points per sweep
float phaseData[3][NUM_INCREMENTS];

bool buttonPress = true; // Button press simulator - Initialize as true to avoid immediate trigger


void setup() {
    Wire.begin();
    Serial.begin(9600);
    delay(12000);  // Give time for Serial monitor to initialize. adjust per your needs

    //Initialize multiplexer
    Serial.println("Initializing multiplexer...");
    if (!multiplexer.begin()) {
        Serial.println("TCA9548 initialization failed!");
        while (1);
    }
    Serial.println("Multiplexer initialized successfully.");

    buttonPress = false; //Button press simulator - Set to false after setup is complete
}

void loop() {
    if (!buttonPress) {   
        Serial.println("Starting impedance measurements...");

        for (int channel = 1; channel <= 3; channel++) {
            Serial.print("Switching to channel ");
            Serial.println(channel);

            multiplexer.selectChannel(channel);
            delay(50); // Allow channel to stabilize
            
            // Check if multiplexer connected to the AD5934
            if (!multiplexer.isConnected(AD5934_ADDR, channel)) {
                Serial.print("Multiplexer failed to select channel ");
                Serial.println(channel);
                continue; // Skip to next channel
            }
            
            Serial.print("Initializing and sweeping AD5934 on channel ");
            Serial.println(channel);

            // Use the corresponding start frequency for each channel
            unsigned long startFrequency = 0; // Initialize variable for the frequency

            switch (channel) {
                case 1:
                    startFrequency = START_FREQUENCY1;
                    break;
                case 2:
                    startFrequency = START_FREQUENCY2;
                    break;
                case 3:
                    startFrequency = START_FREQUENCY3;
                    break;
            }

            // Call setupAD5934 with the correct start frequency
            bool setupSuccess = ad5934.setupAD5934(startFrequency);

            // Check if setup succeeded; if not, skip this channel
            if (!setupSuccess) {
                Serial.print("Error initializing AD5934 on channel ");
                Serial.println(channel);
                continue; // Skip to next channel
            }

            // Perform the sweep and process data
            SweepAndProcess(channel, ad5934, multiplexer, magnitudeData, phaseData, channel - 1);

            delay(100); // Small delay before next channel
        }

        // Channel 0 - Read BME680 Sensor
        multiplexer.selectChannel(0);
        Serial.println("Reading environmental data from BME680...");
        float temperature = 0, humidity = 0;
        if (setupBME680(bme)) {  // Ensure BME680 is initialized before readings
            if (bme.performReading()) {
                temperature = bme.temperature;
                humidity = bme.humidity;

                Serial.print("Temperature: ");
                Serial.print(temperature);
                Serial.println(" *C");

                Serial.print("Humidity: ");
                Serial.print(humidity);
                Serial.println(" %");
            } else {
                Serial.println("Failed to read data from BME680.");
            }
        } else {
            Serial.println("Failed to initialize BME680 on channel 0.");
        }
        // After performing sweeps and capturing temperature/humidity
        sendDataToRaspberryPi(magnitudeData, phaseData, temperature, humidity);

        buttonPress = true; // Prevent re-triggering
    }
}




/*void loop() {
    if (!buttonPress) {   
        Serial.println("Starting impedance measurements...");

        // Channel 1 - Initialize and Sweep
        multiplexer.selectChannel(1);
        delay(50); // Allow channel to stabilize
        if (!multiplexer.isConnected(AD5934_ADDR, 1)) {
            Serial.println("Multiplexer failed to select channel 1");
            return;
        }
        Serial.println("Initializing and sweeping AD5934 on channel 1...");
        if (!ad5934.setupAD5934(ad5934, START_FREQUENCY1)) {
            Serial.println("Error initializing AD5934 on channel 1.");
        } else {
            SweepAndProcess(1, ad5934, multiplexer, magnitudeData, phaseData, 0);
        }
        delay(100);

        // Channel 2 - Initialize and Sweep
        multiplexer.selectChannel(2);
        delay(50); // Allow channel to stabilize
        if (!multiplexer.isConnected(AD5934_ADDR, 2)) {
            Serial.println("Multiplexer failed to select channel 2");
            return;
        }
        Serial.println("Initializing and sweeping AD5934 on channel 2...");
        if (!ad5934.setupAD5934(ad5934, START_FREQUENCY2)) {
            Serial.println("Error initializing AD5934 on channel 2.");
        } else {
            SweepAndProcess(2, ad5934, multiplexer, magnitudeData, phaseData, 1);
        }
        delay(100);

        // Channel 3 - Initialize and Sweep
        multiplexer.selectChannel(3);
        delay(50); // Allow channel to stabilize
        if (!multiplexer.isConnected(AD5934_ADDR, 3)) {
            Serial.println("Multiplexer failed to select channel 3");
            return;
        }   
        Serial.println("Initializing and sweeping AD5934 on channel 3...");
        if (!ad5934.setupAD5934(ad5934, START_FREQUENCY3)) {
            Serial.println("Error initializing AD5934 on channel 3.");
        } else {
            SweepAndProcess(3, ad5934, multiplexer, magnitudeData, phaseData, 2);
        }
        delay(100);

        // Channel 0 - Read BME680 Sensor
        multiplexer.selectChannel(0);
        Serial.println("Reading environmental data from BME680...");
        float temperature = 0, humidity = 0;
        if (setupBME680(bme)) {  // Ensure BME680 is initialized before readings
            if (bme.performReading()) {
                temperature = bme.temperature;
                humidity = bme.humidity;

                Serial.print("Temperature: ");
                Serial.print(temperature);
                Serial.println(" *C");

                Serial.print("Humidity: ");
                Serial.print(humidity);
                Serial.println(" %");
            } else {
                Serial.println("Failed to read data from BME680.");
            }
        } else {
            Serial.println("Failed to initialize BME680 on channel 0.");
        }

        // After performing sweeps and capturing temperature/humidity
        //sendDataToRaspberryPi(magnitudeData, phaseData, temperature, humidity);

        //Prevent re-triggering
        buttonPress = true;

    }
    // Delay between measurements
    //delay(30000); //adjust value per your needs
}*/