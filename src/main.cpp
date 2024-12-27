#include <Arduino.h>
#include <Wire.h>                          // For I2C communication
#include <Adafruit_Sensor.h>               // For the unified sensor interface
#include <Adafruit_BME680.h>               // For the BME680 sensor
#include "I2C_EXT.h"
#include "TCA9548.h"                       //Multiplexer TCA9548a library

//Multiplexer instance
TCA9548 multiplexer(0x70); // Address for TCA9548 (default)

//AD5934 instance
AD5934 ad5934; 

//BME680 instance
Adafruit_BME680 bme; 

// Arrays to store data for all devices
float magnitudeData[3][NUM_INCREMENTS]; // 3 devices, NUM_INCREMENTS points per sweep
float phaseData[3][NUM_INCREMENTS];

void setup()
{
    Wire.begin();
    Serial.begin(9600);
    delay(6000);  // Give time for Serial monitor to initialize. adjust per your needs

    //Initialize multiplexer
    if (!multiplexer.begin())
    {
        while (1);
    }
}

void loop()
{
if (Serial.available() >0)
    {
        String command = Serial.readString();

        if(command == "start")
        {
            for (int channel = 1; channel <= 3; channel++)
            {
                multiplexer.selectChannel(channel);
                delay(50); // Allow channel to stabilize

                // Check if multiplexer connected to the AD5934
                if (!multiplexer.isConnected(AD5934_ADDR, channel))
                {
                    continue; // Skip to next channel
                }

                unsigned long startFrequency;

                switch (channel)
                {
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

                if (!ad5934.setupAD5934(startFrequency))
                {
                    continue; // Skip to next channel
                }

                // Call setupAD5934 with the correct start frequency
                bool setupSuccess = ad5934.setupAD5934(startFrequency);

                // Check if setup succeeded; if not, skip this channel
                if (!setupSuccess)
                {
                    continue; // Skip to next channel
                }

                // Perform the sweep and process data
                SweepAndProcess(channel, ad5934, multiplexer, magnitudeData, phaseData, channel - 1);

                delay(100); // Small delay before next channel
            }

            //Gather temperature & humidity readings
            float temperature = 0;
            float humidity = 0;

            multiplexer.selectChannel(0);
            setupBME680(bme);  
            bme.performReading();

            temperature = bme.temperature;
            humidity = bme.humidity;

            // Send magnitudeData and phaseData
            for (int channel = 0; channel < 3; channel++) // Iterate through each device
            {
                for (int i = 0; i < NUM_INCREMENTS; i++) // Iterate through each data point
                {
                    Serial.print(magnitudeData[channel][i], 3);
                    Serial.write(44); // Comma separator between magnitude and phase
                    Serial.print(phaseData[channel][i], 3);

                    if(i!=NUM_INCREMENTS-1)
                    {
                        Serial.write(44);
                    }
                }
                
                if (channel != 3) // Add an ampersand (&) between channels
                {
                    Serial.write(38);
                }
            }
            Serial.write(38); // Separator between data and environmental readings
            Serial.print(temperature, 2);
            Serial.write(38);
            Serial.print(humidity, 2);
        }
    }
}



