#include <Arduino.h>
#include <Wire.h>
#include <Wire.h>                          // For I2C communication
#include <Adafruit_Sensor.h>               // For the unified sensor interface
#include <Adafruit_BME680.h>               // For the BME680 sensor
// #include <Adafruit_GFX.h>               // for a display that requires GFX library
#include "I2C_EXT.h"
#include "TCA9548.h"                       //Multiplexer TCA9548a library

// Multiplexer instance
TCA9548 multiplexer(0x70); // Address for TCA9548 (default)

// AD5934 instances
AD5934 ad5934_1; //1st IC instance
AD5934 ad5934_2; //2nd IC instance
AD5934 ad5934_3; //3rd IC instance
// AD5934 ad5934; // ONLY when using 1 AD5934 Instance of the class for calling non-static methods

// BME680 global instance
Adafruit_BME680 bme; 

bool buttonPress = true; // Button press simulator - Initialize as true to avoid immediate trigger


//helper function definition, supposedly it is good practice to define in "main"
bool initializeDevice(uint8_t channel, AD5934 &device) {
    multiplexer.selectChannel(channel);
    return AD5934::setupAD5934(device); // Assuming setupAD5934 initializes an AD5934 device
}


void setup() {
    Wire.begin();
    Serial.begin(9600);
    delay(20000);  // Give time for Serial monitor to initialize. adjust per your needs

    // Initialize multiplexer
    if (!multiplexer.begin()) {
        Serial.println("TCA9548 initialization failed!");
        while (1);
    }

        // Initialize AD5934 devices on separate channels
    if (!initializeDevice(0, ad5934_1)) Serial.println("Error initializing AD5934 on channel 0");
    if (!initializeDevice(1, ad5934_2)) Serial.println("Error initializing AD5934 on channel 1");
    if (!initializeDevice(2, ad5934_3)) Serial.println("Error initializing AD5934 on channel 2");

    Serial.println("Initializing BME680...");
    if (!setupBME680(bme)) {
        Serial.println("Error: BME680 setup failed.");
    }

    buttonPress = false; // Button press simulator - Set to false after setup is complete
}

void loop() {
    if (!buttonPress) {   
    Serial.println("Starting impedance measurement...");

        SweepAndProcess(0, ad5934_1, multiplexer);
        SweepAndProcess(1, ad5934_2, multiplexer);
        SweepAndProcess(2, ad5934_3, multiplexer);


        // After all sweeps, power down devices to save energy
        ad5934_1.setPowerMode(POWER_DOWN);
        ad5934_2.setPowerMode(POWER_DOWN);
        ad5934_3.setPowerMode(POWER_DOWN);
    

    // Immediately after impedance sweep, read and display temperature and humidity
    Serial.println("Reading temperature and humidity...");
    readSensorData(bme); // Call the function to handle temperature and humidity readings

    // Prevent re-triggering
    buttonPress = true;
    }

    // Delay between measurements
    delay(30000); //adjust value per your needs

}
