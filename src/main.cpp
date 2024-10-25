#include <Arduino.h>

// put function declarations here:
//int myFunction(int, int);

void setup() {
  Serial.begin(9600);  // Initialize serial communication with Raspberry Pi
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}