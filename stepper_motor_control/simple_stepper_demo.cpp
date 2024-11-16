#include <Arduino.h>

// Pin Definitions
const int stepPin = 3; // Connect to STEP pin on DM542T
const int dirPin = 4;  // Connect to DIR pin on DM542T

// Constants
const int stepsPerRevolution = 200; // Full steps per revolution (1.8° per step in full-step mode)
const int microstepping = 16; // Microstepping value set on the driver
const int stepsPerCycle = stepsPerRevolution * microstepping; // Total steps for one full revolution
const int stepDelay = 2000; // Microseconds delay for motor speed

void setup() {
    // Configure Pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

}

void loop() {
    // Move motor in one direction
    digitalWrite(dirPin, LOW); // Set direction to one way
    for (int i = 0; i < stepsPerRevolution; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay);
    }
    delay(500); // Pause before reversing

    // Move motor in the opposite direction
    digitalWrite(dirPin, HIGH); // Reverse direction
    for (int i = 0; i < stepsPerRevolution; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay);
    }
    delay(500); // Pause before reversing again
}
