#include <Arduino.h>

// Pin Definitions
const int stepPin = 3; // Connect to STEP pin on DM542T
const int dirPin = 4;  // Connect to DIR pin on DM542T
const int enablePin = 2; // Connect to ENABLE pin on DM542T (optional)

// Constants
const float spoolRadius = 0.091; // Spool radius in meters
const int stepsPerRevolution = 200; // Steps per revolution (adjust for microstepping)
const float pi = 3.14159; // Pi value
const int microstepping = 16; // Microstepping value (e.g., 16x)

// Derived Values
const float circumference = 2 * pi * spoolRadius; // Circumference of spool
const float stepsPerMeter = (stepsPerRevolution * microstepping) / circumference;

String inputString = ""; // String to store user input
bool inputComplete = false;

void setup() {
    // Initialize Serial Communication
    Serial.begin(9600);
    Serial.println("Stepper Motor Line Control Initialized");
    Serial.println("Enter length to drop in meters (e.g., 1.2):");

    // Configure Pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);

    // Enable the stepper driver
    digitalWrite(enablePin, LOW); // LOW to enable, HIGH to disable
}

void loop() {
    // Check for incoming serial data
    while (Serial.available() > 0) {
        char receivedChar = Serial.read(); // Read the incoming character

        if (receivedChar == '\n' || receivedChar == '\r') { // Check for end of input
            inputComplete = true;
        } else {
            inputString += receivedChar; // Append character to input string
            Serial.print(receivedChar);  // Echo the character back to the terminal
        }
    }

    if (inputComplete) {
        float dropLength = inputString.toFloat(); // Convert input to float

        // Clear input string for next input
        inputString = "";
        inputComplete = false;

        if (dropLength > 0) {
            Serial.println(); // Move to the next line
            Serial.print("Dropping ");
            Serial.print(dropLength);
            Serial.println(" meters of line...");

            // Calculate the number of steps required
            int stepsToMove = round(dropLength * stepsPerMeter);

            // Set Direction (Assume one direction for dropping line)
            digitalWrite(dirPin, LOW); // LOW for dropping, HIGH for retracting

            // Move the stepper motor
            for (int i = 0; i < stepsToMove; i++) {
                digitalWrite(stepPin, HIGH);
                delayMicroseconds(100); // Adjust speed (shorter delay = faster)
                digitalWrite(stepPin, LOW);
                delayMicroseconds(100);
            }

            Serial.println("Line dropped.");
        } else {
            Serial.println("\nInvalid input. Enter a positive value.");
        }

        Serial.println("Enter length to drop in meters (e.g., 1.2):");
    }
}
