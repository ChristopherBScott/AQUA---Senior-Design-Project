#include <Arduino.h>

// Include the required Arduino libraries:                                                            //ADDED LINE
#include "OneWire.h"                                                                                  //ADDED LINE
#include "DallasTemperature.h"                                                                        //ADDED LINE

// Pin Definitions
const int stepPin = 3; // Connect to STEP pin on DM542T
const int dirPin = 4;  // Connect to DIR pin on DM542T

// Define to which pin of the Arduino the 1-Wire bus is connected:                                    //ADDED LINE
#define ONE_WIRE_BUS 2                                                                                //ADDED LINE

// Create a new instance of the oneWire class to communicate with any OneWire device:                 //ADDED LINE
OneWire oneWire(ONE_WIRE_BUS);                                                                        //ADDED LINE

// Pass the oneWire reference to DallasTemperature library:                                           //ADDED LINE
DallasTemperature sensors(&oneWire);                                                                  //ADDED LINE

// Constants
const float spoolRadius = 0.091; // Spool radius in meters
const int stepsPerRevolution = 200; // Steps per revolution (adjust for microstepping)
const float pi = 3.14159; // Pi value
const int speed = 250;

// Derived Values
const float circumference = 2 * pi * spoolRadius; // Circumference of spool
const float metersPerStep = (1.8 * (pi/180)) * spoolRadius; // ArcLength = theta(radians) * radius

// Variables
int stepsToMove = 0;
int lastStepsToMove = 0;

String inputString = ""; // String to store user input
bool inputComplete = false;

unsigned long previousMillis = 0; // To store last time temperature was read                               //ADDED LINE
const long interval = 2000; // Interval to read temperature (2 seconds)                                    //ADDED LINE

void setup() {
    // Initialize Serial Communication
    Serial.begin(9600);

    // Start up the library:                                                                               //ADDED LINE
    sensors.begin();                                                                                       //ADDED LINE
    //sensors.setResolution(9); // sets resolution of readings to 9 bit, up to 12 for more precision       //ADDED LINE


    Serial.println("Stepper Motor Line Control Initialized");
    Serial.println("Enter length to drop in meters (e.g., 1.2):");

    // Configure Pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
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

        if (dropLength >= 0) {
            Serial.println(); // Move to the next line
            Serial.print("Dropping ");
            Serial.print(dropLength);
            Serial.println(" meters of line...");
            

            // Calculate the number of steps required
            stepsToMove = round(dropLength / metersPerStep);
            Serial.print("Steps from zero: ");
            Serial.println(stepsToMove);
            

            if (stepsToMove > lastStepsToMove) {
                
                stepsToMove = stepsToMove - lastStepsToMove;
                
                digitalWrite(dirPin, LOW); // Direction
                for (int i = 0; i < stepsToMove; i++) {
                    digitalWrite(stepPin, HIGH);
                    delayMicroseconds(speed); // Adjust speed (shorter delay = faster)
                    digitalWrite(stepPin, LOW);
                    delayMicroseconds(speed);
                }
                lastStepsToMove = stepsToMove;
                stepsToMove = 0;
                Serial.println("Line length changed.");
            }
            elif (stepsToMove < lastStepsToMove) {
                stepsToMove = lastStepsToMove - stepsToMove;
                digitalWrite(dirPin, HIGH);  // Direction
                for (int i = 0; i < stepsToMove; i++) {
                    digitalWrite(stepPin, HIGH);
                    delayMicroseconds(speed); // Adjust speed (shorter delay = faster)
                    digitalWrite(stepPin, LOW);
                    delayMicroseconds(speed);
                }
                lastStepsToMove = stepsToMove;
                stepsToMove = 0;
                Serial.println("Line length changed");
            }
            
        } else {
            Serial.println("\nInvalid input.");
        }

        Serial.println("Enter length to drop in meters (e.g., 1.2):");
    }

    // Handle temperature readings every 2 seconds                                    //ADDED LINE
    unsigned long currentMillis = millis();                                           //ADDED LINE
    if (currentMillis - previousMillis >= interval) {                                 //ADDED LINE
        // Save the last time temperature was read                                    //ADDED LINE
        previousMillis = currentMillis;                                               //ADDED LINE

        // Request temperature readings from all sensors                              //ADDED LINE
        sensors.requestTemperatures();                                                //ADDED LINE

        // Print temperature                                                          //ADDED LINE
        float tempC = sensors.getTempCByIndex(0);                                     //ADDED LINE
        Serial.print("Temperature: ");                                                //ADDED LINE
        Serial.print(tempC);                                                          //ADDED LINE
        Serial.print(" \xC2\xB0"); // shows degree symbol                             //ADDED LINE
        Serial.println("C");                                                          //ADDED LINE
    }
}
