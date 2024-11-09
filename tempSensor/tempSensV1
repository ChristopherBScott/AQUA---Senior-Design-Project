#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is connected to digital pin 2
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(9600);
  sensors.begin();  // Start the library
}

void loop() {
  sensors.requestTemperatures(); // Send command to get temperatures
  float temperatureC = sensors.getTempCByIndex(0); // Get temperature in Celsius
  float temperatureF = temperatureC * (9.0 / 5.0) + 32.0; // convert to Farenheit
  Serial.print("Temperature: ");
  Serial.print(temperatureF);
  Serial.println(" °F");

  delay(1000); // Wait 1 second before the next reading
}
