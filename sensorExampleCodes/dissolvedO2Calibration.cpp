#include <Arduino.h>
#include <math.h>

const int doPin = A0;         // DO sensor analog input
const float vRef = 5.0;       // ADC reference voltage (5V for Arduino Mega)

// Calibration voltages
const float V0 = 0.013;       // Voltage at 0% DO
const float V100 = 0.685;     // Voltage at 100% DO

const float tempC = 15.4;     // Temperature of 100% DO calibration water

// Function to compute DO in mg/L using Benson & Krause + linear voltage calibration
float getDOSaturation(float voltage, float tempC, float V0, float V100) {
  float T = tempC + 273.15;

  float lnDOsat = -139.34411 
                + (1.575701e5 / T) 
                - (6.642308e7 / pow(T, 2)) 
                + (1.243800e10 / pow(T, 3)) 
                - (8.621949e11 / pow(T, 4));
  float DO_sat = exp(lnDOsat);  // mg/L at 100% saturation

  float DO_mg_per_L = (voltage - V0) * (DO_sat / (V100 - V0));
  if (DO_mg_per_L < 0) DO_mg_per_L = 0;

  return DO_mg_per_L;
}

// Function to print elapsed time since startup
void printElapsedTime() {
  unsigned long elapsedMillis = millis();
  unsigned int seconds = (elapsedMillis / 1000) % 60;
  unsigned int minutes = (elapsedMillis / 60000);

  Serial.print("Time Elapsed: ");
  if (minutes > 0) {
    Serial.print(minutes);
    Serial.print(" min ");
  }
  Serial.print(seconds);
  Serial.println(" sec");
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("DFRobot DO Sensor - Accurate Calibration Mode");
}

void loop() {
  int raw = analogRead(doPin);
  float voltage = raw * (vRef / 1023.0);

  float DO_mg_per_L = getDOSaturation(voltage, tempC, V0, V100);

  Serial.print("Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V | DO: ");
  Serial.print(DO_mg_per_L, 2);
  Serial.println(" mg/L");

  printElapsedTime();
  Serial.println("------------------------");

  delay(1000); // 1 second interval
}
