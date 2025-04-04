// === Trinket M0 Sensor Sender (Bit-Banged UART) ===
// Pin Allocations:
// ----------------
// Digital #0 (PA08 / A2) - I2C SDA (MS5837 pressure sensor)
// Digital #2 (PA09 / A1) - I2C SCL (MS5837 pressure sensor)
// Digital #1 (PA02 / A0) - Analog input (Oxygen sensor)
// Digital #3 (PA07 / A3) - 1-Wire temp sensor (DS18B20)
// Digital #4 (PA06 / A4) - Bit-banged UART TX to Mega RX1
// GND ↔ GND shared with Mega

#include <Wire.h>
#include <MS5837.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define OXYGEN_PIN A0
#define ONE_WIRE_PIN A3
#define SOFT_TX_PIN 4
#define BAUD_RATE 9600
#define BIT_DELAY (1000000 / BAUD_RATE)  // µs per bit

MS5837 pressureSensor;
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature tempSensor(&oneWire);

void softSerialWriteByte(uint8_t b) {
  digitalWrite(SOFT_TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY);

  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(SOFT_TX_PIN, (b >> i) & 0x01);
    delayMicroseconds(BIT_DELAY);
  }

  digitalWrite(SOFT_TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY);
}

void sendInt16Soft(int value) {
  softSerialWriteByte(highByte(value));
  softSerialWriteByte(lowByte(value));
}

void setup() {
  pinMode(SOFT_TX_PIN, OUTPUT);
  digitalWrite(SOFT_TX_PIN, HIGH); // Idle state

  Serial.begin(9600);
  //while (!Serial);

  Serial.println("Trinket bit-banged UART sensor sender starting...");

  Wire.begin();

  if (!pressureSensor.init()) {
    Serial.println("Pressure sensor NOT detected!");
  } else {
    pressureSensor.setModel(MS5837::MS5837_02BA);
    pressureSensor.setFluidDensity(997);
  }
  
  tempSensor.begin();
}

void loop() {
  int oxygenVal = analogRead(OXYGEN_PIN);

  pressureSensor.read();
  float pressure = pressureSensor.pressure(); // mbar
  float depth = pressureSensor.depth();       // m

  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);

  int pressureVal = pressure * 10;   // mbar * 10
  int tempVal = tempC * 100;         // °C * 100
  int depthVal = depth * 100;        // m * 100 → cm

  // Send packet
  softSerialWriteByte(0xAA); // Start byte
  sendInt16Soft(oxygenVal);
  sendInt16Soft(pressureVal);
  sendInt16Soft(tempVal);
  sendInt16Soft(depthVal);
  softSerialWriteByte(0x55); // End byte
  
  
  Serial.print("Oxygen: "); Serial.println(oxygenVal);
  Serial.print("pressureVal: "); Serial.println(pressureVal);
  Serial.print("tempVal: "); Serial.println(tempVal);
  Serial.print("depthVal: "); Serial.println(depthVal);
  Serial.println("======================");

  delay(1000);
}
