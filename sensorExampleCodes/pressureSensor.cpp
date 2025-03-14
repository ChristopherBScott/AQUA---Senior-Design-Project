#include <Wire.h>
#include <MS5837.h>

MS5837 sensor;

//SDA to pin 20
//SCL to pin 21
//MS5837 pressure sensor

void setup() {
    Serial.begin(115200);
    Wire.begin();

    if (!sensor.init()) {
        Serial.println("MS5837 sensor not found!");
        while (1);
    }

    sensor.setModel(MS5837::MS5837_02BA);
    sensor.setFluidDensity(997); // Freshwater density (997 kg/m³), use 1029 for seawater

    Serial.println("MS5837 initialized.");
}

void loop() {
    sensor.read(); // Get sensor readings

    Serial.print("Pressure (mbar): ");
    Serial.print(sensor.pressure());
    Serial.println(" mbar");

    Serial.print("Temperature (°C): ");
    Serial.print(sensor.temperature());
    Serial.println(" °C");

    Serial.print("Depth (m): ");
    Serial.print(sensor.depth());
    Serial.println(" m");

    Serial.print("Altitude (m): ");
    Serial.print(sensor.altitude());
    Serial.println(" m");

    Serial.println("--------------------------");
    delay(1000);
}
