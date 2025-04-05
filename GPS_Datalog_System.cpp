#include <Arduino.h>

/*

Fix Type
0 = No Fix
2 = 2D Fix
3 = 3D Fix
4 = GNSS + Dead Reckoning
5 = Time-only Fix

RTK Status:
0 = No RTK Fix
1 = RTK Float (sub-meter accuracy)
2 = RTK Fixed (centimeter-level accuracy)

*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

SFE_UBLOX_GNSS myGNSS;
File logFile;

const int chipSelect = 53;  // Chip Select (CS) pin for the SD card module

void setup() {
    Serial.begin(9600);   // USB Serial for debugging
    Wire.begin();           // Initialize I2C
    pinMode(13, OUTPUT);  //setting up L LED on board to blink in case of error

    digitalWrite(13, LOW); //make LED default off

    // Initialize SD card
    if (!SD.begin(chipSelect)) {
        Serial.println("SD card initialization failed!");
        while (1){  // Stop if no SD card is found
            digitalWrite(13, HIGH);  // Turn LED on
            delay(300);             // Wait 
            digitalWrite(13, LOW);   // Turn LED off
            delay(300);             // Wait
        };
    }
    Serial.println("SD card initialized.");

    // Initialize ZED-F9P GPS module
    if (!myGNSS.begin()) {
        Serial.println("ZED-F9P not detected. Check connections.");
        while (1){  // Stop if no SD card is found
            digitalWrite(13, HIGH);  // Turn LED on
            delay(300);             // Wait 
            digitalWrite(13, LOW);   // Turn LED off
            delay(300);             // Wait
        };
    }
    Serial.println("ZED-F9P connected!");
    Serial.println("Lat, Lon, Alt (m)");

    myGNSS.setI2COutput(COM_TYPE_UBX);  // Use UBX protocol
    myGNSS.setNavigationFrequency(1);   // Set update rate to 1Hz

    // Open file for logging
    logFile = SD.open("gps_log.txt", FILE_WRITE);
    if (logFile) {
        logFile.println("Lat, Lon, Alt (m), Fix Type, RTK Status, Speed (m/s), UTC Time, Date");  // Write header
        logFile.close();
    } else {
        Serial.println("Error opening gps_log.txt");
    }
}

void loop() {

    if (myGNSS.getGnssFixOk()) {
        double latitude = myGNSS.getLatitude() / 1e7;  // Convert to decimal degrees
        double longitude = myGNSS.getLongitude() / 1e7;  
        float altitude = myGNSS.getAltitude() / 1000.0;  // Convert to meters

        // UTC Time & Date
        uint16_t year = myGNSS.getYear();
        uint8_t month = myGNSS.getMonth();
        uint8_t day = myGNSS.getDay();
        uint8_t hour = myGNSS.getHour();
        uint8_t minute = myGNSS.getMinute();
        uint8_t second = myGNSS.getSecond();

        // Ground Speed in m/s and km/h
        float speed_mps = myGNSS.getGroundSpeed() / 1000.0; // Convert mm/s to m/s

        // Fix Type
        uint8_t fixType = myGNSS.getFixType();

        // RTK Status 
        uint8_t rtkStatus = myGNSS.getCarrierSolutionType();


        // Print to Serial Monitor (Debug purposes only, prolly remove before uploading)
        //Serial.print("Lat: "); 
        Serial.print(latitude, 8); Serial.print(" , "); 
        //Serial.print(" Lon: "); 
        Serial.print(longitude, 8); Serial.print(" , "); 
        //Serial.print(" Alt: "); 
        Serial.println(altitude, 3); //Serial.println(" m");
        //Serial.print(" Sats: "); Serial.print(numSats); 
        //Serial.print(" | FixType: "); Serial.print(fixType);
        //Serial.print(" | RTK: "); Serial.println(rtkStatus);
        //Serial.print(" Speed: "+String(speed_mps,3)+" m/s\n");
        //Serial.println(" UTC Time: "+String(hour)+":"+String(minute)+":"+String(second)+" | Date: "+String(month)+"-"+String(day)+"-"+String(year));

        // Write to SD Card
        String dataline = String(latitude, 8)+", "+String(longitude, 8)+", "+
            String(altitude, 3)+", "+String(fixType)+", "+String(rtkStatus)+
            ", "+String(speed_mps,3)+", "+String(hour)+":"+String(minute)+":"+String(second)+", "+String(month)+
            "-"+String(day)+"-"+String(year);

        logFile = SD.open("gps_log.txt", FILE_WRITE);
        if (logFile) { 
            logFile.println(dataline);
            logFile.close();
        } else {
            Serial.println("Error writing to gps_log.txt");
        }
    } else {
        Serial.println("No GPS fix.");
    }

    delay(1000);  // Wait 1 second before next reading
}
