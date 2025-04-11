/*
// Title: AQUA Boat Project Full code
// Author: Cameron deLeeuw, Chris Scott
// Notes: 
      V3.4: Added integration with AQUAconnect.TM
            All sensors except for ultrasonic are connected directly to TrinketM0 microcontroller.
            Trinket is acting as a data hub in which data will be transfered to the Mega on surface vessel.
        Trinket Sensors:
        MS5837 PRESSURE sensor voltage: 3 V - 5 V
          - Purple, 5 V
          - Gray, GND
          - I2C SCL :: Digital #2 (PA09 / A1) :: White
          - I2C SDA :: Digital #0 (PA08 / A2) :: Black
         OXYGEN sensor voltage: 
            - 5 V
            - GND
            - Analog out :: Digital #1 (PA02 / A0)
        DS18B20 TEMPERATURE sensor voltage: 3 V - 5 V
          - Yellow :: Digital #3 (PA07 / A3)

        Mega Sensors:
        ULTRASONIC sensor voltage: 5 V - 24 V
          - white, pin 19 on mega
          - yellow, pin 18 on mega
        Trinket on Serial1:
            - Digital #4 (PA06 / A4) - Bit-banged UART TX to Mega RX1
        GPS 
            - TX1, RX1 UART to Serial3 (Mega pins 14 and 15)
        SD card
            - CS, black :: Pin 53
            - SCK, white :: Pin 52
            - MOSI, gray :: Pin 51
            - MISO, purple :: Pin 50
            - VCC, blue :: 5V
            - GND, green :: GND
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#define ULTRASONIC_COM 0x55
#define TRINKET_SERIAL Serial1
#define ULTRASONIC_SERIAL Serial2
#define GPS_SERIAL Serial3
#define MAX_WAYPOINTS 20

//AQUA Connect variables
float waypointDepths[MAX_WAYPOINTS];
int numWaypoints = 0;
int validEntries = 0;
int waypointNumber = 0;

//Motor variables
const int motorDirPin = 9;
const int motorPWMPin = 10;
const int fullSpeedPWM = 255;
const int slowSpeedPWM = 100;
const int manualMotorPin0 = 30;
const int manualMotorPin1 = 29;

bool waypointReached = false;
const unsigned long deployedWaitTime = 5000;  //was 90000

#define SD_CS 53

//Sensor variables
float oxygen = 0;
float temperature = 0;
float pressure = 0;
float depth = 0;
float distance = 0;
float gpsSpeed = 0.0;
float speed = 0.0;

//GPS variables
SFE_UBLOX_GNSS gps;
bool fixOK = false;
double latitude = 0, longitude = 0;
float altitude = 0;
uint8_t fixType = 0, rtkStatus = 0;
uint16_t year; uint8_t month, day, hour, minute, second;

//Oxygen calibration constants
const float vRef = 5.0;
const float V0 = 0.013;
const float V100 = 0.685;
const float calibTempC = 15.4;

File gpsLogFile;
File sensorLogFile;

//Variables for waypoint verification
unsigned long lastGpsLog = 0;
unsigned long lowSpeedStartTime = 0;
const float lowSpeedThreshold = 0.05;
const float movementTrigger = 0.1;
const unsigned long dwellTime = 3000;
bool hasBeenMoving = false;
bool enteredLowSpeedThreshold = false;

//Variables for reel control
float ultrasonicThreshold = 0.1;
bool targetDepthReached = false;
unsigned long targetDepthReachedTIME = 0;
bool sensorReadingsComplete = false;
const float surfaceThreshold = 0.1;
float targetDepth = .5;   //change based on desired depth
const float slowDownMargin = 0.1;
// for debug
bool reelingDown = false;
bool reelingUp = false;
bool surfaceReached = false;
bool wayPointAborted = false;

void readTrinketPacket();
void updateUltrasonic();
void reelOperation();
void logGPSData();
void logSensorData();
void gpsUpdate();
void manualMotorOveride();
void parseWaypointData(const String& data);


// ========== MAIN SETUP ========== //
void setup() {
  //Serial rates
  Serial.begin(115200);
  TRINKET_SERIAL.begin(9600);
  ULTRASONIC_SERIAL.begin(115200);
  GPS_SERIAL.begin(38400);

  pinMode(motorDirPin, OUTPUT);
  pinMode(motorPWMPin, OUTPUT);
  digitalWrite(motorDirPin, HIGH);
  pinMode(manualMotorPin0, INPUT_PULLUP);
  pinMode(manualMotorPin1, INPUT_PULLUP);

  if (!gps.begin(GPS_SERIAL)) {
    Serial.println("ZED-F9P not detected on UART. Check connections.");
    pinMode(13, OUTPUT);
    while (1) {
      digitalWrite(13, HIGH); delay(300);
      digitalWrite(13, LOW); delay(300);
    }
  }
  else {
    Serial.println("ZED-F9P detected over UART.");
  }

  //gps.setUART1Output(COM_TYPE_UBX);
  //gps.setNavigationFrequency(1);
  //gps.setAutoPVT(true);
  //gps.setAutoPVTcallbackPtr(NULL);

  //Wait for fix type
  while (gps.getFixType() < 3) {
    uint8_t fixType = gps.getFixType();

    Serial.print("GPS Fix Type: ");
    Serial.print(fixType);
    Serial.print(" → ");

    switch (fixType) {
      case 0: Serial.println("No fix"); break;
      case 1: Serial.println("Dead reckoning only"); break;
      case 2: Serial.println("2D fix"); break;
      case 3: Serial.println("3D fix (Good)"); break;
      case 4: Serial.println("GNSS + Dead reckoning"); break;
      case 5: Serial.println("Time only"); break;
      case 6: Serial.println("RTK Float (High precision)"); break;
      case 7: Serial.println("RTK Fixed (Centimeter-level) 😎"); break;
      default: Serial.println("Unknown fix type"); break;
    }
    delay(500);
  }

  if (!SD.begin(SD_CS)) Serial.println("SD initialization failed!");

  gpsLogFile = SD.open("gps_log.txt", FILE_WRITE);
  if (gpsLogFile) {
    gpsLogFile.println("Lat, Lon, Alt, Fix Type, RTK Status, Speed, Time, Date");
    gpsLogFile.println("==========================================================");
    gpsLogFile.close();
  } else {
    Serial.println("Error opening gps_log.txt");
  }

  sensorLogFile = SD.open("sens_log.txt", FILE_WRITE);
  if (sensorLogFile) {
    sensorLogFile.println("Hi! i've been written onto an SD card! Yay! This is my purpose!");
    sensorLogFile.close();
  } else {
    Serial.println("Error opening sens_log.txt");
  }

  Serial.println("AQUA Boat System Ready");

  //Count # of valid waypointDepths entries 
  for (int i = 0; i < 10; i++) {
    if (waypointDepths[i] > 0.1 && waypointDepths[i] <= 3.0) {
      validEntries++;
    }
  }
}



// ========== MAIN LOOP ========== //
void loop() {

  //manualMotorOveride(); //Manual motor overide for when reel gets in a bad situation
  
      // get and log GPS data every second
      if ((millis() - lastGpsLog) >= 1000) {  //Log gps data every second
        lastGpsLog = millis();
        gpsUpdate();
        logGPSData();
        Serial.print("Time(seconds) -> "); Serial.println(millis() / 1000);
        Serial.print("Speed (m/s): "); Serial.println(gpsSpeed);
        Serial.print("hasBeenMoving: "); Serial.println(hasBeenMoving);
        Serial.print("WaypointReached: "); Serial.println(waypointReached);
        Serial.print("Reeling down: "); Serial.print(reelingDown); Serial.print(" Current depth: "); Serial.println(depth); 
        Serial.print("target depth reached:"); Serial.println(targetDepthReached);
        Serial.print("SensorReadingsComplete: "); Serial.println(sensorReadingsComplete);
        Serial.print("surfaceReached: "); Serial.println(surfaceReached);
        Serial.print("== Waypoint aborted ==:"); Serial.println(wayPointAborted);
        Serial.println("");
        if (surfaceReached) surfaceReached = false; //debug
        if (wayPointAborted) wayPointAborted = false; //debug
        if (reelingUp) reelingUp = false;  //debug
      } 

      //Wait for command from AQUA Connect
      if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
    
        if (input.startsWith("WAYPOINTS:")) {
          parseWaypointData(input);
        }
      }

      //Reassign target depth with the next depth entry from AQUA Connect
      if (surfaceReached == true) {
        targetDepth = waypointDepths[waypointNumber];
        if (validEntries > waypointNumber) {
          waypointNumber = 0;
        }
      }
      
      if (gpsSpeed > movementTrigger) {
        hasBeenMoving = true; // if speed is greater than .5 m/s, boat is in motion, hasBeenMoving is set
        lowSpeedStartTime = 0; // reset dwell timer if we started moving again
      }
      if ((gpsSpeed < lowSpeedThreshold) && hasBeenMoving) { // If the speed is now lower than lowSpeedThreshold(.1m/s) and was just moving above 0.5 m/s, waypoint might be reached
        enteredLowSpeedThreshold = true;
        if (lowSpeedStartTime == 0) {
          lowSpeedStartTime = millis(); // start the 5 second dwell timer
        } else if ((millis() - lowSpeedStartTime) >= dwellTime) { // If not moving for 5 seconds, waypoint reached, begin reel operation
          waypointReached = true;
          hasBeenMoving = false;
          lowSpeedStartTime = 0;
        }
      }
      if ((gpsSpeed > lowSpeedThreshold) && enteredLowSpeedThreshold) { //If the boat begins to move again above 0.1 m/s, reset lowSpeedStartTime && hasBeenMoving = false;
        wayPointAborted = true;
        waypointReached = false;
        enteredLowSpeedThreshold = false;
        hasBeenMoving = false; 
        lowSpeedStartTime = 0;
        targetDepthReached = false;
        sensorReadingsComplete = true; //If waypoint is aborted && module is deployed, reel module up
      }

    readTrinketPacket();
    updateUltrasonic();
    reelOperation();  //Operates the reel logic 

    if (targetDepthReached == true) {   //If target depth is reached, start 90 second timer
      if (targetDepthReachedTIME == 0) targetDepthReachedTIME = millis();
      if ((millis() - targetDepthReachedTIME) >= deployedWaitTime) {  //When 90 seconds is complete, log sensor readings to SD card
        logSensorData(); 
        targetDepthReached = false;
        targetDepthReachedTIME = 0;
        sensorReadingsComplete = true;
      }
    }
}


// ========== FUNCTIONS ========= //
void gpsUpdate() {
  if (gps.getPVT()) {  // Efficient batch update from the GPS module
    fixOK = gps.getGnssFixOk();
    latitude = gps.getLatitude() / 1e7;
    longitude = gps.getLongitude() / 1e7;
    altitude = gps.getAltitude() / 1000.0;
    fixType = gps.getFixType();
    rtkStatus = gps.getCarrierSolutionType();
    gpsSpeed = gps.getGroundSpeed() / 1000.0;
    //Serial.println(latitude);

    year = gps.getYear();
    month = gps.getMonth();
    day = gps.getDay();
    hour = gps.getHour();
    minute = gps.getMinute();
    second = gps.getSecond();
  }
}

void updateUltrasonic() {
  static unsigned long lastPing = 0;
  if (millis() - lastPing >= 100) {
    lastPing = millis();
    ULTRASONIC_SERIAL.write(ULTRASONIC_COM);
  }

  static unsigned char buffer[4];
  uint8_t CS;

  while (ULTRASONIC_SERIAL.available() >= 4) {
    if (ULTRASONIC_SERIAL.read() == 0xFF) {
      buffer[0] = 0xFF;
      for (int i = 1; i < 4; i++) buffer[i] = ULTRASONIC_SERIAL.read();
      CS = buffer[0] + buffer[1] + buffer[2];
      if (buffer[3] == CS) {
        distance = ((buffer[1] << 8) + buffer[2]) / 1000.0;
      }
    }
  }
}

void readTrinketPacket() {
  static enum { WAIT_START, READ_O2_H, READ_O2_L, READ_P_H, READ_P_L,
                READ_T_H, READ_T_L, READ_D_H, READ_D_L, WAIT_END } state = WAIT_START;
  static uint16_t o2_raw, pressure_raw, temp_raw, depth_raw;
  static byte high;

  while (TRINKET_SERIAL.available()) {
    byte b = TRINKET_SERIAL.read();

    switch (state) {
      case WAIT_START: if (b == 0xAA) state = READ_O2_H; break;
      case READ_O2_H: high = b; state = READ_O2_L; break;
      case READ_O2_L: o2_raw = word(high, b); state = READ_P_H; break;
      case READ_P_H: high = b; state = READ_P_L; break;
      case READ_P_L: pressure_raw = word(high, b); state = READ_T_H; break;
      case READ_T_H: high = b; state = READ_T_L; break;
      case READ_T_L: temp_raw = word(high, b); state = READ_D_H; break;
      case READ_D_H: high = b; state = READ_D_L; break;
      case READ_D_L: depth_raw = word(high, b); state = WAIT_END; break;

      case WAIT_END:
        if (b == 0x55) {
          float voltage = o2_raw * (vRef / 1023.0); //Oxygen Calibration
          float T = calibTempC + 273.15;
          float DO_sat = exp(-139.34411 + (1.575701e5 / T)
            - (6.642308e7 / pow(T, 2)) + (1.243800e10 / pow(T, 3))
            - (8.621949e11 / pow(T, 4)));
          oxygen = (voltage - V0) * (DO_sat / (V100 - V0));
          oxygen = max(oxygen, 0);

          pressure = pressure_raw / 10.0;
          temperature = temp_raw / 100.0;
          depth = depth_raw / 100.0;
        }
        state = WAIT_START;
        break;
    }
  }
  if(depth > 20) depth = 0; //Pressure sensor might print an unreasonable value, set it to 20 m
}

void reelOperation() {

    if (waypointReached == true) 
    {
      if(depth < (targetDepth - slowDownMargin)) { //Begin descent
        digitalWrite(motorDirPin, HIGH); // Reel down
        reelingDown = true; //debug
        //analogWrite(motorPWMPin, fullSpeedPWM);
      }
      else if((depth >= (targetDepth - slowDownMargin)) && (depth < targetDepth) ) {  //Slow motor down if between slowDownMargin and targetDepth
        digitalWrite(motorDirPin, HIGH); // Reel down slowly
        //analogWrite(motorPWMPin, slowSpeedPWM);
      }
      //Serial.print("distance"); Serial.println(distance);
      if ((depth >= targetDepth) || ((distance <= ultrasonicThreshold) && distance != 0)){  //Stop motor if beyond target depth or < 100 mm from bottom. No distance from ultrasonic reads 0 hence && distance != 0
        reelingDown = false;  //debug
        analogWrite(motorPWMPin, 0);
        targetDepthReached = true;
        waypointReached = false;
      }
    }

    if(sensorReadingsComplete == true) { //Reel up if the sensor readings have completed
      if(depth > surfaceThreshold) {
        digitalWrite(motorDirPin, LOW);
        reelingUp = true; //debug
        //analogWrite(motorPWMPin, fullSpeedPWM);
      }
      else if(depth <= surfaceThreshold) {  //When module has returned, reset all flags
        analogWrite(motorPWMPin, 0);
        sensorReadingsComplete = false;
        //waypointReached = false;
        surfaceReached = true;
        enteredLowSpeedThreshold = false;
        waypointNumber++;
      }
    }
//}
}

void logGPSData() {
  gpsLogFile = SD.open("gps_log.txt", FILE_WRITE);
  if (gpsLogFile) {
    gpsLogFile.print("{\"latitude\":"); gpsLogFile.print(latitude, 8);
    gpsLogFile.print(",\"longitude\":"); gpsLogFile.print(longitude, 8);
    gpsLogFile.print(",\"altitude\":"); gpsLogFile.print(altitude, 3);
    gpsLogFile.print(",\"fixType\":"); gpsLogFile.print(fixType);
    gpsLogFile.print(",\"rtkStatus\":"); gpsLogFile.print(rtkStatus);
    gpsLogFile.print(",\"Jake_speed\":"); gpsLogFile.print(gpsSpeed, 2);
    gpsLogFile.print(",\"time\":\"");
    if (hour < 10) { gpsLogFile.print("0"); }   //For handling syntax 9/2/25 -> 09/02/2025
    gpsLogFile.print(hour); gpsLogFile.print(":");
    if (minute < 10) { gpsLogFile.print("0"); }
    gpsLogFile.print(minute); gpsLogFile.print(":");
    if (second < 10) { gpsLogFile.print("0"); }
    gpsLogFile.print(second);

    gpsLogFile.print("\",\"date\":\"");
    if (month < 10) { gpsLogFile.print("0"); }
    gpsLogFile.print(month); gpsLogFile.print("-");
    if (day < 10) { gpsLogFile.print("0"); }
    gpsLogFile.print(day); gpsLogFile.print("-");

    gpsLogFile.print(year);
    gpsLogFile.println("\"}");
    gpsLogFile.close();
  }
}

void logSensorData() {
  sensorLogFile = SD.open("sens_log.txt", FILE_WRITE);
  if (sensorLogFile) {
    sensorLogFile.print("{\"depth\":"); sensorLogFile.print(depth, 2);
    sensorLogFile.print(",\"distance\":"); sensorLogFile.print(distance, 2);
    sensorLogFile.print(",\"temperature\":"); sensorLogFile.print(temperature, 2);
    sensorLogFile.print(",\"oxygen\":"); sensorLogFile.print(oxygen, 2);
    sensorLogFile.print(",\"latitude\":"); sensorLogFile.print(latitude, 8);
    sensorLogFile.print(",\"longitude\":"); sensorLogFile.print(longitude, 8);
    sensorLogFile.println("}");
    sensorLogFile.close();
  }
}


void parseWaypointData(const String& data) {
  int firstColon = data.indexOf(':');
  int secondColon = data.indexOf(':', firstColon + 1);

  if (firstColon == -1 || secondColon == -1) {
    Serial.println("Invalid format");
    return;
  }

  numWaypoints = data.substring(firstColon + 1, secondColon).toInt();
  if (numWaypoints > MAX_WAYPOINTS) numWaypoints = MAX_WAYPOINTS;

  String depths = data.substring(secondColon + 1);
  int lastIndex = 0;

  for (int i = 0; i < numWaypoints; i++) {
    int commaIndex = depths.indexOf(',', lastIndex);
    String depthStr;

    if (commaIndex == -1) {
      depthStr = depths.substring(lastIndex);
    } else {
      depthStr = depths.substring(lastIndex, commaIndex);
      lastIndex = commaIndex + 1;
    }

    waypointDepths[i] = depthStr.toFloat();
  }

  // Debug print
  Serial.print("Received "); Serial.print(numWaypoints); Serial.println(" waypoints:");
  for (int i = 0; i < numWaypoints; i++) {
    Serial.print("  Waypoint "); Serial.print(i + 1);
    Serial.print(": Depth = "); Serial.println(waypointDepths[i]);
  }
  Serial.println("WAYPOINTS_RECEIVED");
}

//void manualMotorOveride() 
//{
//  if (digitalRead(manualMotorPin0) == LOW) {
//    digitalWrite(motorDirPin, HIGH);
//    analogWrite(motorPWMPin, slowSpeedPWM);
//  }
//  else if (digitalRead(manualMotorPin1) == LOW) {
//    digitalWrite(motorDirPin, LOW);
//    analogWrite(motorPWMPin, slowSpeedPWM);
//  }
//  if (digitalRead(manualMotorPin0) == HIGH && digitalRead(manualMotorPin1) == HIGH) {
//    analogWrite(motorPWMPin, 0);
//  }
//}
