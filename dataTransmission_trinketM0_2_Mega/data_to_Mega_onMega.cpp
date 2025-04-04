#include <Arduino.h>

#define ULTRASONIC_COM 0x55
#define mySerial Serial2  // Ultrasonic on Serial2

unsigned char buffer_RTT[4] = {0};
uint8_t CS;
int Distance = 0;

void setup() {
  Serial.begin(9600);     // Debug to Serial Monitor
  Serial1.begin(9600);    // Receive from Trinket
  mySerial.begin(115200); // Ultrasonic Sensor
  Serial.println("Mega listening for sensor packet...");
}

void loop() {
  // ---------- Background: Request and read ultrasonic ----------
  static unsigned long lastUltraPing = 0;
  if (millis() - lastUltraPing >= 100) {
    lastUltraPing = millis();
    mySerial.write(ULTRASONIC_COM);
  }

  // Only process if we have enough bytes
  while (mySerial.available() >= 4) {
    if (mySerial.read() == 0xFF) {
      buffer_RTT[0] = 0xFF;
      for (int i = 1; i < 4; i++) {
        buffer_RTT[i] = mySerial.read();
      }
      CS = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];
      if (buffer_RTT[3] == CS) {
        Distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
      } else {
        Serial.println("Ultrasonic checksum mismatch");
      }
    }
  }

  // ---------- Handle Trinket Serial Packet ----------
  static enum {
    WAIT_START, READ_O2_H, READ_O2_L, READ_P_H, READ_P_L,
    READ_T_H, READ_T_L, READ_D_H, READ_D_L, WAIT_END
  } state = WAIT_START;

  static int o2, pressure, temp, depth;
  static byte high, low;

  while (Serial1.available()) {
    byte b = Serial1.read();

    switch (state) {
      case WAIT_START:
        if (b == 0xAA) state = READ_O2_H;
        break;

      case READ_O2_H: high = b; state = READ_O2_L; break;
      case READ_O2_L: low = b; o2 = word(high, low); state = READ_P_H; break;

      case READ_P_H: high = b; state = READ_P_L; break;
      case READ_P_L: low = b; pressure = word(high, low); state = READ_T_H; break;

      case READ_T_H: high = b; state = READ_T_L; break;
      case READ_T_L: low = b; temp = word(high, low); state = READ_D_H; break;

      case READ_D_H: high = b; state = READ_D_L; break;
      case READ_D_L: low = b; depth = word(high, low); state = WAIT_END; break;

      case WAIT_END:
        if (b == 0x55) {
          Serial.print("O2 (raw ADC): "); Serial.println(o2);
          Serial.print("Pressure (mbar): "); Serial.println(pressure / 10.0);
          Serial.print("Temp (°C): "); Serial.println(temp / 100.0);
          Serial.print("Depth (m): "); Serial.println(depth / 100.0);
          Serial.print("Ultrasonic Distance (mm): "); Serial.println(Distance);
          Serial.println("---------------------------");
        }
        state = WAIT_START;
        break;
    }
  }
}
