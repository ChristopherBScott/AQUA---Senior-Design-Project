#include <Arduino.h>
#include <PID_v1.h>

// === Pin Definitions ===
#define TRINKET_SERIAL Serial1
#define motorDirPin 9
#define motorPWMPin 10

// === Default PID Settings ===
const double defaultTargetDepth = 1;
const double defaultKp = 1000.0;
const double defaultKi = 0;
const double defaultKd = 0;

// === PID Control Variables ===
double depth = 0;
double pidInput = 0;
double pidOutput = 0;
double pidSetpoint = defaultTargetDepth;
double Kp = defaultKp, Ki = defaultKi, Kd = defaultKd;
PID depthPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// === Serial Input ===
String inputBuffer = "";
bool waitingForInput = false;
int inputStage = 0;

// === System State ===
enum SystemState { IDLE, DEPLOYING, RETURNING };
SystemState currentState = IDLE;

// === Depth Packet Parsing ===
enum PacketState {
  WAIT_START, READ_O2_H, READ_O2_L, READ_P_H, READ_P_L,
  READ_T_H, READ_T_L, READ_D_H, READ_D_L, WAIT_END
};

PacketState state = WAIT_START;
byte high;
uint16_t depth_raw;

void readTrinketPacket();
void handleSerialInput();

void setup() {
  Serial.begin(115200);
  TRINKET_SERIAL.begin(9600);

  pinMode(motorDirPin, OUTPUT);
  pinMode(motorPWMPin, OUTPUT);
  analogWrite(motorPWMPin, 0);

  depthPID.SetTunings(Kp, Ki, Kd);
  depthPID.SetMode(AUTOMATIC);
  depthPID.SetOutputLimits(75, 255);

  Serial.println("=== PID Depth Control System ===");
  Serial.println("Available commands:");
  Serial.println(" - go     : deploy to target depth using PID");
  Serial.println(" - stop   : return to surface depth < 0.1 m");
  Serial.println(" - reset  : update targetDepth, Kp, Ki, Kd via serial");
  Serial.println();
  Serial.print("Target Depth: "); Serial.println(pidSetpoint);
  Serial.print("Kp: "); Serial.println(Kp);
  Serial.print("Ki: "); Serial.println(Ki);
  Serial.print("Kd: "); Serial.println(Kd);
}

void loop() {
  readTrinketPacket();
  handleSerialInput();

  pidInput = depth;
  depthPID.Compute();

  switch (currentState) {
    case DEPLOYING:
      if (depth < pidSetpoint - 0.01) {
        digitalWrite(motorDirPin, HIGH);
        analogWrite(motorPWMPin, (int)pidOutput);
      } else if (depth > pidSetpoint + 0.01) {
        digitalWrite(motorDirPin, LOW);
        analogWrite(motorPWMPin, (int)pidOutput);
      } else {
        analogWrite(motorPWMPin, 0);
      }
      break;

    case RETURNING:
      if (depth > 0.1) {
        digitalWrite(motorDirPin, LOW);
        analogWrite(motorPWMPin, 200);
      } else {
        analogWrite(motorPWMPin, 0);
        currentState = IDLE;
        Serial.println("Returned to surface. System is IDLE.");
      }
      break;

    case IDLE:
      analogWrite(motorPWMPin, 0);
      break;
  }

  // Teleplot output
  //Serial.print(">depth:"); Serial.println(depth);
  //Serial.print(">target:"); Serial.println(pidSetpoint);
  //Serial.print(">pidOutput:"); Serial.println(pidOutput);

  delay(100);
}

void readTrinketPacket() {
  static uint16_t dummy;
  while (TRINKET_SERIAL.available()) {
    byte b = TRINKET_SERIAL.read();
    switch (state) {
      case WAIT_START: if (b == 0xAA) state = READ_O2_H; break;
      case READ_O2_H: state = READ_O2_L; high = b; break;
      case READ_O2_L: dummy = word(high, b); state = READ_P_H; break;
      case READ_P_H: state = READ_P_L; high = b; break;
      case READ_P_L: dummy = word(high, b); state = READ_T_H; break;
      case READ_T_H: state = READ_T_L; high = b; break;
      case READ_T_L: dummy = word(high, b); state = READ_D_H; break;
      case READ_D_H: state = READ_D_L; high = b; break;
      case READ_D_L: depth_raw = word(high, b); state = WAIT_END; break;
      case WAIT_END:
        if (b == 0x55) {
          depth = depth_raw / 100.0;
          if (depth > 20) depth = 0;
        }
        state = WAIT_START;
        break;
    }
  }
}

void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    Serial.write(c);
    if (c == '\n' || c == '\r') {
      inputBuffer.trim();
      if (!waitingForInput) {
        if (inputBuffer == "go") {
          currentState = DEPLOYING;
          Serial.println("\nDeploying to target depth...");
        } else if (inputBuffer == "stop") {
          currentState = RETURNING;
          Serial.println("\nReturning to surface...");
        } else if (inputBuffer == "reset") {
          waitingForInput = true;
          inputStage = 0;
          Serial.println("\nEnter new target depth (m):");
        }
        inputBuffer = "";
      } else {
        float value = inputBuffer.toFloat();
        switch (inputStage) {
          case 0:
            pidSetpoint = value;
            Serial.print("\nTarget depth set to: "); Serial.println(pidSetpoint);
            Serial.println("Enter Kp:");
            break;
          case 1:
            Kp = value;
            Serial.print("\nKp set to: "); Serial.println(Kp);
            Serial.println("Enter Ki:");
            break;
          case 2:
            Ki = value;
            Serial.print("\nKi set to: "); Serial.println(Ki);
            Serial.println("Enter Kd:");
            break;
          case 3:
            Kd = value;
            Serial.print("\nKd set to: "); Serial.println(Kd);
            depthPID.SetTunings(Kp, Ki, Kd);
            Serial.println("PID updated.");
            waitingForInput = false;
            break;
        }
        inputBuffer = "";
        inputStage++;
      }
    } else {
      inputBuffer += c;
    }
  }
}
