#include <Servo.h>
#include <Wire.h>

// I²C address for this game-control board
#define GAME_CTRL_ADDR 30

// Room LED pins (indexed 0..4)
const int ledRoomPins[] = { 6, A3, A1, A2, A0 };
const uint8_t numRoomPins = sizeof(ledRoomPins) / sizeof(ledRoomPins[0]);

// "Hauntable" room IDs (2-6), will shrink when rooms are fixed
uint8_t hauntableRooms[] = { 2, 3, 4, 5, 6 };
uint8_t numHauntable     = sizeof(hauntableRooms) / sizeof(hauntableRooms[0]);
uint8_t NUM_ROOMS = 5;

// State
bool     gameActive         = false;
uint8_t  lastCmd            = 0;
uint8_t  lastPayload        = 0;
uint8_t  currentHauntedID   = 0;
bool     isHauntedSolid     = false;  // <-- only true during the solid-LED phase

// Phase & timings
enum Phase { PHASE1 = 1, PHASE2, PHASE3 };
Phase   phase        = PHASE3;
int     timeDangerMax = 5;

// Misc pins
const int rgbRed     = 10;
const int rgbGreen   = 11;
const int rgbBlue    = 12;
const int buzzerPin  = 9;
const int servoPin   = 13;
Servo     room5Servo;

// Motor A connections (for Room 3 & 5 effects)
const int enPins[2]  = { 3, 5 };
const int in1Pins[2] = { 2, 8 };
const int in2Pins[2] = { 4, 7 };

// Forward declarations
void resetAllEffects();
void allLED(int v);
void setRGBWhite();
void setRGBRed();
void runMotor(int which);
void stopMotor(int which);
void dangerLED(int roomIdx, int tMax);
void hauntedLED(int roomIdx);
void generateHaunt();

void setup() {
  Wire.begin(GAME_CTRL_ADDR);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  // LEDs
  for (uint8_t i = 0; i < numRoomPins; i++) pinMode(ledRoomPins[i], OUTPUT);
  pinMode(rgbRed,   OUTPUT);
  pinMode(rgbGreen, OUTPUT);
  pinMode(rgbBlue,  OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // Servo
  room5Servo.attach(servoPin);

  // Motor pins
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(enPins[i],  OUTPUT);
    pinMode(in1Pins[i], OUTPUT);
    pinMode(in2Pins[i], OUTPUT);
  }

  randomSeed(analogRead(A0));
}

void loop() {
  if (!gameActive) return;
  
  // Idle pause between haunt cycles
  resetAllEffects();
  allLED(0);
  delay(random(1000, 2000));

  // Single haunt per cycle; phase changes frequency
  switch (phase) {
    case PHASE1:
      timeDangerMax = 5;
      generateHaunt();
      break;

    case PHASE2:
      timeDangerMax = 10;
      for (int i = 0; i < 3 && gameActive; i++) {
        generateHaunt();
        delay(random(500, 2000));
      }
      break;

    case PHASE3:
      timeDangerMax = 5;
      for (uint8_t i = 0; i < numHauntable && gameActive; i++) {
        generateHaunt();
        delay(random(500, 2000));
      }
      break;
  }
}

// I²C: receive control or fixed-room notifications
void onReceive(int count) {
  if (count < 1) return;
  lastCmd     = Wire.read();
  lastPayload = (Wire.available() ? Wire.read() : 0);

  if (lastCmd == 0x00) {
    if (lastPayload == 0x01)      gameActive = true;
    else if (lastPayload == 0x02) {
      gameActive = false;
      for (uint8_t i = 0; i < NUM_ROOMS; i++) {
        hauntableRooms[i] = i + 2;    // since rooms are IDs 2..6
      }
      numHauntable = NUM_ROOMS;
      resetAllEffects();
    }
  }
  else if (lastCmd == 0x03) {
    // Room Fixed: remove from hauntable list
    for (uint8_t i = 0; i < numHauntable; i++) {
      if (hauntableRooms[i] == lastPayload) {
        for (uint8_t j = i; j + 1 < numHauntable; j++)
          hauntableRooms[j] = hauntableRooms[j+1];
        numHauntable--;
        break;
      }
    }
  }
}

// I²C: report current haunted ID only if in solid phase
void onRequest() {
  if (lastCmd == 0x02 && isHauntedSolid) {
    Wire.write((uint8_t)0x02);
    Wire.write(currentHauntedID);
  } else {
    Wire.write((uint8_t)0xFF);
  }
}

// Pick and enact one haunt event
void generateHaunt() {
  if (numHauntable == 0) return;
  if (numHauntable == 1) {
    isHauntedSolid = false;
    delay(1000);
  }
  uint8_t idx = random(numHauntable);
  uint8_t id  = hauntableRooms[idx];       // ID in [2..6]
  currentHauntedID = id;
  int roomIdx = id - 2;                    // map ID→0..4

  // Blinking phase: no haunted-room reported
  isHauntedSolid = false;
  dangerLED(roomIdx, timeDangerMax);

  // Solid phase: now reportable
  isHauntedSolid = true;
  hauntedLED(roomIdx);
}

void dangerLED(int roomIdx, int tMax) {
  for (int i = 0; i <= tMax && gameActive; i++) {
    analogWrite(ledRoomPins[roomIdx], 255);
    delay(100);
    analogWrite(ledRoomPins[roomIdx], 0);
    delay(100);
  }
}

void hauntedLED(int roomIdx) {
  analogWrite(ledRoomPins[roomIdx], 255);
  switch (roomIdx) {
    case 0: setRGBRed();         break;  // room ID 2
    case 3: runMotor(0);         break;  // room ID 3
    case 1: tone(buzzerPin,300); break;  // room ID 4
    case 4: runMotor(1);         break;  // room ID 5
    case 2: room5Servo.write(60);break;  // room ID 6
  }
}

void resetAllEffects() {
  for (uint8_t i = 0; i < numRoomPins; i++) 
    analogWrite(ledRoomPins[i], 0);
  setRGBWhite();
  for (int m = 0; m < 2; m++) 
    stopMotor(m);
  noTone(buzzerPin);
  room5Servo.write(0);



  // Clear solid-phase flag so we won't report until next solid
  isHauntedSolid = false;
}

void allLED(int v) {
  for (uint8_t i = 0; i < numRoomPins; i++) 
    analogWrite(ledRoomPins[i], v);
}

void setRGBWhite() {
  int r = constrain(220+random(-20,20),180,255);
  int g = constrain(220+random(-20,20),180,255);
  analogWrite(rgbRed,   r);
  analogWrite(rgbGreen, g);
  digitalWrite(rgbBlue, HIGH);
}

void setRGBRed() {
  digitalWrite(rgbRed,   HIGH);
  digitalWrite(rgbGreen, LOW);
  digitalWrite(rgbBlue,  LOW);
}

void runMotor(int which) {
  analogWrite(enPins[which], 255);
  digitalWrite(in1Pins[which], HIGH);
  digitalWrite(in2Pins[which], LOW);
  delay(400);
  digitalWrite(in1Pins[which], LOW);
  digitalWrite(in2Pins[which], HIGH);
  delay(400);
}

void stopMotor(int which) {
  digitalWrite(in1Pins[which], LOW);
  digitalWrite(in2Pins[which], LOW);
}
