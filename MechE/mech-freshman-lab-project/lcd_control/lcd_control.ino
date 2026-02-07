#include <Wire.h>
#include <LiquidCrystal.h>

// Command Message Types
#define MSG_CMD_CONTROL      0x00
#define MSG_CMD_LOCATION     0x01
#define MSG_CMD_HAUNTED      0x02
#define MSG_CMD_ROOM_FIXED   0x03

// Control Payloads
#define MSG_GAME_START       0x01
#define MSG_GAME_OVER        0x02

// Response Codes
#define MSG_RESP_LOCATION    0x01
#define MSG_RESP_HAUNTED     0x02
#define MSG_RESP_CALIBRATED  0x03

// I2C addresses
#define MOTOR_CTRL_ADDR 10
#define GAME_CTRL_ADDR  30

// LCD pins
LiquidCrystal lcd(10, 9, 4, 5, 6, 7);
const int buttonPin = 2;

// Game state
int  timeRemaining   = 205;
int  HP              = 5;
bool gameOverFlag    = false;
volatile bool buttonPressed = false;

// these store actual room IDs (2–6)
uint8_t currentPosition = 0;
uint8_t hauntedRoom     = 255;
uint8_t targetRoom      = 2;

// Rooms: index 0 ID 2, 1 ID 3, …, 4 ID 6
const char* roomNames[]  = { "Tv", "Pr", "St", "Kt", "Bd" };
const uint8_t NUM_ROOMS  = sizeof(roomNames)/sizeof(roomNames[0]);

// dynamic list of available (unfixed) room IDs
uint8_t availableRooms[NUM_ROOMS] = { 2, 3, 4, 5, 6 };
uint8_t numAvailable             = NUM_ROOMS;

unsigned long lastTick = 0;

void setup() {
  Wire.begin();            
  Serial.begin(115200);
  lcd.begin(16,2);
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin),
                  notifyFixing, FALLING);
  randomSeed(analogRead(A0));
  assignNewTarget();
  Introduction();
  sendControl(MSG_GAME_START);
  lastTick = millis();
  displayStatus();
}

void loop() {
  unsigned long now = millis();
  if (now - lastTick >= 1000) {
    lastTick += 1000;
    pollMotorPosition();
    pollHauntedRoom();
    if (timeRemaining > 0 && HP > 0) {
      timeRemaining--;
      if (currentPosition == hauntedRoom)
        HP = max(0, HP - 1);
      displayStatus();
    }
    else if (!gameOverFlag) {
      // Game over
      gameOverFlag = true;
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("THE POLTERGEIST");
      lcd.setCursor(0,1); lcd.print("GOT YOU.");
      delay(2000);
      sendControl(MSG_GAME_OVER);
      waitForRecalibration();  // wait motor
      // reset for next round
      timeRemaining = 205;
      HP            = 5;
      gameOverFlag  = false;
      hauntedRoom   = 255;
      // refill rooms
      for (uint8_t i = 0; i < NUM_ROOMS; i++)
        availableRooms[i] = i + 2;
      numAvailable = NUM_ROOMS;
      assignNewTarget();
      Introduction();
      sendControl(MSG_GAME_START);
      displayStatus();
    }
  }

  // handle “Room Fixed”
  if (buttonPressed) {
    if (currentPosition == targetRoom &&
        targetRoom != hauntedRoom) {
      // remove from availableRooms[]
      for (uint8_t i = 0; i < numAvailable; i++) {
        if (availableRooms[i] == targetRoom) {
          for (uint8_t j = i; j + 1 < numAvailable; j++)
            availableRooms[j] = availableRooms[j+1];
          numAvailable--;
          break;
        }
      }
      // if only one room left, congratulations + restart cycle
      if (numAvailable == 1) {
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("CONGRATS!");
        lcd.setCursor(0,1); lcd.print("All fixed!");
        delay(3000);
        sendControl(MSG_GAME_OVER);
        waitForRecalibration();  // wait motor
        // reset all
        timeRemaining = 205;
        HP            = 5;
        gameOverFlag  = false;
        hauntedRoom   = 255;
        for (uint8_t i = 0; i < NUM_ROOMS; i++)
          availableRooms[i] = i + 2;
        numAvailable = NUM_ROOMS;
        assignNewTarget();
        Introduction();
        sendControl(MSG_GAME_START);
        displayStatus();
      } else {
        // normal “fixed” flow
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("Room Fixed:");
        lcd.setCursor(0,1);
        lcd.print(roomNames[targetRoom - 2]);
        notifyGameControlRoomFixed();
        delay(500);
        assignNewTarget();
        displayStatus();
      }
    }
    buttonPressed = false;
  }
}

void Introduction() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("WELCOME TO");
  lcd.setCursor(0,1); lcd.print("POLTERGEIST");
  delay(3000);
}

void sendControl(uint8_t payload) {
  const uint8_t addrs[2] = { MOTOR_CTRL_ADDR, GAME_CTRL_ADDR };
  for (uint8_t i = 0; i < 2; i++) {
    Wire.beginTransmission(addrs[i]);
    Wire.write((uint8_t)MSG_CMD_CONTROL);
    Wire.write(payload);
    Wire.endTransmission();
    delay(20);
  }
}

void pollMotorPosition() {
  Wire.beginTransmission(MOTOR_CTRL_ADDR);
  Wire.write((uint8_t)MSG_CMD_LOCATION);
  Wire.endTransmission();
  delay(5);
  Wire.requestFrom((uint8_t)MOTOR_CTRL_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint8_t mt = Wire.read();
    uint8_t v  = Wire.read();
    if (mt == MSG_RESP_LOCATION) currentPosition = v;
  }
}

void pollHauntedRoom() {
  Wire.beginTransmission(GAME_CTRL_ADDR);
  Wire.write((uint8_t)MSG_CMD_HAUNTED);
  Wire.endTransmission();
  delay(10);
  Wire.requestFrom((uint8_t)GAME_CTRL_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint8_t mt = Wire.read();
    uint8_t v  = Wire.read();
    if (mt == MSG_RESP_HAUNTED && v >= 2 && v < 2 + NUM_ROOMS) hauntedRoom = v;
  }
}

void waitForRecalibration() {
  while (true) {
    Wire.requestFrom((uint8_t)MOTOR_CTRL_ADDR, (uint8_t)1);
    if (Wire.available() && Wire.read() == MSG_RESP_CALIBRATED) break;
    delay(50);
  }
}

void notifyFixing() {
  buttonPressed = true;
}

// pick next target from availableRooms[]
void assignNewTarget() {
  if (numAvailable == 0) {
    targetRoom = 0;
    return;
  }
  targetRoom = availableRooms[random(numAvailable)];
}

void notifyGameControlRoomFixed() {
  Wire.beginTransmission(GAME_CTRL_ADDR);
  Wire.write((uint8_t)MSG_CMD_ROOM_FIXED);
  Wire.write(targetRoom);
  Wire.endTransmission();
}

void displayStatus() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TIME:"); lcd.print(timeRemaining);
  lcd.setCursor(9,0);
  lcd.print("ROOM:");
  if (targetRoom >= 2 && targetRoom < 2 + NUM_ROOMS)
    lcd.print(roomNames[targetRoom - 2]);
  else
    lcd.print("--");
  lcd.setCursor(0, 1);
  lcd.print("HP:");
  for (int i = 0; i < HP; i++) lcd.print('+');
}
