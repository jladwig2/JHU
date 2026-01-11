#include <Wire.h>
#include <LiquidCrystal.h>

// I²C addresses
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

// these now store actual room IDs (2–6)
uint8_t currentPosition = 0;
uint8_t hauntedRoom     = 255;
uint8_t targetRoom      = 2;

// Rooms: index 0→ID 2, 1→ID 3, …, 4→ID 6
const char* roomNames[]  = { "Tv", "Pr", "St", "Kt", "Bd" };
const uint8_t NUM_ROOMS  = sizeof(roomNames)/sizeof(roomNames[0]);

// dynamic list of available (unfixed) room IDs
uint8_t availableRooms[NUM_ROOMS] = { 2, 3, 4, 5, 6 };
uint8_t numAvailable             = NUM_ROOMS;

unsigned long lastTick = 0;

// ——— Helpers ——————————————————————————————————————

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
    Wire.write((uint8_t)0x00);  // msgType = control
    Wire.write(payload);        // 0x01=start, 0x02=game over
    Wire.endTransmission();
    delay(20);
  }
}

void pollMotorPosition() {
  Wire.beginTransmission(MOTOR_CTRL_ADDR);
  Wire.write((uint8_t)0x01);
  Wire.endTransmission();
  delay(5);
  Wire.requestFrom((uint8_t)MOTOR_CTRL_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint8_t mt = Wire.read();
    uint8_t v  = Wire.read();
    if (mt == 0x01) currentPosition = v;
  }
}

void pollHauntedRoom() {
  Wire.beginTransmission(GAME_CTRL_ADDR);
  Wire.write((uint8_t)0x02);
  Wire.endTransmission();
  delay(5);
  Wire.requestFrom((uint8_t)GAME_CTRL_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint8_t mt = Wire.read();
    uint8_t v  = Wire.read();
    if (mt == 0x02 && v >= 2 && v < 2 + NUM_ROOMS) hauntedRoom = v;
  }
}

void waitForRecalibration() {
  while (true) {
    Wire.requestFrom((uint8_t)MOTOR_CTRL_ADDR, (uint8_t)1);
    if (Wire.available() && Wire.read() == 0x03) break;
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
  Wire.write((uint8_t)0x03);  // msgType = room fixed
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

// ——— Core ——————————————————————————————————————

void setup() {
  Wire.begin();            // I²C master
  Serial.begin(115200);
  lcd.begin(16,2);
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin),
                  notifyFixing, FALLING);
  randomSeed(analogRead(A0));
  assignNewTarget();
  Introduction();
  sendControl(0x01);      // game start
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
      sendControl(0x02);       // notify game over
      waitForRecalibration();  // wait motor
      // reset for next round
      timeRemaining = 205;
      HP            = 5;
      gameOverFlag  = false;
      // refill rooms
      for (uint8_t i = 0; i < NUM_ROOMS; i++)
        availableRooms[i] = i + 2;
      numAvailable = NUM_ROOMS;
      assignNewTarget();
      Introduction();
      sendControl(0x01);       // notify start
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
      // if none left → congratulations + restart cycle
      if (numAvailable == 1) {
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("CONGRATS!");
        lcd.setCursor(0,1); lcd.print("All fixed!");
        delay(3000);
        sendControl(0x02);       // notify game over
        waitForRecalibration();  // wait motor
        // reset all
        timeRemaining = 205;
        HP            = 5;
        gameOverFlag  = false;
        for (uint8_t i = 0; i < NUM_ROOMS; i++)
          availableRooms[i] = i + 2;
        numAvailable = NUM_ROOMS;
        assignNewTarget();
        Introduction();
        sendControl(0x01);       // notify start
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
