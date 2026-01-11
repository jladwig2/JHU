#include <AccelStepper.h>
#include <avr/pgmspace.h>  // For PROGMEM support
#include <Wire.h>

#define MOTOR_CTRL_ADDR 10
#define MOTOR_INTERFACE_TYPE AccelStepper::DRIVER
#define VRX_PIN A1
#define VRY_PIN A0

uint8_t lastCmd = 0, lastPayload = 0;
bool gameActive     = false;  // only move when true
bool calibrateDone  = false;  // set true once recalibration finishes
bool gotGameOver = false;

const int trigPin = 8;
const int echoPin = 9; 

// Define pins used for the two step/direction drivers
const int rightStep = 3;
const int rightDir  = 2;
const int leftStep  = 5;
const int leftDir   = 4;
int xIn, yIn;
long leftTarget, rightTarget;
char move;
char blockedX = 'N';
char blockedY = 'N';
uint8_t locationVal;

// Global grid “player” position.
uint8_t currentX = 0;
uint8_t currentY = 0;

// ---------- GRID SETUP ----------
// Stored as bytes in flash.
const uint8_t gridWidth = 21;
const uint8_t gridHeight = 28;
const uint8_t gridData[] PROGMEM = {
  5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6,
  5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6,
  5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6,
  5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 6, 6, 6, 6,
  1, 1, 1, 1, 1, 1, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 1, 6, 6, 6, 6,
  1, 1, 1, 1, 1, 1, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 1, 6, 6, 6, 6,
  1, 1, 1, 1, 1, 1, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 1, 6, 6, 6, 6, 
  1, 1, 1, 1, 1, 1, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 1, 6, 6, 6, 6, 
  1, 1, 1, 1, 1, 1, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 1, 6, 6, 6, 6,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4, 4, 4, 4, 4, 1, 6, 6, 6, 6,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4, 4, 4, 4, 4, 1, 6, 6, 6, 6, 
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4, 4, 4, 4, 4, 1, 6, 6, 6, 6, 
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4, 4, 4, 4, 4, 1, 6, 6, 6, 6,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3,
  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
  2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
};

// Create two AccelStepper objects
AccelStepper rightStepper(MOTOR_INTERFACE_TYPE, rightStep, rightDir);
AccelStepper leftStepper(MOTOR_INTERFACE_TYPE, leftStep, leftDir);

char lastHorizontalMove;
const int moveSteps = 100;

void setup() {
  Wire.begin(MOTOR_CTRL_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  Serial.begin(921600);

  pinMode(rightStep, OUTPUT);
  pinMode(rightDir, OUTPUT);
  pinMode(leftStep, OUTPUT);
  pinMode(leftDir, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  leftStepper.setMaxSpeed(1000);
  leftStepper.setSpeed(0);
  rightStepper.setMaxSpeed(1000);
  rightStepper.setSpeed(0);

  lastHorizontalMove = 'N';

  //printGrid();
}

void loop() {
  if (!gameActive) {
    if (gotGameOver) {
      resetMotor();     // drive back to (0,0)
      Serial.println("Motor returned to bottom left");
      reCalibrate();    // ultrasonic zeroing
      gotGameOver = false;
      calibrateDone = true;
    } else {
      return;
    }
  } 
  // Read the joystick values.
  xIn = analogRead(VRX_PIN);
  yIn = -1 * analogRead(VRY_PIN) + 1023;
  //Serial.print("xIn: " );
  //Serial.println(xIn);
  //Serial.print("yIn: ");
  //Serial.println(yIn);

  if (abs(xIn - 512) > abs(yIn - 512)) {
    if (xIn < 200 && blockedX != 'L') {
      if (lastHorizontalMove != 'R') {
        leftTarget = leftStepper.currentPosition() - moveSteps;
        rightTarget = rightStepper.currentPosition() - moveSteps;
        move = 'L';
      } else {
        leftTarget = leftStepper.currentPosition() - moveSteps - 25;
        rightTarget = rightStepper.currentPosition() - moveSteps - 25;
        move = 'L';
      }
      
    } else if (xIn > 800 && blockedX != 'R') {
      if (lastHorizontalMove != 'L') {
        leftTarget = leftStepper.currentPosition() + moveSteps;
        rightTarget = rightStepper.currentPosition() + moveSteps;
        move = 'R';
      } else {
        leftTarget = leftStepper.currentPosition() + moveSteps + 25;
        rightTarget = rightStepper.currentPosition() + moveSteps + 25;
        move = 'R';
      }
      
    } else {
      leftTarget = leftStepper.currentPosition();
      rightTarget = rightStepper.currentPosition();
      move = 'N';
    }
  } else {
    if (yIn < 200 && blockedY != 'D') {
      leftTarget = leftStepper.currentPosition() - moveSteps;
      rightTarget = rightStepper.currentPosition() + moveSteps;
      move = 'D';
    } else if (yIn > 800 && blockedY != 'U') {
      leftTarget = leftStepper.currentPosition() + moveSteps;
      rightTarget = rightStepper.currentPosition() - moveSteps;
      move = 'U';
    } else {
      leftTarget = leftStepper.currentPosition();
      rightTarget = rightStepper.currentPosition();
      move = 'N';
    }
  }

  leftStepper.moveTo(leftTarget);
  rightStepper.moveTo(rightTarget);

  leftStepper.setSpeed((leftStepper.targetPosition() - leftStepper.currentPosition()) / 0.225);
  rightStepper.setSpeed((rightStepper.targetPosition() - rightStepper.currentPosition()) / 0.225);

  // Run the steppers until they reach their targets.
  while (abs(leftStepper.distanceToGo()) > 0 || abs(rightStepper.distanceToGo()) > 0) {
    if (abs(leftStepper.distanceToGo()) > 0 && abs(rightStepper.distanceToGo()) > 0) {
      leftStepper.runSpeed();
      rightStepper.runSpeed();
    } else {
      rightStepper.moveTo(rightStepper.currentPosition());
      leftStepper.moveTo(leftStepper.currentPosition());
    } 
  }

  updatePosition(move);

  if (move == 'R' || move =='L') {
    lastHorizontalMove = move;
  }
}

void onI2CReceive(int count) {
  if (count < 1) return;
  lastCmd = Wire.read();
  if (Wire.available()) lastPayload = Wire.read();

  if (lastCmd == 0x00) {
    // control message
    if (lastPayload == 0x02) {
      // game over → recalibrate
      gameActive    = false;
      gotGameOver = true;
    }
    else if (lastPayload == 0x01) {
      // game start
      gameActive    = true;
      gotGameOver = false;
      calibrateDone = false;
    }
  }
  // 0x01 = location request -> handled in onI2CRequest
}

// Called when LCD does requestFrom(MOTOR_CTRL_ADDR, …)
void onI2CRequest() {
  switch (lastCmd) {
    case 0x00: 
      //Serial.println("Poll for calibration received");
      // master is asking “have you finished recalibrating?”
      Wire.write(calibrateDone ? (uint8_t)0x03 : (uint8_t)0x02);
      break;

    case 0x01:
      // master is asking for the magnet’s location
      //Serial.println("Poll for location received");
      Wire.write((uint8_t)0x01);      // message type
      Wire.write(locationVal);        // latest cell value
      break;

    default:
      Wire.write((uint8_t)0xFF);      // “unknown”
  }
}

// Given a logical Y coordinate (0 = bottom), return the physical row index (0 = top).
uint8_t physicalRow(uint8_t logicalY) {
  return gridHeight - 1 - logicalY;
}

// Print the grid to Serial, marking the current position.
// We use the F() macro for constant strings to reduce RAM usage.
void printGrid() {
  Serial.println(F("Grid (logical view):"));
  // Loop over physical rows from 0 (top) to gridHeight - 1 (bottom)
  for (uint8_t i = 0; i < gridHeight; i++) {
    // Convert physical row i to logical Y: logicalY = gridHeight - 1 - i
    uint8_t logicalY = gridHeight - 1 - i;
    for (uint8_t j = 0; j < gridWidth; j++) {
      if (logicalY == currentY && j == currentX)
        Serial.print(F("X "));
      else {
        uint8_t cell = pgm_read_byte_near(gridData + i * gridWidth + j);
        Serial.print(cell);
        Serial.print(F(" "));
      }
    }
    Serial.println();
  }
}

void updatePosition(char cmd) {
  int newX = currentX;
  int newY = currentY;
  
  // Update logical coordinates: 
  // 'L': decrease X; 'R': increase X; 'U': increase Y; 'D': decrease Y.
  if (cmd == 'L') newX--;
  else if (cmd == 'R') newX++;
  else if (cmd == 'U') newY++;  
  else if (cmd == 'D') newY--;
  
  // Clamp logical coordinates.
  if (newX < 0) newX = 0;
  if (newX >= gridWidth) newX = gridWidth - 1;
  if (newY < 0) newY = 0;
  if (newY >= gridHeight) newY = gridHeight - 1;
  
  // Update global position (logical).
  currentX = newX;
  currentY = newY;
  
  // Convert the logical Y to a physical row in gridData.
  uint8_t physRow = physicalRow(currentY);
  uint8_t cellVal = pgm_read_byte_near(gridData + physRow * gridWidth + currentX);

  //Serial.print(F("Updated grid (logical pos): "));
  //Serial.print(currentX);
  //Serial.print(F(","));
  //Serial.print(currentY);
  //Serial.print(F(" - Physical cell value: "));
  //Serial.println(cellVal);

  locationVal = cellVal;
  
  //printGrid();

  blockedY = 'N'; // 'U' if up is blocked, 'D' if down is blocked, else 'N'
  blockedX = 'N'; // 'L' if left is blocked, 'R' if right is blocked, else 'N'

  // --- Check Vertical Neighbors ---
  // For logical UP, we check (currentY + 1). In our grid, the top logical row is gridHeight - 1.
  if (currentY == gridHeight - 1) {
    blockedY = 'U';  // Cannot move up if already at the top logical row.
  } else {
    // Convert the logical row above (currentY + 1) to the physical index.
    uint8_t physRowAbove = physicalRow(currentY + 1);
    if (pgm_read_byte_near(gridData + (physRowAbove * gridWidth + currentX)) == 0)
      blockedY = 'U';
  }

  // For logical DOWN, we check (currentY - 1). The bottom logical row is 0.
  if (currentY == 0) {
    blockedY = 'D';  // Cannot move down if already at the bottom.
  } else {
    // Convert the logical row below (currentY - 1) to its physical index.
    uint8_t physRowBelow = physicalRow(currentY - 1);
    if (pgm_read_byte_near(gridData + (physRowBelow * gridWidth + currentX)) == 0)
      blockedY = 'D';
  }

  // --- Check Horizontal Neighbors ---
  // For LEFT, check if at leftmost edge or if the cell to the left is blocked.
  if (currentX == 0) {
    blockedX = 'L';
  } else {
    // Use physical row conversion for the current logical row.
    uint8_t physRowCurrent = physicalRow(currentY);
    if (pgm_read_byte_near(gridData + (physRowCurrent * gridWidth + (currentX - 1))) == 0)
      blockedX = 'L';
  }

  // For RIGHT, check if at the rightmost edge or if the cell to the right is blocked.
  if (currentX == gridWidth - 1) {
    blockedX = 'R';
  } else {
    uint8_t physRowCurrent = physicalRow(currentY);
    if (pgm_read_byte_near(gridData + (physRowCurrent * gridWidth + (currentX + 1))) == 0)
      blockedX = 'R';
  }
}

void resetMotor() {
  while (currentY != 0) {
    Serial.print("Resetting motor");
    leftTarget = leftStepper.currentPosition() - moveSteps;
    rightTarget = rightStepper.currentPosition() + moveSteps;

    leftStepper.moveTo(leftTarget);
    rightStepper.moveTo(rightTarget);

    leftStepper.setSpeed((leftStepper.targetPosition() - leftStepper.currentPosition()) / 0.225);
    rightStepper.setSpeed((rightStepper.targetPosition() - rightStepper.currentPosition()) / 0.225);

    // Run the steppers until they reach their targets.
    while (abs(leftStepper.distanceToGo()) > 0 || abs(rightStepper.distanceToGo()) > 0) {
      if (abs(leftStepper.distanceToGo()) > 0 && abs(rightStepper.distanceToGo()) > 0) {
        leftStepper.runSpeed();
        rightStepper.runSpeed();
      } else {
        rightStepper.moveTo(rightStepper.currentPosition());
        leftStepper.moveTo(leftStepper.currentPosition());
      } 
    }

    currentY = currentY - 1;
  }

  while (currentX != 0) {
    leftTarget = leftStepper.currentPosition() - moveSteps;
    rightTarget = rightStepper.currentPosition() - moveSteps;

    leftStepper.moveTo(leftTarget);
    rightStepper.moveTo(rightTarget);

    leftStepper.setSpeed((leftStepper.targetPosition() - leftStepper.currentPosition()) / 0.225);
    rightStepper.setSpeed((rightStepper.targetPosition() - rightStepper.currentPosition()) / 0.225);

    // Run the steppers until they reach their targets.
    while (abs(leftStepper.distanceToGo()) > 0 || abs(rightStepper.distanceToGo()) > 0) {
      if (abs(leftStepper.distanceToGo()) > 0 && abs(rightStepper.distanceToGo()) > 0) {
        leftStepper.runSpeed();
        rightStepper.runSpeed();
      } else {
        rightStepper.moveTo(rightStepper.currentPosition());
        leftStepper.moveTo(leftStepper.currentPosition());
      } 
    }
    currentX = currentX - 1;
  }
}

void reCalibrate() {
  Serial.println("Calibration started");
  float distance = 10.0;
  while (distance > 4.2) {
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);

    distance = duration * 0.0343 / 2.;
    Serial.print("Distance:");
    Serial.print(distance);
    Serial.println();

    if (distance > 4.2) {
      leftTarget = leftStepper.currentPosition() - moveSteps;
      rightTarget = rightStepper.currentPosition() - moveSteps;

      leftStepper.moveTo(leftTarget);
      rightStepper.moveTo(rightTarget);

      leftStepper.setSpeed((leftStepper.targetPosition() - leftStepper.currentPosition()) / 0.225);
      rightStepper.setSpeed((rightStepper.targetPosition() - rightStepper.currentPosition()) / 0.225);

      // Run the steppers until they reach their targets.
      while (abs(leftStepper.distanceToGo()) > 0 || abs(rightStepper.distanceToGo()) > 0) {
        if (abs(leftStepper.distanceToGo()) > 0 && abs(rightStepper.distanceToGo()) > 0) {
          leftStepper.runSpeed();
          rightStepper.runSpeed();
        } else {
          rightStepper.moveTo(rightStepper.currentPosition());
          leftStepper.moveTo(leftStepper.currentPosition());
        } 
      }

      leftTarget = leftStepper.currentPosition() - moveSteps / 5;
      rightTarget = rightStepper.currentPosition() + moveSteps / 5;

      leftStepper.moveTo(leftTarget);
      rightStepper.moveTo(rightTarget);

      leftStepper.setSpeed((leftStepper.targetPosition() - leftStepper.currentPosition()) / 0.1);
      rightStepper.setSpeed((rightStepper.targetPosition() - rightStepper.currentPosition()) / 0.1);

      // Run the steppers until they reach their targets.
      while (abs(leftStepper.distanceToGo()) > 0 || abs(rightStepper.distanceToGo()) > 0) {
        if (abs(leftStepper.distanceToGo()) > 0 && abs(rightStepper.distanceToGo()) > 0) {
          leftStepper.runSpeed();
          rightStepper.runSpeed();
        } else {
          rightStepper.moveTo(rightStepper.currentPosition());
          leftStepper.moveTo(leftStepper.currentPosition());
        } 
      }
    } else {
      break;
    }
  } 
}


