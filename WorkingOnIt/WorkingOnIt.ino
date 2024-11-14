#include <LiquidCrystal.h>

// LCD and Joystick setup
LiquidCrystal lcd(41, 40, 37, 36, 35, 34);
const int joystickX = A1;
const int joystickY = A2;
const int confirmButton = 2;

// Elevator control variables
const int numFloors = 8;  // Floors 0 through 7
int currentFloor = 0;     // Elevator starts at floor 0
int selectedFloor = 0;    // The floor the user is on
int targetFloor = -1;     // The final destination floor
bool isSelectingFloor = true;
bool isChoosingDirection = false;
bool isMovingToUserFloor = false;
bool isWaitingForDestination = false;
bool isWaitingForCall = false;  // New flag to indicate idle state after reaching destination
int direction = 0;              // 1 for Up, -1 for Down
int buttonPins[numFloors] = { 22, 23, 24, 25, 26, 27, 28, 29 };
int ledPins[numFloors] = { 49, 48, 47, 46, 45, 44, 43, 42 };

// Motor and encoder configuration
const int enablePin = 7;
const int pwmPin = 6;
const int encA = 20;
const int encB = 21;
volatile long encCount = 0;
const long countsPerFloor = 3900*2;
volatile int lastEncoded = 0;

// Speed settings
int baseSpeed = 254;
int slowSpeed = 0;

void setup() {
  lcd.begin(16, 2);
  Serial.begin(9600);

  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  // Initialize button and LED pins
  for (int i = 0; i < numFloors; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }

  // Initialize motor and encoder setup
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);
  pinMode(pwmPin, OUTPUT);
  analogWrite(pwmPin, 127);

  encCount = 0;
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB), updateEncoder, CHANGE);

  pinMode(confirmButton, INPUT_PULLUP);

  // Set initial state: Start with waiting for a call
  isWaitingForCall = true;
  isSelectingFloor = false;
  isChoosingDirection = false;
  isMovingToUserFloor = false;
  isWaitingForDestination = false;

  // Display the initial prompt
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Select your floor:");
  updateFloorSelectionDisplay();
}

void loop() {
  static String lastDirectionPrompt = "";  // To store the last prompt message

  // Step 1: Waiting for a New Call
  if (isWaitingForCall) {
    // Display "Select your" on the first line and "floor: " on the second line
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Select your");
    lcd.setCursor(0, 1);
    lcd.print("floor: ");
    lcd.print(selectedFloor); // Display the currently selected floor number

    // Joystick-based floor selection
    int xValue = analogRead(joystickX);
    if (xValue > 800 && selectedFloor < numFloors - 1) {
      selectedFloor++;
      delay(200);
    } else if (xValue < 200 && selectedFloor > 0) {
      selectedFloor--;
      delay(200);
    }

    // Continuously update the selected floor display on the second line
    lcd.setCursor(7, 1); // Move cursor to update only the floor number part
    lcd.print(selectedFloor);

    // Confirm selection
    if (digitalRead(confirmButton) == HIGH) {
      delay(200);
      isWaitingForCall = false;          // Exit waiting state
      isSelectingFloor = true;           // Begin direction selection process
      lastDirectionPrompt = "";          // Reset the prompt tracker
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Choose Up or Down");
    }

    // Add a 10ms delay to slow down loop in Stage 1
    delay(10);
  }

  // Step 2: Direction Selection
  else if (isSelectingFloor) {
    String currentPrompt;

    // Determine available direction based on selected floor
    if (selectedFloor == 0) {
      currentPrompt = "Move: Up";  // Only Up option at floor 0
    } else if (selectedFloor == 7) {
      currentPrompt = "Move: Down";  // Only Down option at floor 7
    } else {
      currentPrompt = "Move: Up or Down";  // Both options available on floors 1 to 6
    }

    // Display the prompt only if it has changed
    if (currentPrompt != lastDirectionPrompt) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(currentPrompt);
      lastDirectionPrompt = currentPrompt;
    }

    // Handle joystick input for direction
    int yValue = analogRead(joystickY);
    if (yValue > 800 && selectedFloor < 7) {  // Up option, if not at top floor
      direction = 1;
      isSelectingFloor = false;  // Exit selection state
      isChoosingDirection = true;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Calling elevator Up");
    } else if (yValue < 200 && selectedFloor > 0) {  // Down option, if not at bottom floor
      direction = -1;
      isSelectingFloor = false;  // Exit selection state
      isChoosingDirection = true;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Calling elevator Down");
    }
  }

  // Step 3: Moving to User Floor
  else if (isChoosingDirection) {
    moveToFloor(selectedFloor);
    isChoosingDirection = false;
    isWaitingForDestination = true;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Select destination");
  }

  // Step 4: Waiting for Destination Selection
  else if (isWaitingForDestination) {
    listenForFloorButtonPress();
    if (targetFloor != -1 && targetFloor != currentFloor) {
      isWaitingForDestination = false;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Moving to Floor");
      lcd.setCursor(0, 1);
      lcd.print(targetFloor);
      moveToFloor(targetFloor);

      // Set the elevator to wait for a new call after reaching the target floor
      isWaitingForCall = true;
      selectedFloor = currentFloor;
      targetFloor = -1;
      direction = 0;
      lastDirectionPrompt = "";
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Waiting for call");
    }
  }
}


// Auxiliary Functions for Display and Movement
void updateFloorSelectionDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Select floor:");
  lcd.print(selectedFloor);
  delay(10);
}

void moveToFloor(int requestedFloor) {
  if (requestedFloor != currentFloor) {
    digitalWrite(enablePin, HIGH);
    int direction = (requestedFloor > currentFloor) ? 1 : -1;
    long targetCount = countsPerFloor * abs(requestedFloor - currentFloor);
    setMotorSpeed(direction, baseSpeed);

    encCount = 0;  // Reset encoder count before movement starts
    while (abs(encCount) < targetCount) {
      int floorsPassed = abs(encCount) / countsPerFloor;
      int currentPassedFloor = currentFloor + (floorsPassed * direction);

      if (currentPassedFloor != currentFloor) {
        updateFloorIndicators(currentPassedFloor);
      }

      delay(1);
    }

    stopMotor();
    currentFloor = requestedFloor;  // Update current floor after reaching the destination
    updateFloorIndicators(currentFloor);
    displayFloor(currentFloor);
  }
}

void listenForFloorButtonPress() {
  for (int i = 0; i < numFloors; i++) {
    if (digitalRead(buttonPins[i]) == HIGH) {
      targetFloor = i;
      digitalWrite(ledPins[i], HIGH);
      Serial.print("Selected destination floor: ");
      Serial.println(targetFloor);
      return;
    }
  }
}

void updateFloorIndicators(int floor) {
  for (int i = 0; i < numFloors; i++) {
    digitalWrite(ledPins[i], LOW);
  }
  digitalWrite(ledPins[floor], HIGH);
  displayFloor(floor);
}

void displayFloor(int floor) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("At Floor:");
  lcd.setCursor(0, 1);
  lcd.print(floor);
}

void setMotorSpeed(int direction, int speed) {
  speed = constrain(speed, 0, 255);
  int adjustedSpeed = (direction > 0) ? map(speed, 0, 255, 125, 0) : map(speed, 0, 255, 129, 255);
  analogWrite(pwmPin, adjustedSpeed);
}

void stopMotor() {
  analogWrite(pwmPin, 127);
  digitalWrite(enablePin, LOW);
}

void updateEncoder() {
  int MSB = digitalRead(encA);  // Most Significant Bit
  int LSB = digitalRead(encB);  // Least Significant Bit

  int encoded = (MSB << 1) | LSB;          // Combine the bits into a single value
  int sum = (lastEncoded << 2) | encoded;  // Shift the previous state and add the new one

  // Determine direction based on state changes
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encCount--;

  lastEncoded = encoded;  // Store this state for the next time
}
