#include <LiquidCrystal.h>
#include "dac.h"  // Include DAC library for stepper motor control

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
const long countsPerFloor = 3900 * 2;
volatile int lastEncoded = 0;

// Speed settings
int baseSpeed = 180;
int slowSpeed = 0;

// Stepper motor door control variables
const int phaseA_control = 66;
const int enableA = 67;
const int phaseB_control = 68;
const int enableB = 69;
const int halfTurnSteps = 25; // Adjusted to ensure motor moves either half a turn or 1.5 turns
const int stepDelay = 7; // Delay between steps for smooth movement

// Door states
enum DoorState { CLOSED, HALF, OPEN };
DoorState doorState = CLOSED;

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
    attachInterrupt(digitalPinToInterrupt(encA), updateEncoder, CHANGE); // Ensure function is properly declared
    attachInterrupt(digitalPinToInterrupt(encB), updateEncoder, CHANGE);

    pinMode(confirmButton, INPUT_PULLUP);

    // Initialize stepper motor pins for the door control
    dac_init();
    set_dac(4095, 4095); // Set DAC for max reference voltage

    pinMode(phaseA_control, OUTPUT);
    pinMode(enableA, OUTPUT);
    pinMode(phaseB_control, OUTPUT);
    pinMode(enableB, OUTPUT);

    // Enable H-bridges for the stepper motor
    digitalWrite(enableA, HIGH);
    digitalWrite(enableB, HIGH);

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
    updateFloorSelectionDisplay(); // Ensure function is properly declared
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
        closeDoor(); // Ensure function is properly declared
        moveToFloor(selectedFloor); // Ensure function is properly declared
        isChoosingDirection = false;
        isWaitingForDestination = true;
        openDoor();  // Open door upon reaching the user
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Select destination");
    }

    // Step 4: Waiting for Destination Selection
    else if (isWaitingForDestination) {
        listenForFloorButtonPress(); // Ensure function is properly declared
        if (targetFloor != -1 && targetFloor != currentFloor) {
            closeDoor(); // Ensure function is properly declared
            isWaitingForDestination = false;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Moving to Floor");
            lcd.setCursor(0, 1);
            lcd.print(targetFloor);
            moveToFloor(targetFloor); // Ensure function is properly declared

            openDoor(); // Open door upon reaching the target floor

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

// Door Control Functions
void openDoor() {
    if (doorState == CLOSED) {
        stepMotor(true, halfTurnSteps); // Fully open the door
        doorState = OPEN;
    } else if (doorState == HALF) {
        stepMotor(true, halfTurnSteps); // Move from HALF to OPEN
        doorState = OPEN;
    }
    unsigned long doorOpenTime = millis(); // Record the time when the door opens
    while (millis() - doorOpenTime < 10000) {
        delay(10); // Allow other code to run while waiting
    }
    closeDoor();
}
    

void closeDoor() {
    if (doorState == OPEN) {
        stepMotor(false, halfTurnSteps * 1); // Fully close the door (1.5 turns)
        doorState = CLOSED;
    } else if (doorState == HALF) {
        stepMotor(false, halfTurnSteps); // Move from HALF to CLOSED
        doorState = CLOSED;
    }
}

void halfOpenDoor() {
    if (doorState == CLOSED) {
        stepMotor(true, halfTurnSteps); // Move from CLOSED to HALF
        doorState = HALF;
    } else if (doorState == OPEN) {
        stepMotor(false, halfTurnSteps); // Move from OPEN to HALF
        doorState = HALF;
    }
}

void stepMotor(bool direction, int steps) {
    for (int i = 0; i < steps; i++) {
        if (direction) {
            // Step 1
            digitalWrite(phaseA_control, HIGH);
            digitalWrite(phaseB_control, LOW);
            delay(stepDelay);

            // Step 2
            digitalWrite(phaseA_control, LOW);
            digitalWrite(phaseB_control, LOW);
            delay(stepDelay);

            // Step 3
            digitalWrite(phaseA_control, LOW);
            digitalWrite(phaseB_control, HIGH);
            delay(stepDelay);

            // Step 4
            digitalWrite(phaseA_control, HIGH);
            digitalWrite(phaseB_control, HIGH);
            delay(stepDelay);
        } else {
            // Reverse sequence for closing the door
            digitalWrite(phaseA_control, HIGH);
            digitalWrite(phaseB_control, HIGH);
            delay(stepDelay);

            digitalWrite(phaseA_control, LOW);
            digitalWrite(phaseB_control, HIGH);
            delay(stepDelay);

            digitalWrite(phaseA_control, LOW);
            digitalWrite(phaseB_control, LOW);
            delay(stepDelay);

            digitalWrite(phaseA_control, HIGH);
            digitalWrite(phaseB_control, LOW);
            delay(stepDelay);
        }
    }
}

// Encoder Interrupt Function
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

// Function to Update Floor Selection Display
void updateFloorSelectionDisplay() {
    lcd.setCursor(0, 1);
    lcd.print("Floor: ");
    lcd.print(selectedFloor);
}

// Move Elevator to Requested Floor
void moveToFloor(int requestedFloor) {
    if (doorState != CLOSED) {
        closeDoor(); // Ensure the door is closed before moving
    }
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

// Function to Listen for Floor Button Presses
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

// Update LED Indicators for Current Floor
void updateFloorIndicators(int floor) {
    for (int i = 0; i < numFloors; i++) {
        digitalWrite(ledPins[i], LOW);
    }
    digitalWrite(ledPins[floor], HIGH);
    displayFloor(floor);
}

// Display Current Floor on LCD
void displayFloor(int floor) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("At Floor:");
    lcd.setCursor(0, 1);
    lcd.print(floor);
}

// Set Motor Speed in a Given Direction
void setMotorSpeed(int direction, int speed) {
    speed = constrain(speed, 0, 255);
    int adjustedSpeed = (direction > 0) ? map(speed, 0, 255, 125, 0) : map(speed, 0, 255, 129, 255);
    analogWrite(pwmPin, adjustedSpeed);
}

// Stop Motor Function
void stopMotor() {
    analogWrite(pwmPin, 127);
    digitalWrite(enablePin, LOW);
}
