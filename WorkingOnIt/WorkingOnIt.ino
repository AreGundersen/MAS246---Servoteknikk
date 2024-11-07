#include <LiquidCrystal.h>
// Elevator setup and motor control
const int numFloors = 8;
int buttonPins[numFloors] = {22, 23, 24, 25, 26, 27, 28, 29};
int ledPins[numFloors] = {49, 48, 47, 46, 45, 44, 43, 42};

// Motor and encoder configuration
const int enablePin = 7;       // Motor enable pin
const int pwmPin = 6;          // PWM pin for motor speed control
const int encA = 20;           // Encoder channel A
const int encB = 21;           // Encoder channel B
volatile long encCount = 0;    // Encoder count (1 round = 4211)
const long countsPerFloor = 4048 * 2; // Adjusted for 1 full rotation per floor

int currentFloor = 1;
volatile int lastEncoded = 0;  // Stores the previous state of the encoder
int lastPressedFloor = 1; // Initialize to a default floor, e.g., 1

// Configurable speed and deceleration settings
int baseSpeed = 254;         // Base speed for normal movement
int slowSpeedUp = 0;         // Minimum upward speed when decelerating to prevent sudden stop
int slowSpeedDown = 0;       // Minimum downward speed when decelerating to prevent sudden stop
float decelerationDistance = 1; // Controls how early deceleration starts (in floor counts)



// Initialize the LCD with the correct pin assignments
LiquidCrystal lcd(41, 40, 37, 36, 35, 34);

void setup() {

  pinMode(4, OUTPUT);  // Set pin 4 as output for display backlight control
  digitalWrite(4, HIGH);  // Turn on the backlight
  lcd.begin(16, 2);

  // Initialize button and LED pins
  for (int i = 0; i < numFloors; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);  // Start with LEDs off
  }
  updateFloorIndicators(currentFloor);

  // Motor setup
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);
  pinMode(pwmPin, OUTPUT);

  // Set motor PWM to 127 initially to keep it stationary
  analogWrite(pwmPin, 127);

  // Initialize encoder count to zero
  encCount = 0;

  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB), updateEncoder, CHANGE);

  Serial.begin(9600);
}

void loop() {
  // Debugging output to check encoder counts
  Serial.print("Encoder Count: ");
  Serial.println(encCount);

  // Check for button presses
  for (int i = 0; i < numFloors; i++) {
    if (digitalRead(buttonPins[i]) == HIGH) {  // Check against HIGH for button press
      lastPressedFloor = i + 1;  // Update to the last pressed floor
      Serial.print("Button pressed for floor ");
      Serial.println(lastPressedFloor);
      moveToFloor(lastPressedFloor);
      break;
    }
  }
}

void displayFloor(int floor) {
  // Clear the display before writing new content
  lcd.clear();
  
  // Set the cursor to the start of the first line
  lcd.setCursor(0, 0);
  lcd.print("Current Floor:");

  // Set the cursor to the start of the second line
  lcd.setCursor(0, 1);
  lcd.print(floor); // Display the floor number
}

void moveToFloor(int requestedFloor) {
  if (requestedFloor != currentFloor) {
    Serial.print("Moving to floor ");
    Serial.println(requestedFloor);

    // Enable the motor
    digitalWrite(enablePin, HIGH);

    int direction = (requestedFloor > currentFloor) ? 1 : -1;
    int totalFloorsToMove = abs(requestedFloor - currentFloor);
    long targetCount = countsPerFloor * totalFloorsToMove;  // Total encoder count for the whole movement
    int lastPassedFloor = currentFloor;

    // Set base speed based on distance to move
    baseSpeed = (totalFloorsToMove > 1) ? 80 : 100;  // Adjust baseSpeed here if needed
    setMotorSpeed(direction, baseSpeed);  // Start movement at base speed

    int currentSpeed = baseSpeed;  // Track current speed to adjust gradually

    while (abs(encCount) < targetCount) {
      // Debugging output for monitoring `encCount` and `targetCount`
      Serial.print("Current Encoder Count: ");
      Serial.print(encCount);
      Serial.print(" | Target Count: ");
      Serial.println(targetCount);

      int floorsPassed = abs(encCount) / countsPerFloor;
      int currentPassedFloor = currentFloor + (floorsPassed * direction);

      if (currentPassedFloor != lastPassedFloor) {
        lastPassedFloor = currentPassedFloor;
        updateFloorIndicators(currentPassedFloor);
        Serial.print("Passing floor ");
        Serial.println(currentPassedFloor);
      }

      // Calculate remaining counts and determine target speed
      long remainingCounts = targetCount - abs(encCount);
      int targetSpeed;
      if (remainingCounts < (decelerationDistance * countsPerFloor)) {
        if (direction > 0) {
          // Upward deceleration
          targetSpeed = map(remainingCounts, (decelerationDistance * countsPerFloor), 0, baseSpeed, slowSpeedUp);
          targetSpeed = constrain(targetSpeed, slowSpeedUp, baseSpeed);
        } else {
          // Downward deceleration
          targetSpeed = map(remainingCounts, (decelerationDistance * countsPerFloor), 0, baseSpeed, slowSpeedDown);
          targetSpeed = constrain(targetSpeed, slowSpeedDown, baseSpeed);
        }

        // Incrementally adjust currentSpeed towards targetSpeed
        if (currentSpeed < targetSpeed) {
          currentSpeed++;  // Gradually increase speed
        } else if (currentSpeed > targetSpeed) {
          currentSpeed--;  // Gradually decrease speed
        }
        
        setMotorSpeed(direction, currentSpeed);  // Apply the incrementally adjusted speed
      }

      delay(2); // Adjust delay for smoother updates if needed
    }

    stopMotor();
    encCount = 0;
    currentFloor = requestedFloor;
    updateFloorIndicators(currentFloor);

    // Update the floor display
    lcd.clear();  // Clear previous content
    lcd.setCursor(0, 0);  // Set cursor to the first row
    lcd.print("Current Floor: ");
    lcd.setCursor(0, 1);
    lcd.print(currentFloor);  // Display the current floor number

    Serial.print("Arrived at floor ");
    Serial.println(currentFloor);
  } else {
    Serial.println("Already at requested floor.");
  }
}

// Update floor indicators based on the current floor during movement
void updateFloorIndicators(int floor) {
  // Turn off all LEDs
  for (int i = 0; i < numFloors; i++) {
    digitalWrite(ledPins[i], LOW);
  }
  
  // Turn on LED for the last pressed button
  digitalWrite(ledPins[lastPressedFloor - 1], HIGH);
  
  // Display the current floor on the LCD
  displayFloor(floor);  
  delay(10);
}

// Function to set motor speed with deadband adjustment
void setMotorSpeed(int direction, int speed) {
  // Ensure speed is within 0–255
  speed = constrain(speed, 0, 255);

  int adjustedSpeed;

  if (direction > 0) {
    // Upward movement, mapping speed to 0–124 range for full control
    adjustedSpeed = map(speed, 0, 255, 125, 0);
    Serial.print("Moving up with PWM: ");
  } else {
    // Downward movement, mapping speed to 130–255 range for full control
    adjustedSpeed = map(speed, 0, 255, 129, 255);
    Serial.print("Moving down with PWM: ");
  }

  analogWrite(pwmPin, adjustedSpeed);
  Serial.println(adjustedSpeed);  // Print the adjusted speed for debugging
}


void stopMotor() {
  // Set PWM to 127 to hold motor in a neutral state
  analogWrite(pwmPin, 127);

  // Disable the motor entirely to eliminate noise
  digitalWrite(enablePin, LOW);
}

void updateEncoder() {
  int MSB = digitalRead(encA);  // Most Significant Bit
  int LSB = digitalRead(encB);  // Least Significant Bit

  int encoded = (MSB << 1) | LSB;    // Combine the bits into a single value
  int sum = (lastEncoded << 2) | encoded;  // Shift the previous state and add the new one

  // Determine direction based on state changes
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encCount--;

  lastEncoded = encoded;  // Store this state for the next time
}
