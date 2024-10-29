// Pin definitions
#define encoderPinA 2
#define encoderPinB 3
#define motorPWM 9  // PWM pin for motor speed control
#define motorDir1 8 // Motor direction pin 1
#define motorDir2 7 // Motor direction pin 2

volatile long encoderPosition = 0;
long lastEncoderPosition = 0;
int motorSpeed = 0;

void setup() {
  // Initialize encoder pins as inputs
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  
  // Initialize motor control pins as outputs
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir1, OUTPUT);
  pinMode(motorDir2, OUTPUT);

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
  
  Serial.begin(9600); // Initialize serial monitor for debugging
}

void loop() {
  // Read the current encoder position
  long currentEncoderPosition = encoderPosition;
  
  // Calculate the difference in encoder position
  long encoderChange = currentEncoderPosition - lastEncoderPosition;

  // Update last encoder position
  lastEncoderPosition = currentEncoderPosition;

  // Control the motor based on encoder change
  controlMotor(encoderChange);
  
  // Debugging output
  Serial.print("Encoder Position: ");
  Serial.println(encoderPosition);
  
  delay(100); // Adjust loop delay as needed
}

// Function to update encoder position
void updateEncoder() {
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);

  // Quadrature encoding logic
  if (stateA == stateB) {
    encoderPosition++;
  } else {
    encoderPosition--;
  }
}

// Function to control motor
void controlMotor(long encoderChange) {
  if (encoderChange > 0) {
    // Rotate motor forward if encoder turns forward
    digitalWrite(motorDir1, HIGH);
    digitalWrite(motorDir2, LOW);
    motorSpeed = map(encoderChange, 0, 100, 0, 255); // Adjust the speed scaling
  } else if (encoderChange < 0) {
    // Rotate motor backward if encoder turns backward
    digitalWrite(motorDir1, LOW);
    digitalWrite(motorDir2, HIGH);
    motorSpeed = map(abs(encoderChange), 0, 100, 0, 255); // Adjust the speed scaling
  } else {
    // Stop the motor if no encoder movement
    motorSpeed = 0;
  }

  // Set motor speed
  analogWrite(motorPWM, motorSpeed);
}
