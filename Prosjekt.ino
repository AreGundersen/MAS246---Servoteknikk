
// ===================
// Elevator Control Code
// ===================

// DC motor control pins
const int enablePin = 7;  // Pin for motor enable
const int pwmPin = 6;     // PWM pin for motor speed control Down = [0-127], Up = [128-255];

const int analogPin = A0; // Analog input for speed control
const int encA  = 20;
const int encB  = 21;

volatile int encCount = 0; // 1 round = 4211;


// Variables to Track State
bool moving = false;
bool doorOpen = false;
int currentFloor = 1;
int targetFloor = 1;

void setup() {
  Serial.begin(9600); // Begin serial logging

  // Set motor control pins as output
  pinMode(enablePin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);  

  // Turn on the motor (set enable pin HIGH)
  digitalWrite(enablePin, HIGH);

  attachInterrupt(digitalPinToInterrupt(encA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB), encoderISR, CHANGE);

}

void loop() {
  // Read the analog input value from A0 (0-1023)
  int sensorValue = analogRead(analogPin);

  
  // Map the sensor value to a PWM range (0-255)
  int pwmValue = map(sensorValue, 0, 1023, 0, 255);
  
  // Write the PWM value to control motor speed
  analogWrite(pwmPin, pwmValue);


  // Looking at values
  Serial.print("Analog Pin: ");
  Serial.print(analogPin);

  Serial.print(" | Sensor Value: ");
  Serial.print(sensorValue);

  Serial.print(" | Encoder Count: ");
  Serial.print(encCount);

  Serial.print(" | PWM Pin: ");
  Serial.println(pwmValue);


  
  // Small delay for stability (optional)
  delay(50);
}

void encoderISR(){

  int stateA = digitalRead(encA);
  int stateB = digitalRead(encB);

  if (stateA == HIGH){
    encCount++;
  }

}
