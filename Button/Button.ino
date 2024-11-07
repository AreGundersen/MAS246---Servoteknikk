
const uint8_t analogIn = A0;
const uint8_t enableDCPin = 7;
const uint8_t motorDCSignal = 6;
const int numFloors = 8;

const uint8_t buttonFloors[numFloors] = {22, 23, 24, 25, 26, 27, 28, 29};
const uint8_t numberFloors[numFloors] = {0, 1, 2, 3, 4, 5, 6, 7,};
const uint8_t lightFloors[numFloors] = {49, 48, 47, 46, 45, 44, 43, 42};
unsigned long lastDebounceTime[numFloors] = {0};  // Stores the last debounce time for each button
bool buttonStates[numFloors] = {false};  // Stores the last known state of each button
int debounceDelay = 100;



void setup() {

  for (int i = 0; i < numFloors; i++) {
    pinMode(buttonFloors[i],INPUT);  // Set each pin as input with pullup resistor
  }


  Serial.begin(9600);
  pinMode(analogIn, INPUT);
  pinMode(enableDCPin, OUTPUT);
  digitalWrite(enableDCPin, HIGH);
  pinMode(motorDCSignal, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(buttonFloors[i], INPUT);
    pinMode(lightFloors[i], OUTPUT);
    digitalWrite(lightFloors[i], LOW);
  }
}

void loop() {

    for (int i = 0; i < numFloors; i++) {
    int reading = digitalRead(buttonFloors[i]);  // Read the current button state

    // If the button state has changed from the last known state
    if (reading == LOW && !buttonStates[i]) {
      unsigned long currentTime = millis();

      // Check if enough time has passed since the last press
      if (currentTime - lastDebounceTime[i] > debounceDelay) {
        // Update debounce time and state
        lastDebounceTime[i] = currentTime;
        buttonStates[i] = true;
      Serial.print("Button pressed on floor: ");
      Serial.println(i);
      // Call a function to move the lift to floor i
      //moveToFloor(i);
      delay(100);  // Debounce delay
    }
  }

  for (int i = 0; i < 8; i++) {
    if (digitalRead(buttonFloors[i]) == HIGH) {

      digitalWrite(lightFloors[i], HIGH);  // Turn on the corresponding light
      delay(1000);

    } else {

      digitalWrite(lightFloors[i], LOW);   // Turn off the light

    }

  int value = analogRead(analogIn);
  int PWM = map(value, 0, 1023, 0, 255);
  float writeValue = value * 5.0 / 1023.0;

  analogWrite(motorDCSignal, PWM);
  
  }
}
}



