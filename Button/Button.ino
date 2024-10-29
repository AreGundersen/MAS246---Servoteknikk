
const uint8_t analogIn = A0;
const uint8_t enableDCPin = 7;
const uint8_t motorDCSignal = 6;
const uint8_t buttonFloors[8] = {22, 23, 24, 25, 26, 27, 28, 29};
const uint8_t numberFloors[8] = {0, 1, 2, 3, 4, 5, 6, 7,};
const uint8_t lightFloors[8] = {49, 48, 47, 46, 45, 44, 43, 42};
uint8_t i; 
const int timer = 1000;


void setup() {
  Serial.begin(300);
  pinMode(analogIn, INPUT);
  pinMode(enableDCPin, OUTPUT);
  digitalWrite(enableDCPin, HIGH);
  pinMode(motorDCSignal, OUTPUT);

  for (i = 0; i < 8; i++) {
    pinMode(buttonFloors[i], INPUT);
    pinMode(lightFloors[i], OUTPUT);
    digitalWrite(lightFloors[i], LOW);
  }
}

void loop() {
  
  for (int i = 0; i < 8; i++) {
    if (digitalRead(buttonFloors[i]) == HIGH) {

      digitalWrite(lightFloors[i], HIGH);  // Turn on the corresponding light
      delay(timer);

    } else {

      digitalWrite(lightFloors[i], LOW);   // Turn off the light

    }

  int value = analogRead(analogIn);
  int PWM = map(value, 0, 1023, 0, 255);
  float writeValue = value * 5.0 / 1023.0;

  analogWrite(motorDCSignal, PWM);
  
  }
}

