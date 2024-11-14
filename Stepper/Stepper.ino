#include "dac.h"

// Define H-Bridge Control Pins
const int phaseA_control = 66;
const int enableA = 67;
const int phaseB_control = 68;
const int enableB = 69;

// Number of steps per half turn
const int halfTurnSteps = 75;

// Delay between steps for smooth movement (adjust as needed)
const int stepDelay = 7; // in milliseconds

void setup() {
    dac_init();
    set_dac(4095, 4095); // Set DAC for max reference voltage

    pinMode(phaseA_control, OUTPUT);
    pinMode(enableA, OUTPUT);
    pinMode(phaseB_control, OUTPUT);
    pinMode(enableB, OUTPUT);

    // Enable H-bridges
    digitalWrite(enableA, HIGH);
    digitalWrite(enableB, HIGH);
}

void stepMotor(bool direction) {
    // Step sequence for the stepper motor
    for (int i = 0; i < halfTurnSteps; i++) {
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
            // Step 4 (Reverse)
            digitalWrite(phaseA_control, HIGH);
            digitalWrite(phaseB_control, HIGH);
            delay(stepDelay);

            // Step 3 (Reverse)
            digitalWrite(phaseA_control, LOW);
            digitalWrite(phaseB_control, HIGH);
            delay(stepDelay);

            // Step 2 (Reverse)
            digitalWrite(phaseA_control, LOW);
            digitalWrite(phaseB_control, LOW);
            delay(stepDelay);

            // Step 1 (Reverse)
            digitalWrite(phaseA_control, HIGH);
            digitalWrite(phaseB_control, LOW);
            delay(stepDelay);
        }
    }
}

void loop() {
    // Move half-turn clockwise
    stepMotor(true);
    delay(1000); // Pause 1 second at the end

    // Move half-turn counter-clockwise
    stepMotor(false);
    delay(1000); // Pause 1 second before repeating
}
