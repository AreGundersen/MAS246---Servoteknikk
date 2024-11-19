# Elevator Control System

## Overview
This project is an Arduino-based control system for an elevator. It uses a joystick, buttons, and a DC motor to move the elevator between floors, while an LCD display provides real-time feedback to the user. The system also manages the elevator door, making sure it opens and closes safely based on user actions and the current elevator state.

## Components Used
- **Power Supply and Power Button**: Supplies power to the entire module, with a button to easily turn the system on and off.
- **Confirm Button**: Positioned near the joystick to confirm user actions.
- **Arduino Mega 2560**: The core microcontroller that manages the elevator's logic.
- **LCD Display (1602)**: Displays the current floor, door status, and prompts to help guide the user.
- **Joystick**: Used to select floors and choose the direction (up or down).
- **Stepper Motor**: Operates the elevator door, allowing it to open and close smoothly.
- **DC Motor**: Powers the elevator's movement between floors.
- **DAC (MCP4922)**: Manages the current supplied to the stepper motor for door control.
- **Floor Buttons**: Located at each floor and inside the elevator to call the elevator or select the desired floor.
- **LED Indicators**: Indicate the current position of the elevator.

## Features
1. **Elevator Movement**:
   - The elevator moves between 8 floors (0 to 7).
   - If at the top floor, only the option to go down is available, and vice versa at the bottom floor.
   - Use the joystick to choose the direction (up or down) depending on the available options.
   - Moves to the user-selected floor and stops at requested floors.
2. **Door Control**:
   - The door operates in three states: OPEN, HALF, and CLOSED.
   - When the elevator reaches the selected floor, the door opens automatically and closes after 5 seconds.
3. **User Input**:
   - Joystick used for selecting the floor and direction.
   - Buttons are used to call the elevator from each floor or select a target floor while inside.
   - Confirm button for floor selection.
4. **Safety**:
   - The door remains closed during elevator movement.
   - The elevator stops at each requested floor, with clear prompts for user interaction.

## How to Use
1. **Initial State**: The elevator starts at floor 0 in idle mode, waiting for a user to call it.
2. **Selecting Your Floor**:
   - Move the joystick horizontally (X-axis) to select the desired floor.
   - Confirm your selection by pressing the confirm button.
3. **Choosing the Direction**:
   - Depending on the current floor, choose the available direction (Up or Down).
4. **Elevator Movement**:
   - Once a floor is selected, the elevator moves to that floor.
   - Once reached, the door opens and waits for 5 seconds before closing automatically.
5. **Selecting Your Destination**:
   - Once inside the elevator, use the buttons to select your destination floor.

## Setup and Installation
1. **Power Setup**:
   - Connect the power supply to the Arduino and other components.
   - Use the power button to turn the system on, ensuring both Vcc indicators are glowing.
2. **Wiring**:
   - Connect the joystick to pins `A1` (X-axis) and `A2` (Y-axis).
   - Connect the confirm button to digital pin `2`.
   - Use pins `22` to `29` for floor buttons and pins `49` to `42` for LED indicators.
   - Connect the DC motor and encoder for elevator movement.
   - Use pins `66`, `67`, `68`, and `69` for stepper motor door control.
3. **Libraries**:
   - `LiquidCrystal.h` for LCD control.
   - `dac.h` for controlling the stepper motor using DAC.

## Notes
- The door automatically closes after 5 seconds of being open.
- The stepper motor uses a sequence to control door states (OPEN, HALF, CLOSED).
- Ensure all connections are secure before powering on the system.

## Future Improvements
- Add smoother acceleration and deceleration for a more comfortable ride.
- Integrate a queuing system for movement in the same direction.
- Add additional safety features such as obstacle detection for door closure.
- Integrate a more sophisticated scheduling algorithm for handling multiple requests.

## Troubleshooting
- **LCD Not Displaying**: Ensure the connections are properly made between Arduino and the LCD.
- **Motor Not Moving**: Check motor driver connections and verify that the enable pin is set correctly.
- **Buttons Not Responding**: Verify if the pins are correctly configured as `INPUT_PULLUP`.

