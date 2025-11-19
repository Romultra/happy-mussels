#include "config.h"
#include "stepper-utils.h"

// Stepper state
static int currentPosition = 0; // 0 = unknown, 1 = cooling, 2 = algae

// Define steps for each position (example: 0 for cooling, 200 for algae)
const int STEPS_TO_ALGAE = 200 * 19;   // Set this to the number of steps from cooling to algae
const int STEPS_TO_COOLING = 0;  // Home position

void moveSelector(const char* position) {
    int targetPosition = 0;
    if (strcmp(position, "cooling") == 0) {
        targetPosition = 1;
    } else if (strcmp(position, "algae") == 0) {
        targetPosition = 2;
    } else {
        return; // Invalid input
    }
    if (currentPosition == targetPosition) {
        return; // Already at requested position
    }
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_SLEEP_PIN, OUTPUT);
    digitalWrite(STEPPER_SLEEP_PIN, HIGH); // Wake up driver

    int steps = 0;
    bool dir = true;
    if (targetPosition == 2 && currentPosition != 2) {
        steps = STEPS_TO_ALGAE;
        dir = LOW; // Set direction to algae
    } else if (targetPosition == 1 && currentPosition != 1) {
        steps = STEPS_TO_ALGAE;
        dir = HIGH; // Set direction to cooling
    }
    digitalWrite(STEPPER_DIR_PIN, dir);
    for (int i = 0; i < steps; i++) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(800); // Adjust speed as needed
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(800);
    }
    digitalWrite(STEPPER_SLEEP_PIN, LOW); // Put driver to sleep
    currentPosition = targetPosition;
}