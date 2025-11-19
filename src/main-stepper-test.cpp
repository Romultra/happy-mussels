#include <Arduino.h>
#include "stepper-utils.h"

void setup() {
    Serial.begin(115200);
    Serial.println("Stepper test start");
    delay(2000);
    
    Serial.println("Moving to cooling...");
    moveSelector("cooling");
    delay(2000);
    Serial.println("Moving to algae...");
    moveSelector("algae");
    delay(2000);
    Serial.println("Test complete.");
}

void loop() {
    Serial.println("Moving to cooling...");
    moveSelector("cooling");
    delay(2000);
    Serial.println("Moving to algae...");
    moveSelector("algae");
    delay(2000);
    Serial.println("Test complete.");
}
