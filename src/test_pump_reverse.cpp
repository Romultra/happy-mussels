#include <Arduino.h>
#include <utils.h>

void setup() {
  Serial.begin(115200);
  delay(3000); // Wait for Serial
  Serial.println("Pump Reverse Test Start");
  initPumpPWM();
}

void loop() {
  Serial.println("Running pump backwards at low speed (64/255)...");
  setPumpSpeedBackward(64);
  delay(3000);

  Serial.println("Running pump backwards at medium speed (128/255)...");
  setPumpSpeedBackward(128);
  delay(3000);

  Serial.println("Running pump backwards at high speed (255/255)...");
  setPumpSpeedBackward(255);
  delay(3000);

  Serial.println("Turning pump off...");
  setPumpSpeedBackward(0);
  delay(3000);
}
