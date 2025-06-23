#include <config.h>
#include <secrets.h>
#include <utils.h>
#include <network.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>
#include <mqtt.h>

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600; // Adjust for your timezone

double tempSetpoint, tempInput, tempOutput;
PID tempPID(&tempInput, &tempOutput, &tempSetpoint,20,5,1, REVERSE);

double tempValue = 25; // Simulate initial temperature value
long lastMsg = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  connectToWiFi();

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  initMQTT();

  initPumpPWM(); // Initialize pump PWM pins
  setPumpSpeed(255); // Set initial pump speed to 128 (50% duty cycle)
  
  tempInput = tempValue; // Set initial temperature input
  tempSetpoint = 18.0; // Set desired 
  
  tempPID.SetMode(AUTOMATIC);
  tempPID.SetOutputLimits(0, 100); // Set output limits for the PID controller
}

void loop() {
  if (!mqtt.connected()) {
  reconnect();
  }
  mqtt.loop();

  long now = millis();
  if (now - lastMsg > 1000) {
    tempPID.Compute();
    lastMsg = now;
    
    tempValue = tempValue - (tempOutput-5) * 0.01; // Simulate temperature change based on pump output
    tempInput = tempValue;

    Serial.print("Current Temperature: ");
    Serial.print(tempValue);
    Serial.print(" | Setpoint: ");
    Serial.print(tempSetpoint);
    Serial.print(" | Pump Speed: ");
    Serial.println(tempOutput);
    
    // Convert the value to a char array
    char tempString[8];
    dtostrf(tempValue, 1, 2, tempString);
    mqtt.publish("musselfarm/temperature", tempString);
  }
}