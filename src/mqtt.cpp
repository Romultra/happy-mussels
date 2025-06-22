#include <Arduino.h>
#include <config.h>
#include <secrets.h>
#include <utils.h>
#include <network.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "mqtt.h"

// MQTT configuration
const char* mqttHost = "sonarr.yperion.dev";
const uint16_t mqttPort = 8883;

// WiFi and MQTT clients
WiFiClientSecure secureClient;
PubSubClient mqtt(secureClient);

// External variables needed by these functions
extern double tempSetpoint;
extern PID tempPID;
extern double tempInput;

void initMQTT() {
  secureClient.setCACert(caCert);
  mqtt.setServer(mqttHost, mqttPort);
  mqtt.setCallback(onMqttMessage);
}

void reconnect() {  
  // Loop until we're reconnected
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe
      mqtt.subscribe("musselfarm/settemp");

      mqtt.subscribe("musselfarm/pid/p");
      mqtt.subscribe("musselfarm/pid/i");
      mqtt.subscribe("musselfarm/pid/d");

      mqtt.subscribe("musselfarm/lamp");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  Serial.println("Received [" + String(topic) + "]: " + msg);

  // Parse JSON / act on commands here…

  if (String(topic) == "esp32/settemp") {
    // Convert payload to double and set as new setpoint
    tempSetpoint = msg.toDouble();
    Serial.print("New Setpoint: ");
    Serial.println(tempSetpoint);
  }

  if (String(topic) == "musselfarm/pid/p") {
    double p = msg.toDouble();
    tempPID.SetTunings(p, tempPID.GetKi(), tempPID.GetKd());
    Serial.print("PID P set to: ");
    Serial.println(p);
  }

  if (String(topic) == "musselfarm/pid/i") {
    double i = msg.toDouble();
    tempPID.SetTunings(tempPID.GetKp(), i, tempPID.GetKd());
    Serial.print("PID I set to: ");
    Serial.println(i);
  }

  if (String(topic) == "musselfarm/pid/d") {
    double d = msg.toDouble();
    tempPID.SetTunings(tempPID.GetKp(), tempPID.GetKi(), d);
    Serial.print("PID D set to: ");
    Serial.println(d);
  }

  if (String(topic) == "musselfarm/lamp") {
    if (msg == "ON") {
      Serial.println("Lamp ON");
      // Code to turn on the lamp
    } else if (msg == "OFF") {
      Serial.println("Lamp OFF");
      // Code to turn off the lamp
    } else {
      Serial.println("Unknown command for lamp: " + msg);
    }
  }
}

