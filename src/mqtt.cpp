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
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>

// MQTT configuration
const char* mqttHost = "mqtt.yperion.dev";
const uint16_t mqttPort = 8883;

// WiFi and MQTT clients
WiFiClientSecure secureClient;
PubSubClient mqtt(secureClient);

// External variables needed by these functions
extern double tempSetpoint;
extern PID tempPID;
extern double tempInput;
extern double tempOutput;
extern double latestConcentration; // Add this line to access the latest concentration
extern bool startSystem; // Add this line to use the global startSystem variable

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
    if (mqtt.connect("ESP32Client", MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      // Subscribe
      mqtt.subscribe("mussel/start");
      mqtt.subscribe("mussel/commands"); // Subscribe to JSON command topic
      mqtt.subscribe("mussel/startPID");
      mqtt.subscribe("mussel/startAutotune");
      mqtt.subscribe("mussel/startSystem"); // Subscribe to start system topic
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
  if (strcmp(topic, "mussel/startSystem") == 0) {
    if (!startSystem) {
      startSystem = true; // Set the global variable to start the system
      Serial.println("System started via MQTT!");
    }
    return; // No need to process further for this topic
  }

  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  Serial.println("Received [" + String(topic) + "]: " + msg);

  if (String(topic) == "mussel/commands") {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
      Serial.print("Failed to parse JSON command: ");
      Serial.println(error.c_str());
      return;
    }
    if (doc["target_temp"].is<double>()) {
      tempSetpoint = doc["target_temp"];
      Serial.printf("Set target temp to %.2f\n", tempSetpoint);
    }
    // Example: handle pump_speed, target_temp, lamp_state, pid_p/i/d
    if (doc["pid_p"].is<double>() || doc["pid_i"].is<double>() || doc["pid_d"].is<double>()) {
      double kp = doc["pid_p"] | tempPID.GetKp();
      double ki = doc["pid_i"] | tempPID.GetKi();
      double kd = doc["pid_d"] | tempPID.GetKd();
      tempPID.SetTunings(kp, ki, kd);
      Serial.printf("Set PID to P=%.2f I=%.2f D=%.2f\n", kp, ki, kd);
    }
    return;
  }
}

void sendStatus(const char* loop_id) {
  // Use the actual latest concentration reading
  float odValue = latestConcentration; // Use the real reading
  int pump_speed = (int)tempOutput;
  float target_temp = tempSetpoint;
  float pid_p = tempPID.GetKp();
  float pid_i = tempPID.GetKi();
  float pid_d = tempPID.GetKd();

  time_t nowTime;
  time(&nowTime);
  struct tm timeinfo;
  localtime_r(&nowTime, &timeinfo);
  char timeStr[32];
  strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);

  // Compose JSON status
  JsonDocument doc;
  doc["timestamp"] = timeStr;
  doc["temperature"] = tempInput;
  doc["od_value"] = odValue; // Now using actual latest concentration
  doc["pump_speed"] = pump_speed;
  doc["target_temp"] = target_temp;
  doc["pid_p"] = pid_p;
  doc["pid_i"] = pid_i;
  doc["pid_d"] = pid_d;
  doc["loop_id"] = loop_id;
  char jsonBuffer[256];
  size_t n = serializeJson(doc, jsonBuffer);
  mqtt.publish("mussel/status", jsonBuffer, n);

  // Compose CSV row and publish to mussel/pid
  char csvBuffer[256];
  snprintf(csvBuffer, sizeof(csvBuffer), "%s,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%s",
    timeStr,
    tempInput,
    0.0, // Placeholder for OD value, replace with actual reading
    pump_speed,
    target_temp,
    pid_p,
    pid_i,
    pid_d,
    loop_id
  );
  mqtt.publish("mussel/pid", csvBuffer);

  // Save to CSV
  File file = SPIFFS.open("/status_log.csv", FILE_APPEND);
  if (file) {
    file.printf("%s,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%s\n",
      timeStr,
      tempInput,
      0.0, // Placeholder for OD value, replace with actual reading
      pump_speed,
      target_temp,
      pid_p,
      pid_i,
      pid_d,
      loop_id
    );
    file.close();
  } else {
    Serial.println("Failed to open status_log.csv for appending");
  }
}

// Old onMqttMessage implementation (commented out)
/*
void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  Serial.println("Received [" + String(topic) + "]: " + msg);
  /*
  // Parse JSON / act on commands here…
  if (String(topic) == "mussel/commands") {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
      Serial.print("Failed to parse JSON command: ");
      Serial.println(error.c_str());
      return;
    }
    // Example: handle pump_speed, target_temp, lamp_state, pid_p/i/d
    if (doc.containsKey("pump_speed")) {
      int speed = doc["pump_speed"];
      setPumpSpeed(speed);
      Serial.printf("Set pump speed to %d\n", speed);
    }
    if (doc.containsKey("target_temp")) {
      tempSetpoint = doc["target_temp"];
      Serial.printf("Set target temp to %.2f\n", tempSetpoint);
    }
    if (doc.containsKey("lamp_state")) {
      bool lamp = doc["lamp_state"];
      Serial.printf("Set lamp state to %s\n", lamp ? "ON" : "OFF");
      // TODO: Add lamp control code
    }
    if (doc.containsKey("pid_p") || doc.containsKey("pid_i") || doc.containsKey("pid_d")) {
      double kp = doc["pid_p"] | tempPID.GetKp();
      double ki = doc["pid_i"] | tempPID.GetKi();
      double kd = doc["pid_d"] | tempPID.GetKd();
      tempPID.SetTunings(kp, ki, kd);
      Serial.printf("Set PID to P=%.2f I=%.2f D=%.2f\n", kp, ki, kd);
    }
    return;
  }*/
/*
}

// New onMqttMessage implementation for experiment start
extern void onMqttMessage(char* topic, byte* payload, unsigned int len);
*/

