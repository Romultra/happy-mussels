#include <config.h>
#include <secrets.h>
#include <utils.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>

const char* mqttHost = "sonarr.yperion.dev";
const uint16_t mqttPort = 8883;

WiFiClientSecure secureClient;
PubSubClient mqtt(secureClient);

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600; // Adjust for your timezone

double tempSetpoint, tempInput, tempOutput;
PID tempPID(&tempInput, &tempOutput, &tempSetpoint,20,5,1, REVERSE);

double tempValue = 25; // Simulate initial temperature value
long lastMsg = 0;

void onMqttMessage(char* topic, byte* payload, unsigned int len);

void setup() {
  Serial.begin(115200);
  while(!Serial);

  connectToWiFi();

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  secureClient.setCACert(caCert);

  mqtt.setServer(mqttHost, mqttPort);
  mqtt.setCallback(onMqttMessage);
  
  tempInput = tempValue; // Set initial temperature input
  tempSetpoint = 18.0; // Set desired 
  
  tempPID.SetMode(AUTOMATIC);
  tempPID.SetOutputLimits(0, 100); // Set output limits for the PID controller
}

void reconnect() {
  // Add before attempting connection
  Serial.println("Network information:");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("DNS server: ");
  Serial.println(WiFi.dnsIP());

  // Try manual DNS resolution
  IPAddress resolvedIP;
  int dnsResult = WiFi.hostByName(mqttHost, resolvedIP);
  Serial.print("DNS lookup result: ");
  Serial.print(dnsResult);
  Serial.print(" - IP: ");
  if(dnsResult > 0) {
    Serial.println(resolvedIP);
  } else {
    Serial.println("Failed to resolve");
  }
  
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