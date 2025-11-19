#include <config.h>
#include <secrets.h>
#include <utils.h>
#include <network.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
//#include <WiFi.h>
//#include <WiFiClientSecure.h>
//#include <PubSubClient.h>
#include <time.h>
#include <mqtt.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600; // Adjust for your timezone

double tempSetpoint, tempInput, tempOutput;
PID tempPID(&tempInput, &tempOutput, &tempSetpoint,20,5,1, REVERSE);

double tempValue;
long lastMsg = 0;

void setup() {
  Serial.begin(115200);
  //while(!Serial);
  connectToWiFi();

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  initMQTT();

  initPumpPWM(); // Initialize pump PWM pins
  setPumpSpeed(255); // Set initial pump speed to 128 (50% duty cycle)
  
  tempInput = readThermistorTemp(); // Set initial temperature input
  tempSetpoint = 18.0; // Set desired 
  
  tempPID.SetMode(AUTOMATIC);

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
  } else {
    // Write CSV header if file does not exist
    if (!SPIFFS.exists("/status_log.csv")) {
      File file = SPIFFS.open("/status_log.csv", FILE_WRITE);
      if (file) {
        file.println("timestamp,temperature,od_value,pump_speed,target_temp,pid_p,pid_i,pid_d,lamp_state");
        file.close();
      }
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
    setPumpSpeed((int)tempOutput); // Set the pump speed based on PID output

    tempValue = readThermistorTemp();
    tempInput = tempValue; // Read the current temperature from the thermistor

    // Example values for demonstration; replace with real readings as needed
    float od_value = 0.0; // TODO: Replace with actual OD reading
    int pump_speed = (int)tempOutput; // Already set above
    float target_temp = tempSetpoint;
    float pid_p = tempPID.GetKp();
    float pid_i = tempPID.GetKi();
    float pid_d = tempPID.GetKd();
    bool lamp_state = false; // TODO: Replace with actual lamp state

    // Compose JSON status
    StaticJsonDocument<256> doc;
    doc["temperature"] = tempInput;
    doc["od_value"] = od_value;
    doc["pump_speed"] = pump_speed;
    doc["target_temp"] = target_temp;
    doc["pid_p"] = pid_p;
    doc["pid_i"] = pid_i;
    doc["pid_d"] = pid_d;
    doc["lamp_state"] = lamp_state;
    char jsonBuffer[256];
    size_t n = serializeJson(doc, jsonBuffer);
    mqtt.publish("mussel/status", jsonBuffer, n);

    // Save to CSV
    File file = SPIFFS.open("/status_log.csv", FILE_APPEND);
    if (file) {
      time_t nowTime;
      time(&nowTime);
      struct tm timeinfo;
      localtime_r(&nowTime, &timeinfo);
      char timeStr[32];
      strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
      file.printf("%s,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%d\n",
        timeStr,
        tempInput,
        od_value,
        pump_speed,
        target_temp,
        pid_p,
        pid_i,
        pid_d,
        lamp_state ? 1 : 0
      );
      file.close();
    } else {
      Serial.println("Failed to open status_log.csv for appending");
    }
  }
}