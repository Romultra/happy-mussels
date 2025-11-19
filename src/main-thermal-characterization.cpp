#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <time.h>
#include <config.h>
#include <utils.h>
#include <mqtt.h>
#include <network.h>
#include <ArduinoJson.h>

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

// Thermistor pins
const int THERM_PIN_BUCKET1 = 32;
const int THERM_PIN_BUCKET2 = 34;
const int THERM_PIN_AMBIENT = 33;

// Experiment duration (3 hours)
const unsigned long EXPERIMENT_DURATION_MS = 3UL * 60 * 60 * 1000;
const unsigned long PID_INTERVAL_MS = 20UL * 1000; // Log every 20 seconds

unsigned long startTime = 0;
unsigned long lastLogTime = 0;
bool experimentStarted = false;

void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  if (strcmp(topic, "mussel/start") == 0) {
    if (!experimentStarted) {
      experimentStarted = true;
      Serial.println("Experiment started via MQTT!");
      startTime = millis();
      lastLogTime = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Thermal Characterization Setup");
  connectToWiFi();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  initMQTT();
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
  } else {
    if (!SPIFFS.exists("/thermal_log_final.csv")) {
      File file = SPIFFS.open("/thermal_log_final.csv", FILE_WRITE);
      if (file) {
        file.println("timestamp,temp_bucket1,temp_bucket2,temp_ambient");
        file.close();
      }
    }
  }
  Serial.println("\nReady. Send any message to 'mussel/start' to begin the experiment...");
}

void loop() {
  if (!mqtt.connected()) reconnect();
  mqtt.loop();
  if (!experimentStarted) {
    delay(100);
    return;
  }
  unsigned long now = millis();
  if (now - startTime > EXPERIMENT_DURATION_MS) {
    // Experiment finished
    Serial.println("Experiment complete.");
    while (1) delay(1000);
  }
  if (now - lastLogTime >= PID_INTERVAL_MS) {
    float temp1 = readThermistorTempOnPin(THERM_PIN_BUCKET1);
    float temp2 = readThermistorTempOnPin(THERM_PIN_BUCKET2);
    float tempAmb = readThermistorTempOnPin(THERM_PIN_AMBIENT);
    time_t nowTime;
    time(&nowTime);
    struct tm timeinfo;
    localtime_r(&nowTime, &timeinfo);
    char timeStr[32];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
    File file = SPIFFS.open("/thermal_log_final.csv", FILE_APPEND);
    if (file) {
      file.printf("%s,%.2f,%.2f,%.2f\n", timeStr, temp1, temp2, tempAmb);
      file.close();
    }
    Serial.printf("%s,%.2f,%.2f,%.2f\n", timeStr, temp1, temp2, tempAmb);
    // Send MQTT JSON
    JsonDocument doc;
    doc["timestamp"] = timeStr;
    doc["temp_bucket1"] = temp1;
    doc["temp_bucket2"] = temp2;
    doc["temp_ambient"] = tempAmb;
    char jsonBuffer[128];
    size_t n = serializeJson(doc, jsonBuffer);
    mqtt.publish("mussel/isolation", jsonBuffer, n);
    lastLogTime = now;
  }
}
