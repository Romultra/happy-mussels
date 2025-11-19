#include <config.h>
#include <secrets.h>
#include <utils.h>
#include <network.h>
#include <PID_v1.h>
#include <pidautotuner.h>
#include <time.h>
#include <mqtt.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

// Thermistor pins
// const int THERM_PIN = 32; // Use a single thermistor for PID control

// PID variables
double tempSetpoint, tempInput, tempOutput;
double kp = 6, ki = 0, kd = 1; // Initial values, will be tuned
PID tempPID(&tempInput, &tempOutput, &tempSetpoint, kp, ki, kd, REVERSE);

const unsigned long PID_TEST_DURATION_MS = 30UL * 60 * 1000;
const unsigned long PID_INTERVAL_MS = 5UL * 1000; // Log every 5 seconds
const unsigned long AUTO_TUNE_DURATION_MS = 30UL * 60 * 1000; // 30 minutes for autotune

PIDAutotuner tuner;
bool autotuneComplete = false;

bool startPID = false;
bool autoTuneStarted = false;

unsigned long PIDstartTime = 0;
unsigned long autoTuneStartTime = 0;
unsigned long lastLogTime = 0;

void sendStatus(const char* test_id);

void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  if (strcmp(topic, "mussel/startPID") == 0) {
    if (!startPID) {
      startPID = true;
      Serial.println("PID Test started via MQTT!");
      PIDstartTime = millis();
      lastLogTime = 0;
    }
  }

  if (strcmp(topic, "mussel/startAutotune") == 0) {
    if (!autoTuneStarted) {
      autoTuneStarted = true;
      Serial.println("Autotune started via MQTT!");
      autoTuneStartTime = millis();
      lastLogTime = 0;
      tuner.startTuningLoop();
    }
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

void setup() {
  Serial.begin(115200);
  connectToWiFi();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  initMQTT();
  initPumpPWM();
  setPumpSpeed(0); // Start with pump off

  tempSetpoint = 18.0;
  // Configure autotuner
  tuner.setTargetInputValue(tempSetpoint); // Target temperature
  tuner.setLoopInterval(PID_INTERVAL_MS); // ms between tuning steps
  tuner.setOutputRange(0, 255); // Pump speed range
  tuner.setZNMode(PIDAutotuner::znModeBasicPID);

  // --- PID setup fix ---
  //tempPID.SetOutputLimits(0, 255); // Ensure output is in pump range
  tempPID.SetMode(AUTOMATIC);      // Enable PID automatic mode
  // --- end fix ---

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
  } else {
    if (!SPIFFS.exists("/status_log.csv")) {
      File file = SPIFFS.open("/status_log.csv", FILE_WRITE);
      if (file) {
        file.println("timestamp,temperature,pump_speed,target_temp,pid_p,pid_i,pid_d,test_id");
        file.close();
      }
    }
  }
  tempInput = readThermistorTempOnPin(THERM_PIN);
  Serial.println("\nReady. Send any message to 'mussel/startPID' or 'mussel/startAutotune' to begin one of the experiment...");
}

void loop() {
  if (!mqtt.connected()) reconnect();
  mqtt.loop();

  tempInput = readThermistorTempOnPin(THERM_PIN);
  long now = millis();

  // PID test running
  if (startPID) {
    if (now - PIDstartTime > PID_TEST_DURATION_MS) {
      startPID = false;
      sendStatus("pid_finished");
      Serial.println("PID test completed.");
    } 
    else if (now - lastLogTime > PID_INTERVAL_MS) {
      tempPID.Compute();
      Serial.printf("PID Output: %.2f, Input: %.2f, Setpoint: %.2f\n", tempOutput, tempInput, tempSetpoint);
      int pumpValue = (int)tempOutput;
      if (pumpValue < 0) pumpValue = 0;
      if (pumpValue > 255) pumpValue = 255;
      setPumpSpeed(pumpValue);
      sendStatus("pid_test");
      lastLogTime = now;
    }
  }

  if (autoTuneStarted && !autotuneComplete) {
    if (now - autoTuneStartTime > AUTO_TUNE_DURATION_MS) {
      autoTuneStarted = false;
      autotuneComplete = true;
      sendStatus("autotune_timeout");
      Serial.println("Autotune timed out after 20 minutes.");
      setPumpSpeed(0); // Stop pump after autotune timeout
    } 
    else if (now - lastLogTime > PID_INTERVAL_MS) {
      // PID autotune logic using jackw01's library
      double output = tuner.tunePID(tempInput);
      int pumpValue = (int)output;
      if (pumpValue < 0) pumpValue = 0;
      if (pumpValue > 255) pumpValue = 255;
      setPumpSpeed(pumpValue);
      sendStatus("autotune_test");
      Serial.printf("Autotune Output: %.2f, Input: %.2f, Setpoint: %.2f\n", output, tempInput, tempSetpoint);
      lastLogTime = now;
      if (tuner.isFinished()) {
        double newKp = tuner.getKp();
        double newKi = tuner.getKi();
        double newKd = tuner.getKd();
        tempPID.SetTunings(newKp, newKi, newKd);
        Serial.printf("Autotune complete. New PID: P=%.2f I=%.2f D=%.2f\n", newKp, newKi, newKd);
        autotuneComplete = true;
        autoTuneStarted = false;
        sendStatus("autotune_finished");
      }
    }
  }
}

void sendStatus(const char* test_id) {
  // Example values for demonstration; replace with real readings as needed
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
  doc["pump_speed"] = pump_speed;
  doc["target_temp"] = target_temp;
  doc["pid_p"] = pid_p;
  doc["pid_i"] = pid_i;
  doc["pid_d"] = pid_d;
  doc["test_id"] = test_id;
  char jsonBuffer[256];
  size_t n = serializeJson(doc, jsonBuffer);
  mqtt.publish("mussel/status", jsonBuffer, n);

  // Compose CSV row and publish to mussel/pid
  char csvBuffer[256];
  snprintf(csvBuffer, sizeof(csvBuffer), "%s,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%s",
    timeStr,
    tempInput,
    pump_speed,
    target_temp,
    pid_p,
    pid_i,
    pid_d,
    test_id
  );
  mqtt.publish("mussel/pid", csvBuffer);

  // Save to CSV
  File file = SPIFFS.open("/status_log.csv", FILE_APPEND);
  if (file) {
    file.printf("%s,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%s\n",
      timeStr,
      tempInput,
      pump_speed,
      target_temp,
      pid_p,
      pid_i,
      pid_d,
      test_id
    );
    file.close();
  } else {
    Serial.println("Failed to open status_log.csv for appending");
  }
}
