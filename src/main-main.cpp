#include <config.h>
#include <secrets.h>
#include <utils.h>
#include <stepper-utils.h>
#include <network.h>
#include <PID_v1.h>
#include <time.h>
#include <mqtt.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>
#include <math.h>

#define I0_CONST 47046.4

const double A_MAX = 1.26;
const double K = 3.76e-6; // rate constant in 1/(units of concentration)

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

// PID variables
double tempSetpoint, tempInput, tempOutput;
double kp = 50, ki = 0, kd = 5; // Initial values, will be tuned
PID tempPID(&tempInput, &tempOutput, &tempSetpoint, kp, ki, kd, REVERSE);

//const unsigned long PID_TEST_DURATION_MS = 30UL * 60 * 1000;
const unsigned long PID_INTERVAL_MS = 5UL * 1000; // Log every 5 seconds
const unsigned long LOG_INTERVAL_MS = 5UL * 1000; // Log every 5 seconds

bool startOD = false;
bool startSystem = false; // Global flag to control system start via MQTT

// Feeding control variables
enum SystemState { STATE_PID, STATE_FEEDING };
SystemState systemState = STATE_PID;

const unsigned long FEED_INTERVAL_MS = 15UL * 60 * 1000; // 15 minutes
unsigned long lastFeedTime = 0;

unsigned long PIDstartTime = 0;
unsigned long lastLogTime = 0;
unsigned long lastPeltierOnTime = 0;
const unsigned long PELTIERTIMEOUT_MS = 5UL * 60 * 1000; // 5 minutes
int peltierForceOnCycles = 0; // Number of cycles to force Peltier ON after timeout
const int PELTIER_FORCE_CYCLES = 5; // Number of cycles to keep Peltier ON

// how many algae cells mussels need per 15min
const double ALGAE_REQUIREMENT_CELLS = 9.2605e6; // 9.2605 million cells per 15 minutes

double latestConcentration = -1.0; // Global variable to store latest concentration

double computeConcentration(double I);
void handlePIDControl(long now);
void handleFeeding();

void setup() {
  Serial.begin(115200);
  connectToWiFi();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  initMQTT();
  initPumpPWM();
  setPumpSpeed(0); // Start with pump off

  tempSetpoint = 18.0;
  tempPID.SetMode(AUTOMATIC);      // Enable PID automatic mode-

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
  } else {
    if (!SPIFFS.exists("/status_log.csv")) {
      File file = SPIFFS.open("/status_log.csv", FILE_WRITE);
      if (file) {
        file.println("timestamp,temperature,od_value,pump_speed,target_temp,pid_p,pid_i,pid_d,loop_id");
        file.close();
      }
    }
  }
  tempInput = readThermistorTempOnPin(THERM_PIN);
  Serial.println("\nReady.");
}

void loop() {
  if (!mqtt.connected()) reconnect();
  mqtt.loop();

  if (!startSystem) {
    // Wait for startSystem to be set true via MQTT
    delay(100);
    return;
  }
  
  tempInput = readThermistorTempOnPin(THERM_PIN);
  long now = millis();

  // Feeding interval check
  if (now - lastFeedTime > FEED_INTERVAL_MS && systemState == STATE_PID) {
    systemState = STATE_FEEDING;
    handleFeeding();
    lastFeedTime = now;
    systemState = STATE_PID;
    sendStatus("feeding_done"); // Send status after feeding
    return;
  }

  // PID control logic
  if (systemState == STATE_PID) {
    // Replace with actual condition to stop PID test if needed
    if (0) {
      //sendStatus("pid_finished");
      Serial.println("PID loop stopped.");
    }
    else if (now - lastLogTime > PID_INTERVAL_MS) {
      handlePIDControl(now);
    }
  } else if (systemState == STATE_FEEDING) {
    // Status is sent in handleFeeding and after feeding above
  } else {
    // Not in PID or FEEDING, send status
    sendStatus("idle");
  }

  if (startOD) {
    // Handle feeding logic here
    
  }
}

void handlePIDControl(long now) {
  tempPID.Compute();
  Serial.printf("PID Output: %.2f, Input: %.2f, Setpoint: %.2f\n", tempOutput, tempInput, tempSetpoint);
  int pumpValue = (int)tempOutput;
  if (pumpValue < 0) pumpValue = 0;
  if (pumpValue > 255) pumpValue = 255;
  if (peltierForceOnCycles > 0) {
    setPeltier(true);
    lastPeltierOnTime = now;
    peltierForceOnCycles--;
    Serial.println("Peltier forced ON (cycle hold)");
  } else if (pumpValue < 15) {
    // Below threshold, treat as off for relay
    if (now - lastPeltierOnTime > PELTIERTIMEOUT_MS) {
      // Force Peltier ON if it hasn't been ON for a while
      setPeltier(true);
      lastPeltierOnTime = now;
      peltierForceOnCycles = PELTIER_FORCE_CYCLES - 1; // Already ON this cycle
      Serial.println("Peltier forced ON after timeout");
    } else {
      setPeltier(false);
    }
  } else {
    // Optionally, turn relay on when pump is running
    setPeltier(true);
    lastPeltierOnTime = now;
    peltierForceOnCycles = 0;
  }
  setPumpSpeed(pumpValue);
  sendStatus("pid_running");
  lastLogTime = now;
}

void handleFeeding() {
  Serial.println("Switching to FEEDING mode");
  moveSelector("FEED");

  // 1. Load volume to OD sensor (placeholder: pump 5 mL)
  pumpVolume(-16); // Example: load 5 mL to OD sensor

  // 2. Let sample be stationary
  delay(1000); // 1 second to settle

  // 3. Measure intensity and compute concentration
  float intensity = measureIntensity();
  double concentration = computeConcentration(intensity);
  latestConcentration = concentration; // Update global latest concentration
  Serial.printf("Measured intensity: %.2f, concentration: %.2f\n", intensity, concentration);

  // 4. Compute required volume to feed
  // V = required cells / concentration (cells/mL) => mL
  double requiredVolumeML = (concentration > 0) ? (ALGAE_REQUIREMENT_CELLS / concentration) : 0;
  Serial.printf("Required algae volume to feed: %.2f mL (target: %.2e cells, measured: %.2f cells/mL)\n", requiredVolumeML, ALGAE_REQUIREMENT_CELLS, concentration);

  // 5. pump the ramaining tubing volume
  Serial.println("Pumping remaining tubing volume (16 mL)");
  pumpVolume(-10); // PLACEHOLDER

  // 6. Pump required amount using pumpVolume
  if (requiredVolumeML > 0) {
    pumpVolume(-requiredVolumeML);
  }

  // 7. Pump back the algae contained in the tubing
  Serial.println("Pumping back the algae from the tubing");
  pumpVolume(16+10); // PLACEHOLDER: pump back the tubing volume + placeholder

  // 8. Switch back to cooling
  moveSelector("COOL");
  Serial.println("Feeding complete, switching back to PID mode");
}

double computeConcentration(double I) {
  // Safety check: I must be positive and less than or equal to I0_CONST
  if (I <= 0 || I0_CONST <= 0 || I > I0_CONST) {
    return -1.0;
  }

  // 1) Compute optical density (base‐10 log)
  double OD = log10(I0_CONST / I);

  // 2) Compute the inner argument of the natural log:
  //    arg = 1 - (OD / 1.26)
  double arg = 1.0 - OD / 1.26;

  // Must be > 0 to take ln(arg)
  if (arg <= 0) {
    return -1.0;  // indicates out‐of‐range OD
  }

  // 3) Inverted formula:
  //    C = - ln(arg) / (3.76 × 10⁻⁶)
  double C = -log(arg) / 3.76e-6;

  return C;
}