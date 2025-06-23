#include <Wire.h>
#include <utils.h> // For setPumpSpeed()
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include "calib_menu.h"
#include "calib_experiment.h"

// NeoPixel setup
#define NEOPIXEL_PIN 32
#define NEOPIXEL_NUM 1
Adafruit_NeoPixel strip(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Settings
int nMeasurements = 5; // Number of repeated measurements for averaging
int defaultPumpSpeed = 128; // Adjust as needed
int circulationTimeMs = 20000; // Circulation/rinse time in milliseconds (default: 20 sec)

// Preferences for persistent settings
Preferences prefs;

// Global variables for light parameter locking
int bestLedR = 255, bestLedG = 255, bestLedB = 255;
uint8_t lastIntegrationTime = TCS34725_INTEGRATIONTIME_101MS;
uint8_t bestIntegrationTime = TCS34725_INTEGRATIONTIME_240MS;
tcs34725Gain_t bestGain = TCS34725_GAIN_60X;

// TCS34725 sensor setup (default: 101ms, 1x gain)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(bestIntegrationTime, bestGain);

bool csvHeaderWritten = false;

void mainMenu();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // Wait for serial connection to be established
  while (!Serial) {
    delay(10); // Small delay to prevent watchdog issues
  }

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    while (1);
  }

  prefs.begin("odcalib", false); // namespace for calibration settings
  loadLightCalibration();
  loadLogicParameters();

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  initPumpPWM(); // Initialize pump PWM

  if (!tcs.begin()) {
    Serial.println("No TCS34725 sensor found ... check your connections");
    while (1);
  }

  tcs.setIntegrationTime(bestIntegrationTime);
  tcs.setGain(bestGain);

  Serial.println("=== OD Sensor Calibration ===");
  Serial.println("Connect everything and press any key to start.");
  while (!Serial.available());
  Serial.read(); // consume the key press

  mainMenu();
}

void loop() {
  // Do nothing, all logic handled in menu functions
}

void mainMenu() {
  Serial.println("\n[Main Menu]");
  Serial.println("1. Auto-calibrate light parameters");
  Serial.println("2. Start calibration sequence");
  Serial.println("3. Change logic parameters");
  Serial.println("4. Change light parameters");
  Serial.println("5. Balance LED brightness (to 45000)");
  Serial.println("6. CSV data options");
  Serial.println("7. Run pump for N seconds");
  Serial.println("8. Clear preferences");
  Serial.println("9. Exit");
  Serial.println("Enter your choice:");

  while (!Serial.available());
  int choice = Serial.parseInt();
  Serial.read(); // consume extra chars

  switch (choice) {
    case 1:
      autoCalibrateLight();
      mainMenu();
      break;
    case 2:
      calibrationSequence();
      mainMenu();
      break;
    case 3:
      changeLogicParameters();
      mainMenu();
      break;
    case 4:
      changeLightParameters();
      mainMenu();
      break;
    case 5:
      balanceLedBrightness();
      mainMenu();
      break;
    case 6:
      csvMenu();
      mainMenu();
      break;
    case 7: {
      Serial.println("Enter pump run time in seconds:");
      while (!Serial.available());
      int seconds = Serial.parseInt();
      Serial.read();
      runPumpForSeconds(seconds);
      mainMenu();
      break;
    }
    case 8:
      clearPreferences();
      mainMenu();
      break;
    default:
      Serial.println("Goodbye!");
      while (1);
  }
}