#include "calib_menu.h"
#include <Adafruit_TCS34725.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <Arduino.h>

// Extern declarations for variables used from main-calibration.cpp
extern int nMeasurements;
extern int defaultPumpSpeed;
extern int circulationTimeMs;
extern Preferences prefs;
extern bool csvHeaderWritten;

// Helper: check for quit
bool checkForQuit() {
  if (Serial.available()) {
    char c = Serial.peek();
    if (c == 'q' || c == 'Q') {
      Serial.read(); // consume
      Serial.println("[Quit to main menu]");
      return true;
    }
  }
  return false;
}

int waitForUser() {
  while (!Serial.available()) {
    if (checkForQuit()) return 1;
  }
  char c = Serial.read();
  if (c == 'q' || c == 'Q') return 1;
  return 0;
}

char waitForChoice(const char* options) {
  while (true) {
    while (!Serial.available()) {
      if (checkForQuit()) return 'q';
    }
    char c = Serial.read();
    if (c == 'q' || c == 'Q') return 'q';
    for (int i = 0; options[i] != 0; i++) {
      if (c == options[i]) return c;
    }
  }
}

void changeLogicParameters() {
  while (true) {
    Serial.println("\n[Change Logic Parameters]");
    Serial.print("1. Circulation/rinse time (seconds): "); Serial.println(circulationTimeMs / 1000);
    Serial.print("2. Number of measurements: "); Serial.println(nMeasurements);
    Serial.print("3. Default pump speed: "); Serial.println(defaultPumpSpeed);
    Serial.println("4. Return to main menu");
    Serial.println("Enter the number to change, or 4 to return (or 'q' to quit):");
    while (!Serial.available()) {
      if (checkForQuit()) return;
    }
    int paramChoice = Serial.parseInt();
    Serial.read();
    if (paramChoice == 4) break;
    if (checkForQuit()) return;
    Serial.println("Enter new value (or 'q' to quit):");
    while (!Serial.available()) {
      if (checkForQuit()) return;
    }
    int newValue = Serial.parseInt();
    Serial.read();
    if (checkForQuit()) return;
    switch (paramChoice) {
      case 1:
        if (newValue > 0) circulationTimeMs = newValue * 1000;
        break;
      case 2:
        if (newValue > 0) nMeasurements = newValue;
        break;
      case 3:
        if (newValue >= 0 && newValue <= 255) defaultPumpSpeed = newValue;
        break;
      default:
        Serial.println("Invalid option.");
    }
  }
  saveLogicParameters();
  // mainMenu() will be called from the caller
}

void changeLightParameters() {
  extern int bestLedR, bestLedG, bestLedB;
  extern uint8_t bestIntegrationTime;
  extern tcs34725Gain_t bestGain;
  extern uint16_t integrationTimeToMs(uint8_t integrationTime);
  while (true) {
    Serial.println("\n[Change Light Parameters]");
    Serial.print("1. Red LED brightness (0-255): "); Serial.println(bestLedR);
    Serial.print("2. Green LED brightness (0-255): "); Serial.println(bestLedG);
    Serial.print("3. Blue LED brightness (0-255): "); Serial.println(bestLedB);
    Serial.print("4. Integration time (enum value, ms): "); Serial.print(bestIntegrationTime); Serial.print(" ("); Serial.print(integrationTimeToMs(bestIntegrationTime)); Serial.println(" ms)");
    Serial.print("5. Gain (enum value): "); Serial.println((int)bestGain);
    Serial.println("6. Save and return to main menu");
    Serial.println("Enter the number to change, or 6 to save and return (or 'q' to quit):");
    while (!Serial.available()) {
      if (checkForQuit()) return;
    }
    int paramChoice = Serial.parseInt();
    Serial.read();
    if (paramChoice == 6) break;
    if (checkForQuit()) return;
    Serial.println("Enter new value (or 'q' to quit):");
    while (!Serial.available()) {
      if (checkForQuit()) return;
    }
    int newValue = Serial.parseInt();
    Serial.read();
    if (checkForQuit()) return;
    switch (paramChoice) {
      case 1:
        if (newValue >= 0 && newValue <= 255) bestLedR = newValue;
        break;
      case 2:
        if (newValue >= 0 && newValue <= 255) bestLedG = newValue;
        break;
      case 3:
        if (newValue >= 0 && newValue <= 255) bestLedB = newValue;
        break;
      case 4:
        bestIntegrationTime = (uint8_t)newValue;
        break;
      case 5:
        bestGain = (tcs34725Gain_t)newValue;
        break;
      default:
        Serial.println("Invalid option.");
    }
  }
  saveLightCalibration();
  // mainMenu() will be called from the caller
}

void saveLightCalibration() {
  extern int bestLedR, bestLedG, bestLedB;
  extern uint8_t bestIntegrationTime;
  extern tcs34725Gain_t bestGain;
  prefs.putInt("ledR", bestLedR);
  prefs.putInt("ledG", bestLedG);
  prefs.putInt("ledB", bestLedB);
  prefs.putUChar("intTime", bestIntegrationTime);
  prefs.putUChar("gain", (uint8_t)bestGain);
}

void loadLightCalibration() {
  extern int bestLedR, bestLedG, bestLedB;
  extern uint8_t bestIntegrationTime;
  extern tcs34725Gain_t bestGain;
  if (prefs.isKey("ledR")) bestLedR = prefs.getInt("ledR");
  if (prefs.isKey("ledG")) bestLedG = prefs.getInt("ledG");
  if (prefs.isKey("ledB")) bestLedB = prefs.getInt("ledB");
  if (prefs.isKey("intTime")) bestIntegrationTime = prefs.getUChar("intTime");
  if (prefs.isKey("gain")) bestGain = (tcs34725Gain_t)prefs.getUChar("gain");
}

void saveLogicParameters() {
  extern int circulationTimeMs;
  extern int defaultPumpSpeed;
  extern int nMeasurements;
  prefs.putInt("circulationTimeMs", circulationTimeMs);
  prefs.putInt("defaultPumpSpeed", defaultPumpSpeed);
  prefs.putInt("nMeasurements", nMeasurements);
}

void loadLogicParameters() {
  if (prefs.isKey("circulationTimeMs")) circulationTimeMs = prefs.getInt("circulationTimeMs");
  if (prefs.isKey("defaultPumpSpeed")) defaultPumpSpeed = prefs.getInt("defaultPumpSpeed");
  if (prefs.isKey("nMeasurements")) nMeasurements = prefs.getInt("nMeasurements");
}

void clearPreferences() {
  Serial.println("Are you sure you want to clear all preferences? Type YES to confirm:");
  while (!Serial.available());
  String confirm = Serial.readStringUntil('\n');
  confirm.trim();
  if (confirm == "YES") {
    prefs.clear();
    Serial.println("All preferences cleared. Settings will be reset to defaults on next boot.");
  } else {
    Serial.println("Clear preferences cancelled.");
  }
  // mainMenu() will be called from the caller
}

void saveCalibrationData(int concentrationIdx, float resultsFlow[4], float resultsStationary[4], float avgsFlow[4], float avgsStationary[4], int nMeas, const String& sampleName) {
  File file = SPIFFS.open("/calib_data.csv", FILE_APPEND);
  if (!file) return;
  if (!csvHeaderWritten) {
    file.println("# New session: " + String(millis()) + " ms since boot");
    csvHeaderWritten = true;
  }
  // Save sample name as a comment for this block
  file.printf("# Sample: %d, Name: %s\n", concentrationIdx, sampleName.c_str());
  for (int rep = 0; rep < nMeas; rep++) {
    file.printf("%d,flow,red,%.2f\n", concentrationIdx, resultsFlow[rep + 0 * nMeas]);
    file.printf("%d,flow,green,%.2f\n", concentrationIdx, resultsFlow[rep + 1 * nMeas]);
    file.printf("%d,flow,blue,%.2f\n", concentrationIdx, resultsFlow[rep + 2 * nMeas]);
    file.printf("%d,stationary,red,%.2f\n", concentrationIdx, resultsStationary[rep + 0 * nMeas]);
    file.printf("%d,stationary,green,%.2f\n", concentrationIdx, resultsStationary[rep + 1 * nMeas]);
    file.printf("%d,stationary,blue,%.2f\n", concentrationIdx, resultsStationary[rep + 2 * nMeas]);
  }
  file.printf("%d,flow_avg,red,%.2f\n", concentrationIdx, avgsFlow[0]);
  file.printf("%d,flow_avg,green,%.2f\n", concentrationIdx, avgsFlow[1]);
  file.printf("%d,flow_avg,blue,%.2f\n", concentrationIdx, avgsFlow[2]);
  file.printf("%d,stationary_avg,red,%.2f\n", concentrationIdx, avgsStationary[0]);
  file.printf("%d,stationary_avg,green,%.2f\n", concentrationIdx, avgsStationary[1]);
  file.printf("%d,stationary_avg,blue,%.2f\n", concentrationIdx, avgsStationary[2]);
  file.close();
}

void printCalibrationCSV() {
  File file = SPIFFS.open("/calib_data.csv", FILE_READ);
  if (!file) {
    Serial.println("No calibration data file found.");
    return;
  }
  Serial.println("\n--- Calibration Data CSV ---");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
  Serial.println("\n--- End of CSV ---");
}

void csvMenu() {
  while (true) {
    Serial.println("\n[CSV Data Menu]");
    Serial.println("1. Output calibration CSV");
    Serial.println("2. Erase calibration CSV");
    Serial.println("3. Return to main menu");
    Serial.println("Enter your choice (or 'q' to quit):");
    while (!Serial.available()) {
      if (checkForQuit()) return;
    }
    int csvChoice = Serial.parseInt();
    Serial.read();
    if (checkForQuit()) return;
    String confirm;
    switch (csvChoice) {
      case 1:
        printCalibrationCSV();
        break;
      case 2:
        Serial.println("Type YES to confirm erasing all calibration data (or 'q' to quit):");
        while (!Serial.available()) {
          if (checkForQuit()) return;
        }
        confirm = Serial.readStringUntil('\n');
        confirm.trim();
        if (confirm == "q" || confirm == "Q") return;
        if (confirm == "YES") {
          SPIFFS.remove("/calib_data.csv");
          Serial.println("Calibration CSV erased.");
          csvHeaderWritten = false;
        } else {
          Serial.println("Erase cancelled.");
        }
        break;
      case 3:
        // mainMenu() will be called from the caller
        return;
      default:
        Serial.println("Invalid option.");
    }
  }
}
