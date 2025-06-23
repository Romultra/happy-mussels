#include <Wire.h>
#include <utils.h> // For setPumpSpeed()
#include <Adafruit_TCS34725.h>
#include <Preferences.h>
#include <SPIFFS.h>

// Pin assignments
const int ledPinR = 33; // Red LED PWM pin
const int ledPinG = 15; // Green LED PWM pin
const int ledPinB = 32; // Blue LED PWM pin

// Settings
int nMeasurements = 5; // Number of repeated measurements for averaging
int defaultPumpSpeed = 128; // Adjust as needed
int circulationTimeMs = 20000; // Circulation/rinse time in milliseconds (default: 20 sec)

// Preferences for persistent settings
Preferences prefs;

// Global variables for light parameter locking
int bestLedR = 255, bestLedG = 255, bestLedB = 255;
uint8_t bestIntegrationTime = TCS34725_INTEGRATIONTIME_101MS;
tcs34725Gain_t bestGain = TCS34725_GAIN_1X;

// TCS34725 sensor setup (default: 101ms, 1x gain)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(bestIntegrationTime, bestGain);

bool csvHeaderWritten = false;

// --- Function Prototypes ---
void mainMenu();
void changeLogicParameters();
void autoCalibrateLight();
void calibrationSequence();
double measureChannelManual(String color, int ledR = 0, int ledG = 0, int ledB = 0);
float measureChannel(String color);
void waitForUser();
char waitForChoice(const char* options);
void saveLightCalibration();
void loadLightCalibration();
void saveCalibrationData(int concentrationIdx, float resultsFlow[4], float resultsStationary[4], float avgsFlow[4], float avgsStationary[4], int nMeas);
void printCalibrationCSV();
void csvMenu();

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

  pinMode(ledPinR, OUTPUT);
  pinMode(ledPinG, OUTPUT);
  pinMode(ledPinB, OUTPUT);

  initPumpPWM();

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
  Serial.println("4. CSV data options");
  Serial.println("5. Exit");
  Serial.println("Enter your choice:");

  while (!Serial.available());
  int choice = Serial.parseInt();
  Serial.read(); // consume extra chars

  switch (choice) {
    case 1:
      autoCalibrateLight();
      break;
    case 2:
      calibrationSequence();
      break;
    case 3:
      changeLogicParameters();
      break;
    case 4:
      csvMenu();
      break;
    default:
      Serial.println("Goodbye!");
      while (1);
  }
}

void csvMenu() {
  while (true) {
    Serial.println("\n[CSV Data Menu]");
    Serial.println("1. Output calibration CSV");
    Serial.println("2. Erase calibration CSV");
    Serial.println("3. Return to main menu");
    Serial.println("Enter your choice:");
    while (!Serial.available());
    int csvChoice = Serial.parseInt();
    Serial.read();
    
    // Declare all variables before the switch statement
    String confirm;
    
    switch (csvChoice) {
      case 1:
        printCalibrationCSV();
        break;
      case 2:
        Serial.println("Type YES to confirm erasing all calibration data:");
        while (!Serial.available());
        confirm = Serial.readStringUntil('\n');
        confirm.trim();
        if (confirm == "YES") {
          SPIFFS.remove("/calib_data.csv");
          Serial.println("Calibration CSV erased.");
          csvHeaderWritten = false;
        } else {
          Serial.println("Erase cancelled.");
        }
        break;
      case 3:
        mainMenu();
        return;
      default:
        Serial.println("Invalid option.");
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
    Serial.println("Enter the number to change, or 4 to return:");
    while (!Serial.available());
    int paramChoice = Serial.parseInt();
    Serial.read();
    if (paramChoice == 4) break;
    Serial.println("Enter new value:");
    while (!Serial.available());
    int newValue = Serial.parseInt();
    Serial.read();
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
  mainMenu();
}

void autoCalibrateLight() {
  Serial.println("Insert BLANK solvent (no algae) into sensor.");
  Serial.println("Press any key when ready.");
  waitForUser();

  setPumpSpeed(defaultPumpSpeed);
  Serial.print("Rinsing with blank solvent for ");
  Serial.print(circulationTimeMs / 1000);
  Serial.println(" seconds...");
  delay(circulationTimeMs);
  setPumpSpeed(0);

  // Set LEDs to maximum
  bestLedR = 255; bestLedG = 255; bestLedB = 255;

  // TCS34725 settings to test
  uint8_t integrationTimes[] = {
    TCS34725_INTEGRATIONTIME_2_4MS,
    TCS34725_INTEGRATIONTIME_24MS,
    TCS34725_INTEGRATIONTIME_50MS,
    TCS34725_INTEGRATIONTIME_60MS,
    TCS34725_INTEGRATIONTIME_101MS,
    TCS34725_INTEGRATIONTIME_120MS,
    TCS34725_INTEGRATIONTIME_154MS,
    TCS34725_INTEGRATIONTIME_180MS,
    TCS34725_INTEGRATIONTIME_199MS,
    TCS34725_INTEGRATIONTIME_240MS,
    TCS34725_INTEGRATIONTIME_300MS,
    TCS34725_INTEGRATIONTIME_360MS,
    TCS34725_INTEGRATIONTIME_401MS,
    TCS34725_INTEGRATIONTIME_420MS,
    TCS34725_INTEGRATIONTIME_480MS,
    TCS34725_INTEGRATIONTIME_499MS,
    TCS34725_INTEGRATIONTIME_540MS,
    TCS34725_INTEGRATIONTIME_600MS,
    TCS34725_INTEGRATIONTIME_614MS
  };
  tcs34725Gain_t gains[] = {
    TCS34725_GAIN_1X,
    TCS34725_GAIN_4X,
    TCS34725_GAIN_16X,
    TCS34725_GAIN_60X
  };

  const uint16_t minTarget = 40000;
  const uint16_t maxTarget = 50000;

  // Find the optimal settings using the same protocol: measure clear channel with only one LED on at a time
  bool found = false;
  for (uint8_t i = 0; i < sizeof(integrationTimes)/sizeof(integrationTimes[0]); i++) {
    tcs.setIntegrationTime(integrationTimes[i]);
    delay(100);
    for (uint8_t j = 0; j < sizeof(gains)/sizeof(gains[0]); j++) {
      tcs.setGain(gains[j]);
      delay(100);
      // For each color, turn on only that LED and measure clear channel
      float clearR = 0, clearG = 0, clearB = 0;
      // Red LED
      analogWrite(ledPinR, bestLedR); analogWrite(ledPinG, 0); analogWrite(ledPinB, 0);
      delay(100);
      clearR = measureChannel("clear");
      analogWrite(ledPinR, 0);
      // Green LED
      analogWrite(ledPinG, bestLedG); analogWrite(ledPinR, 0); analogWrite(ledPinB, 0);
      delay(100);
      clearG = measureChannel("clear");
      analogWrite(ledPinG, 0);
      // Blue LED
      analogWrite(ledPinB, bestLedB); analogWrite(ledPinR, 0); analogWrite(ledPinG, 0);
      delay(100);
      clearB = measureChannel("clear");
      analogWrite(ledPinB, 0);

      Serial.print("Test IT="); Serial.print(integrationTimes[i]);
      Serial.print(" Gain="); Serial.print(gains[j]);
      Serial.print(" Clear(Red LED): "); Serial.print(clearR);
      Serial.print(" Clear(Green LED): "); Serial.print(clearG);
      Serial.print(" Clear(Blue LED): "); Serial.println(clearB);

      if (clearR > minTarget && clearR < maxTarget &&
          clearG > minTarget && clearG < maxTarget &&
          clearB > minTarget && clearB < maxTarget) {
        bestIntegrationTime = integrationTimes[i];
        bestGain = gains[j];
        found = true;
        break;
      }
    }
    if (found) break;
  }

  // Set found parameters or default if not found
  tcs.setIntegrationTime(bestIntegrationTime);
  tcs.setGain(bestGain);

  // Save the best parameters
  saveLightCalibration();

  Serial.println("Light parameters auto-calibrated and locked in:");
  Serial.print("LEDs: R="); Serial.print(bestLedR);
  Serial.print(" G="); Serial.print(bestLedG);
  Serial.print(" B="); Serial.println(bestLedB);
  Serial.print("Integration Time: "); Serial.println((int)bestIntegrationTime);
  Serial.print("Gain: "); Serial.println((int)bestGain);

  if (!found) {
    Serial.println("Warning: Could not find optimal parameters! Using best available.");
  }

  mainMenu();
}


void calibrationSequence() {
  Serial.println("\n[Calibration Sequence]");
  Serial.println("Prepare calibration concentrations (lowest to highest).");
  Serial.println("For each, follow the instructions.");

  int concentrationIndex = 1;
  while (true) {
    Serial.print("\n=== Calibration for Concentration #"); Serial.print(concentrationIndex); Serial.println(" ===");
    Serial.println("1. REMOVE last concentration (manual, save it)");
    Serial.println("2. Insert blank solvent circuit, then press any key to run rinse cycle.");
    waitForUser();
    setPumpSpeed(defaultPumpSpeed);
    Serial.print("Rinsing with blank solvent for ");
    Serial.print(circulationTimeMs / 1000);
    Serial.println(" seconds...");
    delay(circulationTimeMs);
    setPumpSpeed(0);

    Serial.println("3. EMPTY the tube fully (manual). Press any key when done.");
    waitForUser();

    Serial.println("4. CIRCULATE new calibration solution (pump ON, 20s to remove air pockets). Press any key to start.");
    waitForUser();
    setPumpSpeed(defaultPumpSpeed);
    delay(circulationTimeMs);

    Serial.println("5. Measuring with FLOW (pump ON)...");
    float resultsFlow[3][nMeasurements]; // 0:Red, 1:Green, 2:Blue (clear channel for each color)
    setPumpSpeed(defaultPumpSpeed);
    for (int rep = 0; rep < nMeasurements; rep++) {
      // Red LED on, measure clear
      analogWrite(ledPinR, bestLedR); analogWrite(ledPinG, 0); analogWrite(ledPinB, 0);
      delay(100);
      resultsFlow[0][rep] = measureChannel("clear");
      analogWrite(ledPinR, 0);
      // Green LED on, measure clear
      analogWrite(ledPinG, bestLedG); analogWrite(ledPinR, 0); analogWrite(ledPinB, 0);
      delay(100);
      resultsFlow[1][rep] = measureChannel("clear");
      analogWrite(ledPinG, 0);
      // Blue LED on, measure clear
      analogWrite(ledPinB, bestLedB); analogWrite(ledPinR, 0); analogWrite(ledPinG, 0);
      delay(100);
      resultsFlow[2][rep] = measureChannel("clear");
      analogWrite(ledPinB, 0);
      delay(200);
    }
    delay(800); // Wait a bit before next measurement
    setPumpSpeed(0);
    Serial.println("Measuring with STATIONARY liquid (pump OFF)...");
    float resultsStationary[3][nMeasurements];
    for (int rep = 0; rep < nMeasurements; rep++) {
      // Red LED on, measure clear
      analogWrite(ledPinR, bestLedR); analogWrite(ledPinG, 0); analogWrite(ledPinB, 0);
      delay(100);
      resultsStationary[0][rep] = measureChannel("clear");
      analogWrite(ledPinR, 0);
      // Green LED on, measure clear
      analogWrite(ledPinG, bestLedG); analogWrite(ledPinR, 0); analogWrite(ledPinB, 0);
      delay(100);
      resultsStationary[1][rep] = measureChannel("clear");
      analogWrite(ledPinG, 0);
      // Blue LED on, measure clear
      analogWrite(ledPinB, bestLedB); analogWrite(ledPinR, 0); analogWrite(ledPinG, 0);
      delay(100);
      resultsStationary[2][rep] = measureChannel("clear");
      analogWrite(ledPinB, 0);
      delay(200);
    }

    // Calculate averages for both cases
    float avgsFlow[3], avgsStationary[3];
    for (int c = 0; c < 3; c++) {
      float sumFlow = 0, sumStationary = 0;
      for (int rep = 0; rep < nMeasurements; rep++) {
        sumFlow += resultsFlow[c][rep];
        sumStationary += resultsStationary[c][rep];
      }
      avgsFlow[c] = sumFlow / nMeasurements;
      avgsStationary[c] = sumStationary / nMeasurements;
    }

    // Save calibration data before printing (update saveCalibrationData to handle 3 channels)
    saveCalibrationData(concentrationIndex, &resultsFlow[0][0], &resultsStationary[0][0], avgsFlow, avgsStationary, nMeasurements);

    Serial.println("Averages (Intensity, clear channel) WITH FLOW:");
    Serial.print("Red: "); Serial.println(avgsFlow[0]);
    Serial.print(" Green: "); Serial.println(avgsFlow[1]);
    Serial.print(" Blue: "); Serial.println(avgsFlow[2]);

    Serial.println("Averages (Intensity, clear channel) STATIONARY:");
    Serial.print("Red: "); Serial.println(avgsStationary[0]);
    Serial.print(" Green: "); Serial.println(avgsStationary[1]);
    Serial.print(" Blue: "); Serial.println(avgsStationary[2]);

    Serial.println("Record these values (with the concentration) in your calibration table.");
    Serial.println("Press 'n' for next concentration, or 'q' to quit calibration.");
    char userChoice = waitForChoice("nq");
    if (userChoice == 'q') break;

    concentrationIndex++;
  }
  setPumpSpeed(0);
  mainMenu();
}

// --------- Utility Functions -----------

double measureChannelManual(String color, int ledR, int ledG, int ledB) {
  analogWrite(ledPinR, ledR);
  analogWrite(ledPinG, ledG);
  analogWrite(ledPinB, ledB);
  delay(80); // Let light stabilize
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  analogWrite(ledPinR, 0);
  analogWrite(ledPinG, 0);
  analogWrite(ledPinB, 0);
  if (color == "red") return (double)r;
  if (color == "green") return (double)g;
  if (color == "blue") return (double)b;
  if (color == "clear") return (double)c;
  return 0.0;
}

float measureChannel(String color) {
  // Set all LEDs off
  analogWrite(ledPinR, 0);
  analogWrite(ledPinG, 0);
  analogWrite(ledPinB, 0);

  if (color == "red") {
    analogWrite(ledPinR, bestLedR);
  } else if (color == "green") {
    analogWrite(ledPinG, bestLedG);
  } else if (color == "blue") {
    analogWrite(ledPinB, bestLedB);
  }
  delay(100); // let the light stabilize

  // Read TCS34725 data
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  analogWrite(ledPinR, 0);
  analogWrite(ledPinG, 0);
  analogWrite(ledPinB, 0);

  if (color == "red") return (float)r;
  if (color == "green") return (float)g;
  if (color == "blue") return (float)b;
  if (color == "clear") return (float)c;

  return 0;
}

void waitForUser() {
  while (!Serial.available());
  Serial.read();
}

char waitForChoice(const char* options) {
  while (true) {
    while (!Serial.available());
    char c = Serial.read();
    for (int i = 0; options[i] != 0; i++) {
      if (c == options[i]) return c;
    }
  }
}

// --- Persistent storage functions ---
void saveLightCalibration() {
  prefs.putInt("ledR", bestLedR);
  prefs.putInt("ledG", bestLedG);
  prefs.putInt("ledB", bestLedB);
  prefs.putUChar("intTime", bestIntegrationTime);
  prefs.putUChar("gain", (uint8_t)bestGain);
}

void loadLightCalibration() {
  if (prefs.isKey("ledR")) bestLedR = prefs.getInt("ledR");
  if (prefs.isKey("ledG")) bestLedG = prefs.getInt("ledG");
  if (prefs.isKey("ledB")) bestLedB = prefs.getInt("ledB");
  if (prefs.isKey("intTime")) bestIntegrationTime = prefs.getUChar("intTime");
  if (prefs.isKey("gain")) bestGain = (tcs34725Gain_t)prefs.getUChar("gain");
}

void saveCalibrationData(int concentrationIdx, float resultsFlow[4], float resultsStationary[4], float avgsFlow[4], float avgsStationary[4], int nMeas) {
  File file = SPIFFS.open("/calib_data.csv", FILE_APPEND);
  if (!file) return;
  if (!csvHeaderWritten) {
    file.println("# New session: " + String(millis()) + " ms since boot");
    csvHeaderWritten = true;
  }
  // Save raw data
  for (int rep = 0; rep < nMeas; rep++) {
    file.printf("%d,flow,red,%.2f\n", concentrationIdx, resultsFlow[rep + 0 * nMeas]);
    file.printf("%d,flow,green,%.2f\n", concentrationIdx, resultsFlow[rep + 1 * nMeas]);
    file.printf("%d,flow,blue,%.2f\n", concentrationIdx, resultsFlow[rep + 2 * nMeas]);
    file.printf("%d,stationary,red,%.2f\n", concentrationIdx, resultsStationary[rep + 0 * nMeas]);
    file.printf("%d,stationary,green,%.2f\n", concentrationIdx, resultsStationary[rep + 1 * nMeas]);
    file.printf("%d,stationary,blue,%.2f\n", concentrationIdx, resultsStationary[rep + 2 * nMeas]);
  }
  // Save averages
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
