#include "calib_experiment.h"
#include <Adafruit_TCS34725.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Extern declarations for variables and objects from main-calibration.cpp
extern int bestLedR, bestLedG, bestLedB;
extern uint8_t bestIntegrationTime;
extern tcs34725Gain_t bestGain;
extern Adafruit_TCS34725 tcs;
extern uint8_t lastIntegrationTime;
extern int nMeasurements;
extern int defaultPumpSpeed;
extern int circulationTimeMs;
extern void setPumpSpeed(int);
extern void saveLightCalibration();
extern bool csvHeaderWritten;
extern int waitForUser();
extern char waitForChoice(const char* options);
extern bool checkForQuit();
extern void saveCalibrationData(int concentrationIdx, float resultsFlow[4], float resultsStationary[4], float avgsFlow[4], float avgsStationary[4], int nMeas, const String& sampleName);

void autoCalibrateLight() {
  Serial.println("Insert BLANK solvent (no algae) into sensor.");
  Serial.println("Press any key when ready or 'q' to quit.");
  if (waitForUser() == 1) return;
  runPumpForSeconds(circulationTimeMs / 1000);

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
  const int gainValues[] = {
    1,  // 1X
    4,  // 4X
    16, // 16X
    60  // 60X
  };

  const uint16_t minTarget = 40000;
  const uint16_t maxTarget = 50000;

  // Find the optimal settings using the same protocol: measure clear channel with only one LED on at a time
  bool found = false;
  int bestIntegrationIndex = 0;
  int bestGainIndex = 0;
  // Loop through gains from lowest to highest (outer loop)
  for (uint8_t j = 2; j < sizeof(gains)/sizeof(gains[0]); j++) {
    tcs.setGain(gains[j]);
    delay(100);
    // Loop through integrationTimes from highest to lowest (inner loop)
    for (int i = (int)(sizeof(integrationTimes)/sizeof(integrationTimes[0])) - 2; i >= 0; i--) {
      tcs.setIntegrationTime(integrationTimes[i]);
      delay(100);
      // For each color, turn on only that LED and measure clear channel
      float clearR = 0, clearG = 0, clearB = 0;
      // Red LED
      setNeoPixelColor(bestLedR, 0, 0);
      delay(100);
      if (checkForQuit()) return;
      clearR = measureChannel("clear", integrationTimes[i]);
      setNeoPixelColor(0, 0, 0);
      // Green LED
      setNeoPixelColor(0, bestLedG, 0);
      delay(100);
      if (checkForQuit()) return;
      clearG = measureChannel("clear", integrationTimes[i]);
      setNeoPixelColor(0, 0, 0);
      // Blue LED
      setNeoPixelColor(0, 0, bestLedB);
      delay(100);
      if (checkForQuit()) return;
      clearB = measureChannel("clear", integrationTimes[i]);
      setNeoPixelColor(0, 0, 0);

      Serial.print("Test Gain="); Serial.print(gainValues[j]); Serial.print("X");
      Serial.print(" IT="); Serial.print(integrationTimeToMs(integrationTimes[i])); Serial.print("ms");
      Serial.print(" Clear(Red LED): "); Serial.print(clearR);
      Serial.print(" Clear(Green LED): "); Serial.print(clearG);
      Serial.print(" Clear(Blue LED): "); Serial.println(clearB);

      if (clearR > minTarget && clearR < maxTarget &&
          clearG > minTarget && clearG < maxTarget &&
          clearB > minTarget && clearB < maxTarget) {
        bestIntegrationTime = integrationTimes[i];
        bestGain = gains[j];
        bestIntegrationIndex = i;
        bestGainIndex = j;
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
  Serial.print("Integration Time: "); Serial.print(integrationTimeToMs(integrationTimes[bestIntegrationIndex])); Serial.println(" ms");
  Serial.print("Gain: "); Serial.print(gainValues[bestGainIndex]); Serial.println("X");

  if (!found) {
    Serial.println("Warning: Could not find optimal parameters! Using best available.");
  }
}

void calibrationSequence() {
  Serial.println("\n[Calibration Sequence]");
  Serial.println("Prepare calibration concentrations (lowest to highest).");
  Serial.println("For each, follow the instructions.");

  int concentrationIndex = 1;
  while (true) {
    Serial.print("\n=== Calibration for Concentration #"); Serial.print(concentrationIndex); Serial.println(" ===");
    Serial.println("Enter algae concentration or sample name for this calibration:");
    String sampleName = "";
    while (sampleName.length() == 0) {
      while (!Serial.available()) {
        if (checkForQuit()) return;
      }
      sampleName = Serial.readStringUntil('\n');
      sampleName.trim();
    }
    Serial.print("Sample: "); Serial.println(sampleName);
    Serial.println("1. REMOVE last concentration (manual, save it)");
    Serial.println("2. Insert blank solvent circuit, then press any key to run rinse cycle.");
    if (waitForUser() == 1) return;
    runPumpForSeconds(circulationTimeMs / 1000);

    Serial.println("3. EMPTY the tube fully (manual). Press any key when done.");
    if (waitForUser() == 1) return;

    Serial.println("4. CIRCULATE new calibration solution (pump ON, 20s to remove air pockets). Press any key to start.");
    if (waitForUser() == 1) return;
    runPumpForSeconds(circulationTimeMs / 1000);

    Serial.println("5. Measuring with FLOW (pump ON)...");
    float resultsFlow[3][nMeasurements]; // 0:Red, 1:Green, 2:Blue (clear channel for each color)
    setPumpSpeed(defaultPumpSpeed);
    for (int rep = 0; rep < nMeasurements; rep++) {
      if (checkForQuit()) { setPumpSpeed(0); return; }
      // Red LED on, measure clear
      setNeoPixelColor(bestLedR, 0, 0);
      delay(100);
      if (checkForQuit()) { setPumpSpeed(0); return; }
      resultsFlow[0][rep] = measureChannel("clear", bestIntegrationTime);
      setNeoPixelColor(0, 0, 0);
      // Green LED on, measure clear
      setNeoPixelColor(0, bestLedG, 0);
      delay(100);
      if (checkForQuit()) { setPumpSpeed(0); return; }
      resultsFlow[1][rep] = measureChannel("clear", bestIntegrationTime);
      setNeoPixelColor(0, 0, 0);
      // Blue LED on, measure clear
      setNeoPixelColor(0, 0, bestLedB);
      delay(100);
      if (checkForQuit()) { setPumpSpeed(0); return; }
      resultsFlow[2][rep] = measureChannel("clear", bestIntegrationTime);
      setNeoPixelColor(0, 0, 0);
    }
    setPumpSpeed(0);
    delay(800); // Wait a bit before next measurement
    Serial.println("Measuring with STATIONARY liquid (pump OFF)...");
    float resultsStationary[3][nMeasurements];
    for (int rep = 0; rep < nMeasurements; rep++) {
      if (checkForQuit()) return;
      // Red LED on, measure clear
      setNeoPixelColor(bestLedR, 0, 0);
      delay(100);
      if (checkForQuit()) return;
      resultsStationary[0][rep] = measureChannel("clear", bestIntegrationTime);
      setNeoPixelColor(0, 0, 0);
      // Green LED on, measure clear
      setNeoPixelColor(0, bestLedG, 0);
      delay(100);
      if (checkForQuit()) return;
      resultsStationary[1][rep] = measureChannel("clear", bestIntegrationTime);
      setNeoPixelColor(0, 0, 0);
      // Blue LED on, measure clear
      setNeoPixelColor(0, 0, bestLedB);
      delay(100);
      if (checkForQuit()) return;
      resultsStationary[2][rep] = measureChannel("clear", bestIntegrationTime);
      setNeoPixelColor(0, 0, 0);
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
    saveCalibrationData(concentrationIndex, &resultsFlow[0][0], &resultsStationary[0][0], avgsFlow, avgsStationary, nMeasurements, sampleName);

    Serial.print("Sample: "); Serial.println(sampleName);
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
}

double measureChannelManual(String color, int ledR, int ledG, int ledB, uint8_t integrationTime) {
  // Set the integration time directly
  tcs.setIntegrationTime(integrationTime);
  // Wait for old integration time
  delay(integrationTimeToMs(lastIntegrationTime));
  delay(integrationTimeToMs(integrationTime));
  lastIntegrationTime = integrationTime;
  
  setNeoPixelColor(ledR, ledG, ledB);
  delay(100); // Let light stabilize
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  setNeoPixelColor(0, 0, 0);
  if (color == "red") return (double)r;
  if (color == "green") return (double)g;
  if (color == "blue") return (double)b;
  if (color == "clear") return (double)c;
  return 0.0;
}

float measureChannel(String color, uint8_t integrationTime) {
  // Set integration time
  tcs.setIntegrationTime(integrationTime);
  // Wait for old integration time
  delay(integrationTimeToMs(lastIntegrationTime));
  delay(integrationTimeToMs(integrationTime));
  lastIntegrationTime = integrationTime;
  // Set all LEDs off
  setNeoPixelColor(0, 0, 0);

  if (color == "red") {
    setNeoPixelColor(bestLedR, 0, 0);
  } else if (color == "green") {
    setNeoPixelColor(0, bestLedG, 0);
  } else if (color == "blue") {
    setNeoPixelColor(0, 0, bestLedB);
  }
  delay(100); // let the light stabilize

  // Read TCS34725 data
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  setNeoPixelColor(0, 0, 0);

  if (color == "red") return (float)r;
  if (color == "green") return (float)g;
  if (color == "blue") return (float)b;
  if (color == "clear") return (float)c;

  return 0;
}

void setNeoPixelColor(uint8_t r, uint8_t g, uint8_t b) {
  // This function is implemented in main-calibration.cpp, but for modularity, you may want to move the Adafruit_NeoPixel object to a shared location or pass it as a parameter.
  extern Adafruit_NeoPixel strip;
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();
}

uint16_t integrationTimeToMs(uint8_t integrationTime) {
  switch (integrationTime) {
    case TCS34725_INTEGRATIONTIME_2_4MS: return 3;
    case TCS34725_INTEGRATIONTIME_24MS: return 24;
    case TCS34725_INTEGRATIONTIME_50MS: return 50;
    case TCS34725_INTEGRATIONTIME_60MS: return 60;
    case TCS34725_INTEGRATIONTIME_101MS: return 101;
    case TCS34725_INTEGRATIONTIME_120MS: return 120;
    case TCS34725_INTEGRATIONTIME_154MS: return 154;
    case TCS34725_INTEGRATIONTIME_180MS: return 180;
    case TCS34725_INTEGRATIONTIME_199MS: return 199;
    case TCS34725_INTEGRATIONTIME_240MS: return 240;
    case TCS34725_INTEGRATIONTIME_300MS: return 300;
    case TCS34725_INTEGRATIONTIME_360MS: return 360;
    case TCS34725_INTEGRATIONTIME_401MS: return 401;
    case TCS34725_INTEGRATIONTIME_420MS: return 420;
    case TCS34725_INTEGRATIONTIME_480MS: return 480;
    case TCS34725_INTEGRATIONTIME_499MS: return 499;
    case TCS34725_INTEGRATIONTIME_540MS: return 540;
    case TCS34725_INTEGRATIONTIME_600MS: return 600;
    case TCS34725_INTEGRATIONTIME_614MS: return 614;
    default: return 101; // fallback
  }
}

void balanceLedBrightness() {
  const uint16_t target = 45000;
  const uint8_t maxIterations = 40;
  const uint8_t minLed = 1;
  const uint8_t maxLed = 255;
  const uint8_t step = 4;
  
  bestGain = TCS34725_GAIN_16X;
  bestIntegrationTime = TCS34725_INTEGRATIONTIME_499MS;
  tcs.setGain(bestGain);
  tcs.setIntegrationTime(bestIntegrationTime);

  Serial.println("Insert BLANK solvent (no algae) into sensor.");
  Serial.println("Press any key when ready.");
  if (waitForUser() == 1) return;
  runPumpForSeconds(circulationTimeMs / 1000);

  int* leds[3] = {&bestLedR, &bestLedG, &bestLedB};
  const char* colors[3] = {"red", "green", "blue"};
  for (int c = 0; c < 3; c++) {
    uint8_t led = 200; // start at mid-brightness
    *leds[c] = led;
    for (uint8_t iter = 0; iter < maxIterations; iter++) {
      if (checkForQuit()) return;
      
      // Set only the current channel LED on
      if (c == 0) setNeoPixelColor(led, 0, 0);
      else if (c == 1) setNeoPixelColor(0, led, 0);
      else setNeoPixelColor(0, 0, led);
      delay(100);

      if (checkForQuit()) return;
      float val = measureChannel("clear", bestIntegrationTime);
      Serial.print("Balancing "); Serial.print(colors[c]);
      Serial.print(" LED, value: "); Serial.print(val);
      Serial.print(" (brightness: "); Serial.print(led); Serial.println(")");

      if (abs((int)val - (int)target) < 1000) break;
      if (val < target && led < maxLed - step) led += step;
      else if (val > target && led > minLed + step) led -= step;
      else break;

      *leds[c] = led;
    }

    Serial.print("Final "); Serial.print(colors[c]); Serial.print(" LED brightness: "); Serial.println(*leds[c]);
    setNeoPixelColor(0, 0, 0);
  }
  saveLightCalibration();
  Serial.println("LED brightness balanced and saved.");
}

void runPumpForSeconds(int seconds) {
  if (seconds <= 0) {
    Serial.println("Invalid time. Must be > 0.");
    return;
  }
  Serial.print("Running pump at speed ");
  Serial.print(defaultPumpSpeed);
  Serial.print(" for ");
  Serial.print(seconds);
  Serial.println(" seconds... (press 'q' to stop early)");
  setPumpSpeed(defaultPumpSpeed);
  unsigned long start = millis();
  unsigned long duration = (unsigned long)seconds * 1000UL;
  while (millis() - start < duration) {
    if (checkForQuit()) {
      Serial.println("Pump stopped early by user.");
      break;
    }
    delay(100); // check every 100ms
  }
  setPumpSpeed(0);
  Serial.println("Pump stopped.");
}
