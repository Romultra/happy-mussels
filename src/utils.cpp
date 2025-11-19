#include <Arduino.h>
#include <config.h>
#include <utils.h>
#include <math.h>
#include "adc_lookup_table.h"
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>

//Controlling the 12V pump.

void initPumpPWM() {
  // All PWM constants are local to this function
  const int pwmFreq       = 5000;  // 5 kHz
  const int pwmChannel1   = 0;     // LEDC ch0 → C1
  const int pwmChannel2   = 1;     // LEDC ch1 → C2
  const int pwmResolution = 10;    // 10-bit (0–1023)

  // configure the two PWM channels and attach pins
  ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
  ledcAttachPin(C1_pin, pwmChannel1);

  ledcSetup(pwmChannel2, pwmFreq, pwmResolution);
  ledcAttachPin(C2_pin, pwmChannel2);

  // Configure relay pin as output
  pinMode(RELAY_PIN, OUTPUT);
  // Optionally, set relay off at startup
  digitalWrite(RELAY_PIN, LOW);
}

// Set Peltier relay ON (true) or OFF (false)
void setPeltier(bool on) {
  // Relay ON = LOW, OFF = HIGH
  digitalWrite(RELAY_PIN, on ? LOW : HIGH);
}

bool setPumpIntensity(double intensityPercent) {
  // warning if out of the 50–100% "safe" range
  if (intensityPercent < 50.0 || intensityPercent > 100.0) {
    Serial.printf("ERROR: pump intensity %.2f%% is out of range (50.0 to 100.0%%)\n",
                  intensityPercent);
    return false; // Return false to indicate failure
  }
  // map 0–100% → 0–1023 with floating point precision
  int maxDuty = (1 << 10) - 1;  // 10-bit resolution
  int duty    = (int)((intensityPercent / 100.0) * maxDuty + 0.5);  // Round to nearest integer

  // drive forward: PWM on C1, zero on C2
  // note: we repeat the same channel numbers as in initPumpPWM()
  ledcWrite(0, duty);
  ledcWrite(1, 0);

  Serial.printf("Pump @ %.2f%% → duty %d/%d\n", intensityPercent, duty, maxDuty);
  return true; // Return true to indicate success
}

// Set pump intensity in reverse direction (50-100% duty cycle)
bool setPumpIntensityBackward(double intensityPercent) {
  // warning if out of the 50–100% "safe" range
  if (intensityPercent < 50.0 || intensityPercent > 100.0) {
    Serial.printf("ERROR: pump intensity %.2f%% is out of range (50.0 to 100.0%%)\n", intensityPercent);
    return false; // Return false to indicate failure
  }
  // map 0–100% → 0–1023 with floating point precision
  int maxDuty = (1 << 10) - 1;  // 10-bit resolution
  int duty    = (int)((intensityPercent / 100.0) * maxDuty + 0.5);  // Round to nearest integer

  // drive reverse: PWM on C2, zero on C1
  ledcWrite(0, 0);
  ledcWrite(1, duty);

  Serial.printf("Pump (reverse) @ %.2f%% → duty %d/%d\n", intensityPercent, duty, maxDuty);
  return true; // Return true to indicate success
}

bool setPumpSpeed(int speed8Bit) {  // Map 0-255 pump speed to 50-100% duty cycle range
  // 0 = pump off, 255 = full speed
  
  if (speed8Bit < 0 || speed8Bit > 255) {
    Serial.printf("ERROR: pump speed %d is out of range (0 to 255)\n", speed8Bit);
    return false;
  }
  
  if (speed8Bit == 0) {
    // Turn pump off
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    Serial.println("Pump OFF (0/255 speed)");
    return true;
  }
  // Map 1-255 to 50-100% duty cycle with floating point precision
  double dutyCyclePercent = 50.0 + ((double)(speed8Bit - 1) / 254.0) * 50.0;
  
  Serial.printf("Mapping %d/255 speed to %.2f%% duty cycle\n", speed8Bit, dutyCyclePercent);
  return setPumpIntensity(dutyCyclePercent);
}

// Set pump speed in reverse direction (0 = off, 255 = full speed reverse)
bool setPumpSpeedBackward(int speed8Bit) {
  // 0 = pump off, 255 = full speed reverse
  if (speed8Bit < 0 || speed8Bit > 255) {
    Serial.printf("ERROR: pump speed %d is out of range (0 to 255)\n", speed8Bit);
    return false;
  }
  if (speed8Bit == 0) {
    // Turn pump off
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    Serial.println("Pump OFF (0/255 speed, reverse)");
    return true;
  }
  // Map 1-255 to 50-100% duty cycle with floating point precision
  double dutyCyclePercent = 50.0 + ((double)(speed8Bit - 1) / 254.0) * 50.0;
  Serial.printf("[REVERSE] Mapping %d/255 speed to %.2f%% duty cycle\n", speed8Bit, dutyCyclePercent);
  // Reverse: PWM on C2, zero on C1
  int maxDuty = (1 << 10) - 1;  // 10-bit resolution
  int duty    = (int)((dutyCyclePercent / 100.0) * maxDuty + 0.5);
  ledcWrite(0, 0);
  ledcWrite(1, duty);
  Serial.printf("Pump (reverse) @ %.2f%% → duty %d/%d\n", dutyCyclePercent, duty, maxDuty);
  return true;
}

// ---- Thermistor/Temperature Reading Functions ----

constexpr int NUM_TEMP_SAMPLES = 1000;
constexpr float NOM_RES = 10000.0;
constexpr float SER_RES = 10010.0;
constexpr float TEMP_NOM = 25;
constexpr float THERM_B_COEFF = 3950.0;
constexpr int ADC_MAX_VAL = 4095;
constexpr float ADC_VMAX = 3.16;

float readThermistorTemp() {
  uint32_t raw_sum = 0;
  for (int i = 0; i < NUM_TEMP_SAMPLES; i++) {
    raw_sum += analogRead(THERM_PIN);
  }
  float raw_avg = (float)raw_sum / NUM_TEMP_SAMPLES;
  Serial.print("raw_avg = "); Serial.println(raw_avg);

  // Lookup voltage
  int idx = round(raw_avg);
  if (idx < 0) idx = 0;
  if (idx > ADC_MAX_VAL) idx = ADC_MAX_VAL;
  float V_measured = adc_V_lookup[idx];
  Serial.print("V_measured = "); Serial.println(V_measured, 5);

  // Convert to resistance
  float raw_scaled = ADC_MAX_VAL * V_measured / ADC_VMAX;
  float resistance = (SER_RES * raw_scaled) / (ADC_MAX_VAL - raw_scaled);
  Serial.print("Thermistor resistance: "); Serial.print(resistance); Serial.println(" ohms");

  // Steinhart-Hart equation
  float steinhart = log(resistance / NOM_RES) / THERM_B_COEFF;
  steinhart += 1.0 / (TEMP_NOM + 273.15);
  steinhart = (1.0 / steinhart) - 273.15;
  return steinhart;
}

float readThermistorResistance() {
  uint32_t raw_sum = 0;
  for (int i = 0; i < NUM_TEMP_SAMPLES; i++) {
    raw_sum += analogRead(THERM_PIN);
  }
  float raw_avg = (float)raw_sum / NUM_TEMP_SAMPLES;
  int idx = round(raw_avg);
  if (idx < 0) idx = 0;
  if (idx > ADC_MAX_VAL) idx = ADC_MAX_VAL;
  float V_measured = adc_V_lookup[idx];
  float raw_scaled = ADC_MAX_VAL * V_measured / ADC_VMAX;
  float resistance = (SER_RES * raw_scaled) / (ADC_MAX_VAL - raw_scaled);
  return resistance;
}

// Copy of readThermistorTemp, but takes pin as input
float readThermistorTempOnPin(int pin) {
  uint32_t raw_sum = 0;
  for (int i = 0; i < NUM_TEMP_SAMPLES; i++) {
    raw_sum += analogRead(pin);
  }
  float raw_avg = (float)raw_sum / NUM_TEMP_SAMPLES;
  int idx = round(raw_avg);
  if (idx < 0) idx = 0;
  if (idx > ADC_MAX_VAL) idx = ADC_MAX_VAL;
  float V_measured = adc_V_lookup[idx];
  float raw_scaled = ADC_MAX_VAL * V_measured / ADC_VMAX;
  float resistance = (SER_RES * raw_scaled) / (ADC_MAX_VAL - raw_scaled);
  float steinhart = log(resistance / NOM_RES) / THERM_B_COEFF;
  steinhart += 1.0 / (TEMP_NOM + 273.15);
  steinhart = (1.0 / steinhart) - 273.15;
  return steinhart;
}

// Helper: Convert integration time code to ms
static uint16_t integrationTimeToMs(uint8_t integrationTime) {
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

// Helper: Set NeoPixel color (single pixel)
static void setNeoPixelColor(Adafruit_NeoPixel& strip, uint8_t r, uint8_t g, uint8_t b) {
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();
}

uint8_t bestIntegrationTime = TCS34725_INTEGRATIONTIME_240MS;
tcs34725Gain_t bestGain = TCS34725_GAIN_60X;

// TCS34725 sensor setup (default: 101ms, 1x gain)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(bestIntegrationTime, bestGain);

#define NEOPIXEL_NUM 1
Adafruit_NeoPixel strip(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

uint8_t ledB = 167;

// Helper: Measure a channel (returns blue, still flow)
static float measureChannelBlueStill() {
  delay(integrationTimeToMs(bestIntegrationTime));
  setNeoPixelColor(strip, 0, 0, ledB);
  delay(100); // let the light stabilize
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  setNeoPixelColor(strip, 0, 0, 0);
  return (float)c; // clear channel intensity
}

int nMeasurements = 5; // Number of repeated measurements for averaging
// Main util: Average measured intensity for still flow, blue LED
float measureIntensity() {
  float sum = 0;
  for (int i = 0; i < nMeasurements; i++) {
    sum += measureChannelBlueStill();
    delay(50); // small delay between measurements
  }
  return sum / nMeasurements;
}

const double FLOW_RATE_ML_PER_SEC = 1355.0 / 60; // 80% duty cycle

// Pump a given volume (in mL) at 80% duty cycle (1.355 L/min = 1355 mL/min)
// Returns true if successful
bool pumpVolume(double volumeML) {
  if (volumeML == 0) return false;
  double durationSec = fabs(volumeML) / FLOW_RATE_ML_PER_SEC;
  unsigned long durationMs = (unsigned long)(durationSec * 1000.0);

  if (volumeML > 0) {
    // Forward
    setPumpIntensity(80.0);
    delay(durationMs);
    setPumpIntensity(0.0); // Turn off pump
    Serial.printf("Pumped %.2f mL FORWARD at 80%% duty cycle (%.2f sec)\n", volumeML, durationSec);
  } else {
    // Reverse
    setPumpIntensityBackward(80.0);
    delay(durationMs);
    setPumpIntensityBackward(0.0); // Turn off pump
    Serial.printf("Pumped %.2f mL REVERSE at 80%% duty cycle (%.2f sec)\n", -volumeML, durationSec);
  }
  return true;
}
