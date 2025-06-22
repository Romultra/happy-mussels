#include <Arduino.h>
#include <utils.h>
#include <math.h>

//Controlling the 12V pump.

// Only the pin numbers are global
constexpr int C1_pin = 12;
constexpr int C2_pin = 13;

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
}

bool setPumpIntensity(double intensityPercent) {
  // warning if out of the 60–100% "safe" range
  if (intensityPercent < 60.0 || intensityPercent > 100.0) {
    Serial.printf("ERROR: pump intensity %.2f%% is out of range (60.0 to 100.0%%)\n",
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

bool setPumpSpeed(int speed8Bit) {  // Map 0-255 pump speed to 60-100% duty cycle range
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
    // Map 1-255 to 60-100% duty cycle with floating point precision
  double dutyCyclePercent = 60.0 + ((double)(speed8Bit - 1) / 254.0) * 40.0;
  
  Serial.printf("Mapping %d/255 speed to %.2f%% duty cycle\n", speed8Bit, dutyCyclePercent);
  return setPumpIntensity(dutyCyclePercent);
}

// ---- Thermistor/Temperature Reading Functions ----

// Example lookup table (replace with your calibration data)
const float adc_V_lookup[1024] = {
  // Fill this with your calibration data from main-calibration.cpp output
  // e.g. 0.0185, 0.0494, ... up to 1023 values
};

constexpr int TEMP_ADC_PIN = 32; // Adjust as needed
constexpr int NUM_TEMP_SAMPLES = 25;
constexpr float NOM_RES = 10000.0;
constexpr float SER_RES = 9820.0;
constexpr float TEMP_NOM = 25.0;
constexpr float THERM_B_COEFF = 3950.0;
constexpr int ADC_MAX_VAL = 4095;
constexpr float ADC_VMAX = 3.16;

float readThermistorTemp() {
  uint32_t raw_sum = 0;
  for (int i = 0; i < NUM_TEMP_SAMPLES; i++) {
    raw_sum += analogRead(TEMP_ADC_PIN);
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
