#include <Arduino.h>

// Pin definitions (adjust as needed for your board)
#define DAC_PIN 25  // DAC output pin (ESP32: 25 or 26)
#define ADC_PIN 32  // ADC input pin (ESP32 ADC1_CH4)

#define NUM_SAMPLES 50
#define ADC_MAX 4095
#define DAC_MAX 255

const float DAC_Vmax = 3.16;
const float DAC_Vmin = 0.09; // Not used
const float DAC_QUANTUM = DAC_Vmax / DAC_MAX;

uint16_t adc_read[DAC_MAX + 1];
float adc_V_lookup[ADC_MAX + 1];

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting calibration...");

  // Sweep DAC and record ADC readings
  for (int i = 0; i <= DAC_MAX; i++) {
    Serial.print("Samples acquired: ");
    Serial.print(i);
    Serial.print("/");
    Serial.println(DAC_MAX);

    // Write to DAC (ESP32 only: use dacWrite, else use PWM/analogWrite)
    dacWrite(DAC_PIN, i);
    delay(10); // ADC_DELAY

    uint32_t raw_sum = 0;
    for (int j = 0; j < NUM_SAMPLES; j++) {
      raw_sum += analogRead(ADC_PIN);
      delay(5); // DAC_DELAY
    }
    adc_read[i] = round((float)raw_sum / NUM_SAMPLES);
    Serial.print("  ADC reading: ");
    Serial.println(adc_read[i]);
  }

  // Build lookup table
  for (int i = 0; i <= ADC_MAX; i++) {
    // Count matches
    int count = 0;
    int idx = -1;
    for (int j = 0; j <= DAC_MAX; j++) {
      if (adc_read[j] == i) {
        count++;
        if (idx == -1) idx = j;
      }
    }
    Serial.print("Processing index ");
    Serial.println(i);
    if (count == 1) {
      Serial.println("  1 to 1 match!");
      adc_V_lookup[i] = DAC_QUANTUM * idx;
    } else if (count == 0) {
      Serial.println("  No match!");
      // Interpolate
      int range_min = -1, range_max = -1;
      int i_min = i - 1, i_max = i + 1;
      // Find range_min
      while (range_min == -1) {
        if (i_min < *std::min_element(adc_read, adc_read + DAC_MAX + 1)) {
          range_min = std::min_element(adc_read, adc_read + DAC_MAX + 1) - adc_read;
        } else {
          for (int j = 0; j <= DAC_MAX; j++) {
            if (adc_read[j] == i_min) {
              range_min = j;
              break;
            }
          }
        }
        i_min--;
      }
      // Find range_max
      while (range_max == -1) {
        if (i_max > *std::max_element(adc_read, adc_read + DAC_MAX + 1)) {
          range_max = std::max_element(adc_read, adc_read + DAC_MAX + 1) - adc_read;
        } else {
          for (int j = 0; j <= DAC_MAX; j++) {
            if (adc_read[j] == i_max) {
              range_max = j;
              break;
            }
          }
        }
        i_max++;
      }
      Serial.print("  i_min: "); Serial.print(i_min+1);
      Serial.print(" range_min: "); Serial.print(range_min);
      Serial.print(" i_max: "); Serial.print(i_max-1);
      Serial.print(" range_max: "); Serial.println(range_max);
      // Interpolate
      adc_V_lookup[i] = DAC_QUANTUM * (range_min + ((float)(range_max - range_min) / (i_max - i_min - 1)) * (i - (i_min + 1)));
    } else {
      Serial.println("  Multiple matches!");
      // Collapse
      int min_j = DAC_MAX, max_j = 0;
      for (int j = 0; j <= DAC_MAX; j++) {
        if (adc_read[j] == i) {
          if (j < min_j) min_j = j;
          if (j > max_j) max_j = j;
        }
      }
      adc_V_lookup[i] = DAC_QUANTUM * ((max_j - min_j) / 2.0 + min_j);
    }
    Serial.print("  adc_V :");
    Serial.print(adc_V_lookup[i], 5);
    Serial.println(" V");
  }

  Serial.println("----------------------------------------------------");
  Serial.println("----------------------------------------------------");
  // Print the lookup table
  for (int i = 0; i <= ADC_MAX; i++) {
    Serial.print(adc_V_lookup[i], 5);
    if (i < ADC_MAX) Serial.print(", ");
  }
  Serial.println();
}

void loop() {
  // Nothing to do
}
