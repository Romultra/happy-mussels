// my_functions.h
#ifndef UTILS_H
#define UTILS_H

#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;
extern int nMeasurements;
extern uint8_t bestIntegrationTime;
extern tcs34725Gain_t bestGain;
extern Adafruit_TCS34725 tcs;

/// Call once from setup() to configure PWM on C1/C2
void initPumpPWM();

/// Drive the pump at the given percentage (60.0–100.0% safe range).
/// Prints a warning if % is outside [60.0, 100.0].
/// Uses floating point for finer duty cycle control.
bool setPumpIntensity(double intensityPercent);

bool setPumpIntensityBackward(double intensityPercent);

/// Drive the pump with 8-bit speed control (0-255).
/// Maps 0-255 to the pump's working range (60-100% duty cycle).
/// 0 = pump off, 255 = full speed.
bool setPumpSpeed(int speed8Bit);

bool setPumpSpeedBackward(int speed8Bit);

void setPeltier(bool on);

/// Read the thermistor temperature in Celsius (uses lookup table and Steinhart-Hart)
float readThermistorTemp();

/// Read the thermistor temperature in Celsius from a specified pin
float readThermistorTempOnPin(int pin);

/// Read the thermistor resistance in ohms (uses lookup table)
float readThermistorResistance();

/// Measure the intensity using the TCS34725 sensor and NeoPixel strip
float measureIntensity();

bool pumpVolume(double volumeML);


#endif