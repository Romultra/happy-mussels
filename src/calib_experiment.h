#pragma once
#include <Arduino.h>
#include <Adafruit_TCS34725.h>

void autoCalibrateLight();
void calibrationSequence();
double measureChannelManual(String color, int ledR = 0, int ledG = 0, int ledB = 0, uint8_t integrationTime = TCS34725_INTEGRATIONTIME_101MS);
float measureChannel(String color, uint8_t integrationTime = TCS34725_INTEGRATIONTIME_101MS);
void setNeoPixelColor(uint8_t r, uint8_t g, uint8_t b);
uint16_t integrationTimeToMs(uint8_t integrationTime);
void balanceLedBrightness();
extern int runPumpForSeconds(int seconds);
