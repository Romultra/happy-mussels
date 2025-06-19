// my_functions.h
#ifndef UTILS_H
#define UTILS_H

#include "AdafruitIO.h"  // Include the header file for AdafruitIO

// Function declaration
void connectToWiFi();
void connectToAdafruitIO(AdafruitIO* io);
void printLocalTime();

// Declare the certificate as extern to avoid multiple definitions
extern const char* caCert;




/// GPIO pins for pump driver channels (visible to entire sketch)
extern const int C1_pin;
extern const int C2_pin;

/// Call once from setup() to configure PWM on C1/C2
void initPumpPWM();

/// Drive the pump at the given percentage (60–100% safe range).
/// Prints a warning if % is outside [60,100].
void setPumpIntensity(int intensityPercent);


#endif