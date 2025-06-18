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

#endif