#ifndef NETWORK_H
#define NETWORK_H

#include "AdafruitIO.h"

// WiFi connection management
void connectToWiFi();

// Adafruit IO connectivity  
void connectToAdafruitIO(AdafruitIO* io);

// DNS utilities
int resolveDNS(const char* mqttHost);

// Time/NTP functions
void printLocalTime();

// SSL/TLS certificate for secure connections
extern const char* caCert;

#endif
