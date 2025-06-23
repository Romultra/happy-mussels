#ifndef NETWORK_H
#define NETWORK_H

// WiFi connection management
void connectToWiFi();

// DNS utilities
int resolveDNS(const char* mqttHost);

// Time/NTP functions
void printLocalTime();

// SSL/TLS certificate for secure connections
extern const char* caCert;

#endif
