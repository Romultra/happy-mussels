#include <secrets.h>
#pragma once

// === Wi-Fi Connection Mode ===
// Choose the Wi-Fi between DTU, PERSONAL_1, or PERSONAL_2, WOKWI
//#define DTU
//#define PERSONAL_1
#define PERSONAL_2
//#define WOKWI

#ifdef DTU
  #define WIFI_SSID DTU_SSID
  // WPA2 Enterprise credentials
  #define WIFI_USERNAME DTU_USERNAME
  #define WIFI_PASSWORD DTU_PASSWORD
#elif defined(PERSONAL_1)
  // Personal Wi-Fi credentials
  #define WIFI_SSID PERSONAL_1_SSID
  #define WIFI_PASSWORD PERSONAL_1_PASSWORD
#elif defined(PERSONAL_2)
  // Personal Wi-Fi credentials
  #define WIFI_SSID PERSONAL_2_SSID
  #define WIFI_PASSWORD PERSONAL_2_PASSWORD
#elif defined(WOKWI)
  // Wokwi Wi-Fi credentials
  #define WIFI_SSID "Wokwi-GUEST"
  #define WIFI_PASSWORD ""

#endif

// === Adafruit IO Credentials ===
#define IO_USERNAME SECRET_IO_USERNAME
#define IO_KEY SECRET_IO_KEY