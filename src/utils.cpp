#include <config.h>
#include <WiFi.h>
#include <esp_wpa2.h>
#include <utils.h>

void connectToWiFi() {
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);   // Set WiFi to station mode
  
  // WPA2 Enterprise setup
  WiFi.disconnect(true); // Reset WiFi
  #ifdef DTU
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)WIFI_USERNAME, strlen(WIFI_USERNAME));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)WIFI_USERNAME, strlen(WIFI_USERNAME));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)WIFI_PASSWORD, strlen(WIFI_PASSWORD));

  WiFi.begin(WIFI_SSID);
  esp_wifi_sta_wpa2_ent_enable();
  #elif defined(PERSONAL_1) || defined(PERSONAL_2) || defined(WOKWI)
  // For Personal Wi-Fi or Wokwi, use standard WPA2 PSK
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  #endif
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  return;
}