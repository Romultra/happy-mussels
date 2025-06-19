#include <config.h>
#include <WiFi.h>
#include <esp_wpa2.h>
#include <esp_wifi.h>
#include <utils.h>
#include <AdafruitIO.h>
#include <time.h>  // Added for getLocalTime() function

// Define the caCert variable - moved from header file to implementation file
const char* caCert = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

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
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  #endif
  
  // Wait for connecting with status updates
  for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.print(".");
    
    if (i % 10 == 9) {
      Serial.print(" [Status: ");
      Serial.print(WiFi.status());
      Serial.println("]");
    }
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println();
    
    // Print meaningful error based on status
    switch (WiFi.status()) {
      case WL_NO_SSID_AVAIL:
        Serial.println("ERROR: SSID not found. Check hotspot is on.");
        break;
      case WL_CONNECT_FAILED:
        Serial.println("ERROR: Connection failed. Check password.");
        break;
      case WL_DISCONNECTED:
        Serial.println("ERROR: Failed to connect to AP.");
        break;
      default:
        Serial.print("ERROR: Failed with status: ");
        Serial.println(WiFi.status());
    }
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println();
    Serial.println("FAILED to connect to wifi !");
    Serial.println("Check that:");
    Serial.println("1. Hotspot is turned on and in range");
    Serial.println("2. Credentials are correct");
    Serial.println("3. Hotspot allows new connections");
    Serial.println();
    Serial.println("Continuing without WiFi...");
  } else {
    Serial.println("");
    Serial.println("WiFi connected successfully!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  
  return;
}

// Function to connect to Adafruit IO using an AdafruitIO instance
void connectToAdafruitIO(AdafruitIO* io) {
  // Connect to Adafruit IO
  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io->connect();

  // wait for a connection
  while(io->status() < AIO_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  // we are connected
  Serial.println();
  Serial.println(io->statusText());

  return;
}

int resolveDNS(const char* mqttHost) {
  IPAddress resolvedIP;
  // Resolve the DNS for the MQTT host
  Serial.print("Resolving DNS for ");
  Serial.println(mqttHost);
  
  int dnsResult = WiFi.hostByName(mqttHost, resolvedIP);
  
  Serial.print("DNS lookup result: ");
  Serial.print(dnsResult);
  Serial.print(" - IP: ");
  
  if (dnsResult > 0) {
    Serial.println(resolvedIP);
  } else {
    Serial.println("Failed to resolve");
  }
  
  return dnsResult;
}

void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.print("Day of week: ");
  Serial.println(&timeinfo, "%A");
  Serial.print("Month: ");
  Serial.println(&timeinfo, "%B");
  Serial.print("Day of Month: ");
  Serial.println(&timeinfo, "%d");
  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");

  Serial.println("Time variables");
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  Serial.println(timeHour);
  char timeWeekDay[10];
  strftime(timeWeekDay,10, "%A", &timeinfo);
  Serial.println(timeWeekDay);
  Serial.println();
}



//Controlling the 12V pump.


#include <Arduino.h>
#include "utils.h"

// Only the pin numbers are global
const int C1_pin = 12;
const int C2_pin = 13;

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

void setPumpIntensity(int intensityPercent) {
  // warning if out of the 60–100% “safe” range
  if (intensityPercent < 60 || intensityPercent > 100) {
    Serial.printf("WARNING: pump intensity %d%% is out of range (60 to 100%%)\n",
                  intensityPercent);
  }
  // map 0–100% → 0–1023
  int maxDuty = (1 << 10) - 1;  // 10-bit resolution
  int duty    = map(intensityPercent, 0, 100, 0, maxDuty);

  // drive forward: PWM on C1, zero on C2
  // note: we repeat the same channel numbers as in initPumpPWM()
  ledcWrite(0, duty);
  ledcWrite(1, 0);

  Serial.printf("Pump @ %d%% → duty %d/%d\n", intensityPercent, duty, maxDuty);
}
