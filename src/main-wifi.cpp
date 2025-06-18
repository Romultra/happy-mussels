#include <Arduino.h>
#include <WiFi.h>

void printWiFiDiagnostics() {
  Serial.println("\n----- WiFi Diagnostics -----");
  Serial.print("WiFi Status: ");
  Serial.println(WiFi.status());
  Serial.print("WiFi Mode: ");
  Serial.println(WiFi.getMode());
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("WiFi Channel: ");
  Serial.println(WiFi.channel());
  Serial.print("TX Power: ");
  Serial.println(WiFi.getTxPower());
  Serial.println("---------------------------\n");
  Serial.flush();
}

void scanNetworks() {
  Serial.println("\n==== WiFi Network Scanner ====");
  
  // Reset WiFi completely with longer delays
  Serial.println("Resetting WiFi system...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(1000);
  
  // Configure WiFi in station mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(1000);
  
  // Print diagnostics before scan
  printWiFiDiagnostics();
  
  // Use synchronous scan instead of async for better reliability
  Serial.println("Starting synchronous WiFi scan...");
  Serial.flush();
  
  // Use synchronous scan with default parameters
  int networkCount = WiFi.scanNetworks(false);
  
  Serial.println("Scan complete!");
  if (networkCount == WIFI_SCAN_FAILED) {
    Serial.println("WiFi scan failed");
  } else if (networkCount == 0) {
    Serial.println("No networks found");
  } else {
    Serial.print(networkCount);
    Serial.println(" networks found:");
    
    for (int i = 0; i < networkCount; i++) {
      Serial.printf("%2d: %-32.32s | CH:%2d | %ddBm | %s\n", 
                   i+1, 
                   WiFi.SSID(i).c_str(),
                   WiFi.channel(i),
                   WiFi.RSSI(i),
                   (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Encrypted");
      Serial.flush();
      delay(5);
    }
  }
  
  // Clean up after scan
  WiFi.scanDelete();
  Serial.println("\nScan process completed!");
  Serial.println("==============================");
  Serial.flush();
}

// Alternative scan method trying different parameters
void tryScanWithAlternateParameters() {
  Serial.println("\n==== Trying Alternative Scan Method ====");
  
  // Try a fresh WiFi reset with different power cycling
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(500);
  
  // Set WiFi to STA mode explicitly
  WiFi.mode(WIFI_STA);
  delay(500);
  
  // Try with synchronous scan and specific channel (2.4GHz)
  Serial.println("Scanning channel 1...");
  int networkCount = WiFi.scanNetworks(false, true, false, 500, 1);
  
  if (networkCount > 0) {
    Serial.print("Found ");
    Serial.print(networkCount);
    Serial.println(" networks on channel 1");
    
    for (int i = 0; i < networkCount; i++) {
      Serial.printf("  %s (%.1f dBm)\n", 
                   WiFi.SSID(i).c_str(), 
                   WiFi.RSSI(i));
    }
  } else {
    Serial.println("No networks found on channel 1");
  }
  
  WiFi.scanDelete();
  Serial.println("====================================");
  Serial.flush();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial connection
  }
  delay(2000);
  
  Serial.println("\nESP32 WiFi Network Scanner");
  Serial.println("-------------------------");
  Serial.flush();
  
  // Try both scan methods
  scanNetworks();
  delay(1000);
  tryScanWithAlternateParameters();
}

void loop() {
  // Rescan every 30 seconds
  static unsigned long lastScanTime = 0;
  
  if (millis() - lastScanTime > 30000) {
    Serial.println("\nStarting new scan cycle...");
    
    // Try both methods alternately
    static bool useAlternate = false;
    if (useAlternate) {
      tryScanWithAlternateParameters();
    } else {
      scanNetworks();
    }
    useAlternate = !useAlternate;
    
    lastScanTime = millis();
  }
  
  delay(1000);
}