#include <Arduino.h>
#include <WiFi.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include "secrets.h"
#include "config.h"
#include "network.h"

WebServer server(80);

void handleStatusLog() {
  File file = SPIFFS.open("/status_log.csv", FILE_READ);
  if (!file) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  while (file.available()) {
    String chunk = file.readStringUntil('\n');
    server.sendContent(chunk + "\n");
  }
  file.close();
  server.sendContent("");
}

void handleEraseSPIFFS() {
  Serial.println("Erase SPIFFS requested via /erase_spiffs endpoint");
  bool result = SPIFFS.format();
  if (result) {
    Serial.println("SPIFFS erased successfully. All files deleted.");
    server.send(200, "text/plain", "SPIFFS erased successfully. All files deleted.");
  } else {
    Serial.println("Failed to erase SPIFFS.");
    server.send(500, "text/plain", "Failed to erase SPIFFS.");
  }
}

void handleListFiles() {
  String output = "Files on SPIFFS:\n";
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while (file) {
    output += String(file.name()) + "\t" + String(file.size()) + " bytes\n";
    file = root.openNextFile();
  }
  server.send(200, "text/plain", output);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  connectToWiFi();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  server.on("/status_log.csv", HTTP_GET, handleStatusLog);
  server.on("/erase_spiffs", HTTP_GET, handleEraseSPIFFS);
  server.on("/list_files", HTTP_GET, handleListFiles);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
