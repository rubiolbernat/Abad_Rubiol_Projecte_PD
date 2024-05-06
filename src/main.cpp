#include <Arduino.h>
#include "SPIFFS.h"
#include <WiFi.h>
#include <WebServer.h>

const char* ssid     = "OPPO BernArtNet";
const char* password = "putaespanya1";

WebServer server;

// Declaració del subprograma
String fileContent;
void handle_root() {
  server.send(200, "text/html", fileContent);
}

// Implementació del subprograma per llegir fitxer
String readFile(String fileName) {
  File file = SPIFFS.open(fileName);
  String content = "";

  if (!file) {
    Serial.println("Failed to open file for reading");
    return content;
  }

  while (file.available()) {
    char c = file.read();
    content += c;
  }

  file.close();
  return content;
}

void setup() {
  Serial.begin(115200);

  //Files setup
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  fileContent = readFile("/index.html"); // Assigna el contingut a la variable global
  delay(1000);

  //Web setup
  Serial.println("Try Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected successfully");
  Serial.print("Got IP: ");
  Serial.println(WiFi.localIP());  //Show ESP32 IP on serial

  server.begin();
  Serial.println("HTTP server started");
  
  server.on("/", handle_root);

  // Cridar al subprograma per llegir el contingut del fitxer "text.txt"
  
  // Imprimir el contingut llegit
  //Serial.println("File Content:");
  //Serial.println(fileContent);

  // Resta del teu codi
}

void loop() {
  server.handleClient();
}

