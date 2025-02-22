#include <WiFi.h>
#include <NetworkClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "json.h"

const char *ssid = "ESPTek";
const char *password = "6543KJVpf";

WebServer server(80);

const int BLOCK_ITM = 16;

void WifiInit() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());
  server.begin();
}

void setup() {
  Serial.begin(115200);
  WifiInit();
    pinMode(BLOCK_ITM, INPUT);
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", "<!DOCTYPE html><html><head><title>ESP32 Server</title></head><body><h1>Sent From An ESP32</h1></body><style>html,body {text-align:center;}</style>");
  });
  server.on("/wifi/scan", HTTP_GET, []() {
    int n = WiFi.scanNetworks();
    String output = "";
    output += "<!DOCTYPE html><html><head><title>WiFi Scan</title></head><body><div class='wifi'><h1>Wifi Scanner</h1>";
    for (int i = 0; i < n; ++i) {
      output += "<h3>SSID: " + WiFi.SSID(i) + "</h3><h4>Signal: " + String(WiFi.RSSI(i)) + "</h4>";
    }
    output += "</div></body><style>html {text-align:center;justify-content:center;align-self:center;} .wifi{ display:block; margin: 10px; }</style></html>";
    server.send(200, "text/html", output);
  });
  server.on("/dete", HTTP_GET, []() {
    String output = "";
    output += "{ \n   'GPIO':" + String(digitalRead(BLOCK_ITM)) + "}";
    server.send(200, "text/json", output);
  });
}

void loop() {
  server.handleClient();
  delay(2);
}
