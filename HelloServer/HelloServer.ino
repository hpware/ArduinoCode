#include <WiFi.h>
#include <NetworkClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>

const char *ssid = "hel";
const char *password = "1234567890";
const int beeppin = 0;
const int balsepin = 1;
const int balsepind = 2;
const int lightpin = 3;
WebServer server(80);

void handleRoot() {
  int sensorState = analogRead(balsepin);
  int sensorStated = digitalRead(balsepind);
  String msg = "";
  msg += "<h1>DATA</h1>";
  msg += "<meta http-equiv='refresh' content='2,url=' />";
  msg += "Analog: ";
  msg += sensorState;
  msg += " Digital: ";
  msg += sensorStated;
  server.send(200, "text/html", msg);
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void setup(void) {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  pinMode(beeppin, OUTPUT);
  pinMode(balsepin, INPUT);
  pinMode(balsepind, INPUT);
  beep(50); 
  beep(50);
  delay(1000);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop(void) {
  server.handleClient();
  delay(2);  //allow the cpu to switch to other tasks
}
void beep(unsigned char delayms) {
  analogWrite(beeppin, 20);
  delay(delayms);
  analogWrite(beeppin, 0);
  delay(delayms);
}
