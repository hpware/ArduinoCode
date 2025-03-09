#include "values.h"
#include <Arduino.h>
#include <WiFi.h>            
#include <PubSubClient.h>    
#include <ModbusMaster.h>    
#include <OneWire.h>
#include <DallasTemperature.h>
#include "func/screeninit.h"

WiFiClient espClient;
PubSubClient client(espClient);
ModbusMaster node;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void init() {
    Serial.begin(115200);
    pinMode(b1p, INPUT);
    pinMode(b2p, INPUT);
    pinMode(b3p, INPUT);
    screeninit();
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, LOW);
  
  Serial.println("[WiFi] 連線中...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] 連線成功！");
  Serial.print("[WiFi] 訊號強度: ");
  Serial.println(WiFi.RSSI());
  String wifiName = ssid;
  String ipAddr = WiFi.localIP(); 

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  pinMode(LED_32, OUTPUT);
  pinMode(LED_27, OUTPUT);
  digitalWrite(LED_32, LOW);
  digitalWrite(LED_27, LOW);
  
  node.begin(MODBUS_SLAVE_ID, Serial2);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  sensors.begin();
}