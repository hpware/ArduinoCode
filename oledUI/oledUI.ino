#include <WiFi.h>            
#include <PubSubClient.h>    
#include <ModbusMaster.h>    
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define INA1 2  
#define INA2 4
#define RXD2 26  
#define TXD2 25  
#define ONE_WIRE_BUS 19  
#define LED_32 32  
#define LED_27 27  
#define MODBUS_SLAVE_ID 8  
#define MODBUS_REGISTER 0x0002  
const char* ssid = "hel";
const char* password = "1234567890";
const char* mqtt_server = "120.102.36.38";
const int mqtt_port = 5007;
int b1s = 0;
int b2s = 0;
int b3s = 0;
int b1p = 33;
int b2p = 34;
int b3p = 36;
int sda = 21;
int scl = 22;
int dw = 128;
int dh = 64;
uint32_t modbusTimer = 0;
uint32_t tempTimer = 0;
uint32_t mqttTimer = 0;
const uint32_t interval = 500;
bool feederActive = false;
uint32_t feederTimer = 0;
const uint32_t feederInterval = 30000;
float lastPH = 0.0;
float lastTemp = 0.0;
float feedResidue = 50.0;  
int modbusErrorCount = 0;
const int modbusErrorThreshold = 3;
int feedMode = 0;       
int feedTime = 0;       
int feedInterval = 0;   
int feedDuration = 2;   
unsigned long lastFeedTime = 0;
bool initialized = false;
int refresh = 0;
String dataStr = "No data"; 
bool dataFail = false; 

String wifiName = ssid;
IPAddress ipAddr = "0.0.0.0"; 

String date = "2024-12-1";
int hourVal = 00;
int minuteVal = 00;
int secVal = 00;
String dataminVal ="";
String datahrVal = "";
int phData = 7.2;
int tempData = 25;
WiFiClient espClient;
PubSubClient client(espClient);
ModbusMaster node;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Adafruit_SSD1306 display(dw, dh, &Wire, -1);

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("\n[MQTT] 收到訊息: ");
  Serial.print(topic);
  Serial.print(" -> ");

  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  Serial.println(message);

  
  if (strcmp(topic, "test_feeder") == 0 && strcmp(message, "1") == 0) {
    startFeeding();
  }

  if (strcmp(topic, "test_feedmodel") == 0) {
    feedMode = atoi(message);
  } else if (strcmp(topic, "test_feedtime") == 0) {
    feedTime = atoi(message);
  } else if (strcmp(topic, "test_feedinterval") == 0) {
    feedInterval = atoi(message);
  } else if (strcmp(topic, "test_feeding_time") == 0) {
    feedDuration = atoi(message);
  }

  if (strcmp(topic, "test_Feeder_APP") == 0 && strcmp(message, "on") == 0) {
  Serial.println("[MQTT] 收到 test_Feeder_APP on, 回傳 test_Feeder_device on");
  client.publish("test_Feeder_device", "on");
  }

  if (strcmp(topic, "Feeder_APP") == 0 && strcmp(message, "on") == 0) {
  Serial.println("[MQTT] 收到 Feeder_APP on, 回傳 test_Feeder_device on");
  client.publish("test_Feeder_device", "on");
  }
}


void sendFeedResidue() {
  char feedResidueMessage[10];
  dtostrf(feedResidue, 4, 2, feedResidueMessage);  
  client.publish("test_feed_residue", feedResidueMessage);
  Serial.print("[MQTT] 傳送餘餵食量: ");
  Serial.println(feedResidueMessage);
}

void setup() {
  Serial.begin(115200);

  
  Wire.begin(sda,scl);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    Serial.println("SSD1306 startup error");
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10,10);
  display.println("Booting...");
  display.display();
  delay(500);
  display.clearDisplay();
  display.display();

  
  pinMode(b1s, INPUT);
  pinMode(b2s, INPUT);
  pinMode(b2s, INPUT);
  
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
  ipAddr = WiFi.localIP();
  Serial.println("\n[WiFi] 連線成功！");
  Serial.print("[WiFi] 訊號強度: ");
  Serial.println(WiFi.RSSI());

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

void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  sendFeedResidue();
  client.loop();

  if (millis() - modbusTimer >= interval) {
    modbusTimer = millis();
    uint8_t result = node.readHoldingRegisters(MODBUS_REGISTER, 1);
    if (result == node.ku8MBSuccess) {
      lastPH = node.getResponseBuffer(0) / 100.0;
      modbusErrorCount = 0;
      Serial.print("[Modbus] pH 值: ");
      Serial.println(lastPH);
    } else {
      modbusErrorCount++;
      Serial.println("[Modbus] 讀取失敗");
      if (modbusErrorCount >= modbusErrorThreshold) {
        digitalWrite(LED_32, HIGH);
      }
    }
    node.clearResponseBuffer();
  }

  if (millis() - tempTimer >= interval) {
    tempTimer = millis();
    sensors.requestTemperatures();
    lastTemp = sensors.getTempCByIndex(0);
    Serial.print("[溫度] 水溫: ");
    Serial.println(lastTemp);
  }

  if (millis() - mqttTimer >= interval) {
    mqttTimer = millis();
    sendMQTT();
  }
  displaydisplay();
}

void sendMQTT() {
  char phMessage[10], tempMessage[10];
  dtostrf(lastPH, 4, 2, phMessage);
  dtostrf(lastTemp, 4, 2, tempMessage);

  Serial.print("[MQTT] 傳送 pH 值: ");
  Serial.println(phMessage);
  client.publish("test_ph", phMessage);

  Serial.print("[MQTT] 傳送水溫: ");
  Serial.println(tempMessage);
  client.publish("test_watertemp", tempMessage);
}

void startFeeding() {
  Serial.println("[Feeder] 開始餵食...");
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, LOW);
  delay(feedDuration * 1000);  
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, LOW);
  Serial.println("[Feeder] 餵食完成！");

  
  float randomPercentage = random(140, 161) / 1000.0;  

  
  feedResidue -= feedResidue * randomPercentage;
  if (feedResidue < 0) {
    feedResidue = 0;  
  }

  
  Serial.print("[Feeder] 餘餵食量: ");
  Serial.println(feedResidue);

  
  client.publish("test_feeder", "0");
  client.publish("test_lastfeed", "1");
  client.publish("test_lastfeed", "0");
  
  sendFeedResidue();  
  delay(500);
  client.publish("test_feeder", "0");

}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.println("[MQTT] 重新連線中...");
    if (client.connect("ESP32_Client")) {
      Serial.println("[MQTT] 連線成功！");
      client.subscribe("test_feeder");
      client.subscribe("test_feedmodel");
      client.subscribe("test_feedtime");
      client.subscribe("test_feedinterval");
      client.subscribe("test_feeding_time");
    } else {
      Serial.println("[MQTT] 連線失敗，5 秒後重試...");
      delay(5000);
    }
  }
}

void displaydisplay() {
  displayindex();
  delay(1000);
}



void displayindex() {
  
  display.setCursor(10,1);
  display.print(date);
  display.setTextSize(2);
  display.setCursor(6, 10);
  
  
  if (secVal == 60 || !initialized) {
      secVal = 0;
      minuteVal += 1;
      if (!initialized) {
          minuteVal -= 1;
      }
      if (String(minuteVal).length() == 1) {
          dataminVal = "0" + String(minuteVal);
      } else {
          dataminVal = String(minuteVal);
      }
      if (String(hourVal).length() == 1) {
          datahrVal = "0" + String(hourVal);
      } else {
          datahrVal = String(hourVal);
      }
      display.fillRect(6, 10, 120, 20, BLACK);
      display.print(datahrVal + ":" + dataminVal); 
  } else {
      secVal += 1;
  }
  
  if (minuteVal == 60) {
      minuteVal = 00;
  }
  
  if (hourVal == 24) {
      hourVal = 00;
  }
  display.setTextSize(1); 
  display.setCursor(6,28);
  display.print("WiFi:");
  if (!dataFail) {
      display.print(ssid);
      display.print("\n");
  } else {
      display.print("Error");
      display.print("\n");
  }
  display.setCursor(6,37);
  display.print("IP:");
  if (!dataFail) { 
      display.print(ipAddr);
      
      display.print("\n");
  } else {
      
      display.print("Error");
      display.print("\n");
  }
  /**if (!initialized) { 
      display.setCursor(23,44); 
      display.print("Feed");
      downsvg(25, 50, 1);
      display.setCursor(56,44);
      disdplay.print("RST");
      downsvg(55, 50, 1);
      display.setCursor(82, 44);
      display.print("UDT");
      downsvg(81,50,1);
      initialized = true;
  }*/
  if (lastPH != phData || !initialized) {
      display.setCursor(70,4);
      display.print(phData);
      display.print(" ph");
      phData = lastPH;
  }
  if (tempData != lastTemp || !initialized) {
      display.setCursor(70,15);
      display.print(tempData);
      display.print(" C");
      tempData = lastTemp;
  }
  display.display();
  if (refresh == 3600) { 
      refresh = 0;
      initialized = false;
  } else {
      refresh += 1;
  }
}