#include <WiFi.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "time.h"
#include <U8g2lib.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>
#include <HardwareSerial.h>  // 顯式包含HardwareSerial庫

/*#define INA1 2  
#define INA2 4*/
// RX TX
#define RXD2 26
#define TXD2 25
// 新增 UART3 腳位
#define RXD3 5
#define TXD3 18
#define ONE_WIRE_BUS 19
// LIGHT FOR DEBUGGING
#define LED_32 32
#define LED_27 27
// PH Value stuff
#define MODBUS_SLAVE_ID 8
#define MODBUS_REGISTER 0x0002
// MOTOR
#define MOS 27
// WIFI
const char* ssid = "hel";
const char* password = "1234567890";
const char* mqtt_server = "120.102.36.38";
const int mqtt_port = 5007;
// NTP CLIENT VALUES
const char* ntpServer = "time.stdtime.gov.tw";
const long gmtoss = 8 * 3600;
const int dloss = 0;
// Button states
int b1s = 0;
int b2s = 0;
int b3s = 0;
// Button pins
int b1p = 33;
int b2p = 36;
int b3p = 39;
int sda = 21;
int scl = 22;

uint32_t modbusTimer = 0;
uint32_t tempTimer = 0;
uint32_t mqttTimer = 0;
uint32_t displayTimer = 0;
uint32_t feedCheckTimer = 0; // 檢查自動餵食的計時器
uint32_t wifiReconnectStartTime = 0;

const uint32_t modbusInterval = 500;  // 每500ms讀取一次pH值
const uint32_t tempInterval = 500;    // 每500ms讀取一次水溫
const uint32_t mqttInterval = 10000;  // 改為每10秒上傳一次MQTT
const uint32_t displayInterval = 50;  // 每50ms更新一次顯示，提高時間顯示的流暢度
const uint32_t feedCheckInterval = 1000; // 每1秒檢查一次是否需要餵食

float lastPH = 0.0;
float lastTemp = 0.0;
float feedResidue = 50.0;
int modbusErrorCount = 0;
const int modbusErrorThreshold = 3;
int feedMode = 0;      // 0:無 1:定時 2:週期
int feedTime = 0;      // 每天定時餵食的時間（小時）
int feedInterval = 0;  // 週期性餵食的間隔（小時）
int feedDuration = 2;  // 預設餵食時間為2秒

uint32_t lastFeedTime = 0; // 上次餵食的時間戳記（用於interval模式）
bool feedingScheduled = false; // 今天是否已經執行了定時餵食（用於daily模式）

// 濫用投餵保護機制
const int MAX_FEED_COUNT = 5;        // 一分鐘最多允許5次餵食
const uint32_t FEED_PROTECT_WINDOW = 60000; // 一分鐘的時間窗口(毫秒)
uint32_t feedCommandTimes[MAX_FEED_COUNT]; // 記錄最近5次餵食命令的時間
int feedCommandIndex = 0;            // 當前記錄索引
bool feedLimitReached = false;       // 是否達到餵食限制

bool initialized = false;
bool wifiReconnectRequested = false;
bool feedingInProgress = false;
uint32_t feedingStartTime = 0;

unsigned long lastB1PressTime = 0;
unsigned long lastB2PressTime = 0;
unsigned long lastB3PressTime = 0;
int lastB1State = LOW;
int lastB2State = LOW;
int lastB3State = LOW;
const unsigned long debounceDelay = 100;  // 消抖延遲時間（毫秒）

String modename = "";

// IP ADDR AND NETWORK SETTINGS
IPAddress ipAddr = "0.0.0.0";

// 宣告 Serial3
HardwareSerial Serial3(2);  // 使用UART2硬體通道，但命名為Serial3

WiFiClient espClient;
PubSubClient client(espClient);
ModbusMaster node;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, 0x3D);

// 初始化餵食命令計數器數組
void initFeedCommandTimes() {
  for (int i = 0; i < MAX_FEED_COUNT; i++) {
    feedCommandTimes[i] = 0;
  }
}

// 檢查是否超過餵食頻率限制
bool checkFeedLimit() {
  uint32_t currentTime = millis();
  
  // 記錄當前餵食時間
  feedCommandTimes[feedCommandIndex] = currentTime;
  feedCommandIndex = (feedCommandIndex + 1) % MAX_FEED_COUNT;
  
  // 檢查最早的記錄是否在一分鐘內
  uint32_t oldestTime = feedCommandTimes[(feedCommandIndex) % MAX_FEED_COUNT];
  if (oldestTime == 0) return false; // 還沒滿5次
  
  // 計算時間差
  uint32_t timeElapsed;
  if (currentTime >= oldestTime) {
    timeElapsed = currentTime - oldestTime;
  } else {
    // 處理millis溢出情況
    timeElapsed = (0xFFFFFFFF - oldestTime) + currentTime;
  }
  
  // 如果最老的記錄在一分鐘內，說明一分鐘內已經有5次餵食命令
  if (timeElapsed <= FEED_PROTECT_WINDOW) {
    Serial.println("[保護] 一分鐘內餵食次數過多，已暫時禁用餵食");
    feedLimitReached = true;
    return true;
  }
  
  return false;
}

void setup() {
  Serial.begin(115200);
  // CONFIG NTP CLIENT
  configTime(gmtoss, dloss, ntpServer);
  // SCREEN
  Wire.begin(sda, scl);

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_t0_22_tf);
  drawWiFiIcon(false);  // 顯示未連接的 WiFi 圖標
  u8g2.sendBuffer();
  // BUTTON
  pinMode(b1p, INPUT_PULLUP);
  pinMode(b2p, INPUT_PULLUP);
  pinMode(b3p, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);  // 使用剩餘的GPIO 23作為備用輸入
  pinMode(34, INPUT_PULLUP);  // 使用剩餘的GPIO 34作為備用輸入

  Serial.println("[WiFi] 連線中...");
  WiFi.begin(ssid, password);
  int wifiConnectAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiConnectAttempts < 20) {
    delay(500);
    Serial.print(".");
    wifiConnectAttempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    ipAddr = WiFi.localIP();
    Serial.println("\n[WiFi] 連線成功！");
    Serial.print("[WiFi] 訊號強度: ");
    Serial.println(WiFi.RSSI());

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
  } else {
    Serial.println("\n[WiFi] 連線失敗，繼續使用本地功能");
  }

  pinMode(LED_32, OUTPUT);
  pinMode(LED_27, OUTPUT);
  pinMode(MOS, OUTPUT);
  digitalWrite(LED_32, LOW);
  digitalWrite(LED_27, LOW);
  digitalWrite(MOS, LOW);

  node.begin(MODBUS_SLAVE_ID, Serial2);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial3.begin(9600, SERIAL_8N1, RXD3, TXD3);  // 啟用UART3
  sensors.begin();

  // 初始化餵食命令計數器
  initFeedCommandTimes();

  // OTA
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.setPort(3232);
    ArduinoOTA.setHostname("feeder-prod");
    ArduinoOTA.setPassword("esp32");
    ArduinoOTA.onStart([]() {
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH) {
                  type = "sketch";
                } else {
                  type = "filesystem";
                }
                Serial.println("Updaing content...");
              })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u$$\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
          Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
          Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
          Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
          Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
          Serial.println("End Failed");
        }
      });
    ArduinoOTA.begin();
  }

  // 初始化自動餵食計時器
  feedCheckTimer = millis();
  lastFeedTime = millis();
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();  // 恢復OTA處理
  }
    
  // 非阻塞式按鈕檢查
  checkButtons();

  // 非阻塞式WiFi重連處理
  handleWiFiReconnect();

  // 非阻塞式餵食處理
  handleFeeding();

  // 檢查自動餵食排程
  if (millis() - feedCheckTimer >= feedCheckInterval) {
    feedCheckTimer = millis();
    checkFeedSchedule();
  }

  if (WiFi.status() == WL_CONNECTED && !client.connected()) {
    reconnectMQTT();
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    client.loop();
  }

  // 定時讀取modbus數據
  if (millis() - modbusTimer >= modbusInterval) {
    modbusTimer = millis();
    readModbusData();
  }

  // 定時讀取溫度
  if (millis() - tempTimer >= tempInterval) {
    tempTimer = millis();
    readTemperature();
  }

  // 定時發送MQTT數據（10秒一次）
  if (WiFi.status() == WL_CONNECTED && millis() - mqttTimer >= mqttInterval) {
    mqttTimer = millis();
    sendMQTT();
    sendFeedResidue();
  }

  // 頻繁更新顯示（50ms一次）
  if (millis() - displayTimer >= displayInterval) {
    displayTimer = millis();
    displayIndex2();
  }
  
  // 檢查和重置餵食限制
  if (feedLimitReached) {
    // 檢查最早的記錄是否已經超過一分鐘
    uint32_t currentTime = millis();
    uint32_t oldestTime = feedCommandTimes[feedCommandIndex];
    
    if (currentTime - oldestTime > FEED_PROTECT_WINDOW) {
      feedLimitReached = false;
      Serial.println("[保護] 餵食限制已重置");
    }
  }
  
  // 處理串口3的數據
  if (Serial3.available()) {
    String data = Serial3.readStringUntil('\n');
    Serial.println("[UART3] 收到數據: " + data);
    // 這裡可以添加對UART3接收數據的處理
  }
}

// 檢查自動餵食排程的函數
void checkFeedSchedule() {
  if (feedMode == 0 || feedingInProgress) {
    return; // 如果模式為"無"或正在餵食中，直接返回
  }

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("[時間] 無法獲取時間");
    return; // 無法獲取時間，直接返回
  }

  // 模式1: 定時(Daily)
  if (feedMode == 1) {
    // 檢查當前是否為設定的餵食時間
    if (timeinfo.tm_hour == feedTime && timeinfo.tm_min == 0 && timeinfo.tm_sec == 0) {
      if (!feedingScheduled) {
        Serial.println("[自動餵食] 定時餵食時間到了");
        startFeeding();
        feedingScheduled = true; // 標記今天已執行過定時餵食
      }
    } else if (timeinfo.tm_hour != feedTime) {
      // 如果不是餵食時間，重置標記（為明天做準備）
      feedingScheduled = false;
    }
  }
  // 模式2: 週期(Interval)
  else if (feedMode == 2 && feedInterval > 0) {
    // 計算自上次餵食後經過的小時數
    uint32_t hoursSinceLastFeed = (millis() - lastFeedTime) / (1000 * 60 * 60);
    
    // 如果經過的時間超過設定的間隔，進行餵食
    if (hoursSinceLastFeed >= feedInterval) {
      Serial.println("[自動餵食] 週期餵食時間到了");
      Serial.print("上次餵食後經過: ");
      Serial.print(hoursSinceLastFeed);
      Serial.println(" 小時");
      startFeeding();
      lastFeedTime = millis(); // 更新上次餵食時間
    }
  }
}

void checkButtons() {
  b1s = digitalRead(b1p);
  b2s = digitalRead(b2p);
  b3s = digitalRead(b3p);

  // 獲取當前時間
  unsigned long currentTime = millis();

  // 按鈕1：餵食 - 修改為在任何情況下都能直接啟動馬達
  if (b1s != lastB1State) {
    if ((currentTime - lastB1PressTime) > debounceDelay) {
      if (b1s == HIGH && !feedingInProgress) {
        // 檢查是否達到餵食頻率限制
        if (!feedLimitReached && !checkFeedLimit()) {
          // 直接啟動馬達，不需要網路連接
          startFeeding();
          
          // 如果有網路，則發送MQTT消息
          if (WiFi.status() == WL_CONNECTED && client.connected()) {
            client.publish("test_feeder", "1");
          }
        } else {
          Serial.println("[保護] 餵食頻率過高，操作被拒絕");
        }
      }
      lastB1PressTime = currentTime;
    }
    lastB1State = b1s;
  }
  
  // 按鈕2：重新連接WiFi
  if (b2s != lastB2State) {
    if ((currentTime - lastB2PressTime) > debounceDelay) {
      if (b2s == HIGH && !wifiReconnectRequested && !feedingInProgress) {
        wifiReconnectRequested = true;
        wifiReconnectStartTime = millis();
        WiFi.disconnect();
        Serial.println("[WiFi] 開始重新連接...");
      }
      lastB2PressTime = currentTime;
    }
    lastB2State = b2s;
  }
  
  // 按鈕3
  if (b3s != lastB3State) {
    if ((currentTime - lastB3PressTime) > debounceDelay) {
      if (b3s == HIGH) {
        setup();
      }
      lastB3PressTime = currentTime;
    }
    lastB3State = b3s;
  }
}

void handleWiFiReconnect() {
  if (wifiReconnectRequested) {
    uint32_t elapsed = millis() - wifiReconnectStartTime;
    if (elapsed < 1000) {
      // 等待斷開連接
    } else if (elapsed < 1100) {
      WiFi.begin(ssid, password);
      Serial.println("[WiFi] 嘗試重新連接...");
    } else if (elapsed > 4100) {
      wifiReconnectRequested = false;
      Serial.println("[WiFi] 重新連接流程完成");
    }
  }
}

void handleFeeding() {
  if (feedingInProgress) {
    uint32_t elapsed = millis() - feedingStartTime;
    // 使用從MQTT接收到的feedDuration變數
    if (elapsed >= feedDuration * 1000) {  // feedDuration單位為秒，轉換為毫秒
      // 停止馬達
      digitalWrite(MOS, LOW);
      Serial.println("[Feeder] 餵食完成！");
      Serial.print("[Feeder] 餵食持續時間: ");
      Serial.print(feedDuration);
      Serial.println(" 秒");
      
      // 更新餘餌量
      float consumptionRate = 0.08;  // 4% 投餌消耗量
      feedResidue -= feedResidue * consumptionRate;
      if (feedResidue < 0) {
        feedResidue = 0;  // 確保不會低於 0
      }
      Serial.print("[Feeder] 餘餵食量: ");
      Serial.println(feedResidue);

      // 如果有網路連接，發送 MQTT 狀態更新
      if (WiFi.status() == WL_CONNECTED && client.connected()) {
        client.publish("test_feeder", "0");
        client.publish("test_lastfeed", "1");
        delay(100);  // 小延遲，不會影響顯示
        client.publish("test_lastfeed", "0");
        sendFeedResidue();  // 更新餘量
      }

      // 如果是手動餵食，也更新自動餵食的間隔計時
      if (feedMode == 2) {
        lastFeedTime = millis();
      }

      feedingInProgress = false;
    }
  }
}

void readModbusData() {
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

void readTemperature() {
  sensors.requestTemperatures();
  lastTemp = sensors.getTempCByIndex(0);
  Serial.print("[溫度] 水溫: ");
  Serial.println(lastTemp);
}

// MQTT Area
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

  String message2 = "";
  for (int i = 0; i < length; i++) {
    message2 += (char)payload[i];
  }

  if (strcmp(topic, "test_feeder") == 0 && strcmp(message, "1") == 0) {
    // 檢查是否達到餵食頻率限制
    if (!feedLimitReached && !checkFeedLimit()) {
      startFeeding();
    } else {
      Serial.println("[保護] 已拒絕MQTT餵食請求，頻率過高");
      if (client.connected()) {
        client.publish("test_feedblock", "1"); // 通知已阻止餵食請求
      }
    }
  }

  if (strcmp(topic, "test_feedmodel") == 0) {
    int newFeedMode = atoi(message);
    if (newFeedMode != feedMode) {
      feedMode = newFeedMode;
      // 模式變更時重置相關變數
      feedingScheduled = false;
      lastFeedTime = millis();
      
      Serial.print("[MQTT] 餵食模式改為: ");
      if (feedMode == 0) {
        Serial.println("無");
        modename = "None";
      } else if (feedMode == 1) {
        Serial.println("定時 (每天" + String(feedTime) + "點)");
        modename = "Daily";
      } else if (feedMode == 2) {
        Serial.println("週期 (每" + String(feedInterval) + "小時)");
        modename = "Interval";
      }
    }
  } else if (strcmp(topic, "test_feedtime") == 0) {
    feedTime = atoi(message);
    Serial.print("[MQTT] 設定定時餵食時間: ");
    Serial.print(feedTime);
    Serial.println("點");
    // 模式時間變更時重置排程標記
    if (feedMode == 1) {
      feedingScheduled = false;
    }
  } else if (strcmp(topic, "test_feedinterval") == 0) {
    feedInterval = atoi(message);
    Serial.print("[MQTT] 設定週期餵食間隔: ");
    Serial.print(feedInterval);
    Serial.println("小時");
    // 間隔變更時重置上次餵食時間
    if (feedMode == 2) {
      lastFeedTime = millis();
    }
  } else if (strcmp(topic, "test_feeding_time") == 0) {
    feedDuration = atoi(message);
    Serial.print("[MQTT] 接收到新的餵食時間設定: ");
    Serial.print(feedDuration);
    Serial.println(" 秒");
  }
  
  if (strcmp(topic, "test_feedmodel") == 0) {
    if (message2.toInt() == 0) {
      modename = "None";
    } else if (message2.toInt() == 1) {
      modename = "Daily";
    } else if (message2.toInt() == 2) {
      modename = "Interval";
    }
  }

  if (strcmp(topic, "test_Feeder_APP") == 0 && strcmp(message, "on") == 0) {
    Serial.println("[MQTT] 收到 test_Feeder_APP on, 回傳 test_Feeder_device on");
    client.publish("test_Feeder_device", "on");
  }

  if (strcmp(topic, "Feeder_APP") == 0 && strcmp(message, "on") == 0) {
    Serial.println("[MQTT] 收到 Feeder_APP on, 回傳 test_Feeder_device on");
    client.publish("test_Feeder_device", "on");
  }
  
  // 重置餵食限制命令
  if (strcmp(topic, "test_reset_limit") == 0 && strcmp(message, "1") == 0) {
    feedLimitReached = false;
    initFeedCommandTimes();
    Serial.println("[保護] 餵食限制已手動重置");
    if (client.connected()) {
      client.publish("test_feedblock", "0");
    }
  }
}

void sendMQTT() {
  char phMessage[10], tempMessage[10];
  dtostrf(lastPH, 4, 2, phMessage);
  dtostrf(lastTemp, 4, 1, tempMessage);

  Serial.print("[MQTT] 傳送 pH 值: ");
  Serial.println(phMessage);
  client.publish("test_ph", phMessage);

  Serial.print("[MQTT] 傳送水溫: ");
  Serial.println(tempMessage);
  client.publish("test_watertemp", tempMessage);
}

void sendFeedResidue() {
  char feedResidueMessage[10];
  sprintf(feedResidueMessage, "%d", (int)feedResidue);
  client.publish("test_feed_residue", feedResidueMessage);
  Serial.print("[MQTT] 傳送餘餵食量: ");
  Serial.println(feedResidueMessage);
}

void reconnectMQTT() {
  // 避免長時間阻塞，只嘗試一次
  Serial.println("[MQTT] 嘗試連線...");
  if (client.connect("ESP32_Client")) {
    Serial.println("[MQTT] 連線成功！");
    client.subscribe("test_feeder");
    client.subscribe("test_feedmodel");
    client.subscribe("test_feedtime");
    client.subscribe("test_feedinterval");
    client.subscribe("test_feeding_time");
    client.subscribe("test_Feeder_APP");
    client.subscribe("Feeder_APP");
    client.subscribe("test_reset_limit"); // 訂閱重置限制的主題
  } else {
    Serial.println("[MQTT] 連線失敗，下次循環再試...");
  }
}

void startFeeding() {
  if (!feedingInProgress) {
    Serial.println("[Feeder] 開始餵食...");
    Serial.print("[Feeder] 設定的餵食時間: ");
    Serial.print(feedDuration);
    Serial.println(" 秒");
    
    // 啟動馬達
    digitalWrite(MOS, HIGH);
    feedingInProgress = true;
    feedingStartTime = millis();
  }
}

// Display Component
void displayIndex2() {
  u8g2.clearBuffer();

  // 取得 NTP 時間
  static int lastSecond = -1;
  static String lastDateStr = "";
  static String lastTimeStr = "";
  struct tm timeinfo;

  bool timeUpdated = false;
  if (getLocalTime(&timeinfo)) {
    if (timeinfo.tm_sec != lastSecond) {
      // 秒數變化，更新緩存的時間字符串
      lastSecond = timeinfo.tm_sec;
      lastDateStr = String(timeinfo.tm_year + 1900) + "-" + String(timeinfo.tm_mon + 1) + "-" + String(timeinfo.tm_mday);
      lastTimeStr = String(timeinfo.tm_hour < 10 ? "0" + String(timeinfo.tm_hour) : String(timeinfo.tm_hour))
                    + ":" + String(timeinfo.tm_min < 10 ? "0" + String(timeinfo.tm_min) : String(timeinfo.tm_min))
                    + ":" + String(timeinfo.tm_sec < 10 ? "0" + String(timeinfo.tm_sec) : String(timeinfo.tm_sec));
      timeUpdated = true;
    }
  }

  // 無論秒數是否變化，都顯示最後一次成功獲取的時間
  u8g2.setFont(u8g2_font_t0_22_tf);
  if (lastDateStr != "" && lastTimeStr != "") {
    u8g2.drawStr(0, 14, lastDateStr.c_str());
    u8g2.drawStr(0, 30, lastTimeStr.c_str());
  } else {
    u8g2.drawStr(0, 14, "No Time Data");
    u8g2.drawStr(0, 30, "Connecting...");
  }

  u8g2.setFont(u8g2_font_ncenB08_tr);        // 替換為另一個字體
  String phStr = "PH:" + String(lastPH, 1);  // 保留1位小數
  u8g2.drawStr(0, 45, phStr.c_str());
  String mode = "Mode";  // 保留1位小數
  u8g2.drawStr(90, 45, mode.c_str());
  String wtStr = "WT:" + String(lastTemp, 1) + "*C";  // 保留1位小數
  u8g2.drawStr(0, 60, wtStr.c_str());
  if (modename == "None" || modename == "Daily") {
    u8g2.drawStr(90, 60, modename.c_str());
  } else {
    u8g2.drawStr(85, 60, modename.c_str());
  }

  // 顯示 WiFi 狀態
  bool wifiConnected = WiFi.status() == WL_CONNECTED;
  bool mqttConnected = client.connected();

  drawWiFiIcon(wifiConnected);
  

  u8g2.sendBuffer();
}

// WIFI icon
void drawWiFiIcon(bool connected) {
  int x = 110, y = 5;  // 右上角位置

  if (connected) {
    // 連線時顯示 WiFi 信號
    for (int i = 0; i < 4; i++) {
      int barHeight = (i + 1) * 2;
      int barWidth = 3;
      int barX = x + i * 4;
      u8g2.drawBox(barX, y + 10 - barHeight, barWidth, barHeight);
    }
  } else {
    // 未連線時顯示 WiFi 圖示 + 斜線
    for (int i = 0; i < 4; i++) {
      int barHeight = (i + 1) * 2;
      int barWidth = 3;
      int barX = x + i * 4;
      u8g2.drawFrame(barX, y + 10 - barHeight, barWidth, barHeight);
    }
    // 斜線（表示未連線）
    u8g2.drawLine(x, y, x + 15, y + 12);
  }
}
