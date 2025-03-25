#include <Wire.h>
#include <MPU6050.h>
#include <HardwareSerial.h>
#include "Audio.h"
#include "SD.h"
#include "FS.h"
#include "SPI.h"
#include <FastLED.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi 配置
const char* WIFI_SSID = "hel";
const char* WIFI_PASSWORD = "1234567890";

// MQTT 配置
const char* MQTT_BROKER = "120.102.36.38";
const int MQTT_PORT = 5007;
const char* MQTT_TOPIC_FALL = "fall_alarm";
const char* MQTT_TOPIC_VOLUME = "hat_volume ";  // 新增音量控制主題
const char* MQTT_CLIENT_ID = "ESP32_FallDetection";
bool voice_gender = true;
// 網絡客戶端
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// MPU6050 傳感器
MPU6050 mpu;

// UART 通信 (HUB8735)
#define RXD2 17
#define TXD2 16
HardwareSerial mySerial(2);

// I2S 音頻播放引腳
#define I2S_DOUT 26
#define I2S_BCLK 27
#define I2S_LRC 25

Audio audio;

// SD CARD 引腳
#define SD_CS 5
#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCK 18

// WS2813 LED 控制
#define LED_PIN 4
#define WIDTH 10
#define HEIGHT 5
#define NUM_LEDS (WIDTH * HEIGHT)
CRGB leds[NUM_LEDS];

// 全域變數
int counter = 0;
String receivedData = "";
String Data = "";
bool fallDetected = false;
unsigned long fallDetectedTime = 0;
const unsigned long fallDetectionTimeout = 5000;
bool audioPlaying = false;
unsigned long audioStartTime = 0;
const unsigned long audioMinDuration = 2000;
int currentVolume = 21;  // 預設音量

// 防止重複播放同樣辨識結果的變數
String lastDetectedObject = "";
unsigned long lastDetectionTime = 0;
const unsigned long sameObjectTimeout = 5000;  // 5秒內不重複播放同一辨識結果

// 物體類型冷卻時間
const unsigned long objectTypeTimeout = 5000;                            // 5秒鐘的冷卻時間
unsigned long lastObjectTypeDetectionTime[7] = { 0, 0, 0, 0, 0, 0, 0 };  // 索引0未使用，1-6表示物體類型

// LED閃爍控制
bool ledFlashing = false;
unsigned long lastLedToggle = 0;
bool ledState = false;
const unsigned long ledFlashInterval = 500;  // 調整為500ms

// 修正LED索引對應 (適用蛇形排列)
int XY(int x, int y) {
  if (y % 2 == 0) {
    return (HEIGHT - 1 - y) * WIDTH + (WIDTH - 1 - x);  // 偶數行右到左
  } else {
    return (HEIGHT - 1 - y) * WIDTH + x;  // 奇數行左到右
  }
}

// MQTT 消息回調函數
void mqttCallback(char* topic, uint8_t* message, unsigned int length) {
  // 顯示接收到的 topic
  Serial.print("收到主題: ");
  Serial.println(topic);

  // 顯示接收到的消息
  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
  Serial.print("消息內容: ");
  Serial.println(messageTemp);

  // 處理音量控制
  if (String(topic) == "hat_volume") {
    int newVolume = messageTemp.toInt();
    if (newVolume >= 0 && newVolume <= 100) {
      // 使用 map() 把 0~100 的範圍轉換成 0~21
      currentVolume = map(newVolume, 0, 100, 0, 21);
      audio.setVolume(currentVolume);
      Serial.printf("音量已調整為 %d (對應原始輸入 %d)\n", currentVolume, newVolume);
    } else {
      Serial.println("無效的音量值，請提供 0 到 100 之間的數字。");
    }
  } else if (String(topic) == "hat_sound") {
    int newValue = messageTemp.toInt();  // 修正變數名稱
    if (newValue == 0) {                 // 使用 `==` 來比較
      voice_gender = true;
    } else {
      voice_gender = false;
    }
  }
}

// MQTT 連接函數
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("嘗試連接MQTT Broker...");
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println("已連接");
      // 訂閱音量控制主題
      mqttClient.subscribe("hat_sound");
      mqttClient.subscribe("hat_volume");
    } else {
      Serial.print("失敗，RC=");
      Serial.print(mqttClient.state());
      Serial.println(" 5秒後重試");
      delay(5000);
    }
  }
}

// 發布跌倒警報到MQTT
void publishFallAlarm(int16_t ax, int16_t ay, int16_t az) {
  mqttClient.publish(MQTT_TOPIC_FALL, "楊立輝跌倒了");
  Serial.println("已發布跌倒警報");
}

void playAudio(const char* filename) {
  // 只有當音頻沒在播放時才播放新音頻
  if (!audioPlaying) {
    if (audio.isRunning()) {
      audio.stopSong();
      delay(50);  // 等待音頻停止
    }

    if (!audio.connecttoFS(SD, filename)) {
      Serial.println("無法播放 " + String(filename) + "！");
    } else {
      Serial.println("正在播放 " + String(filename));
      audioPlaying = true;        // 設置為正在播放
      audioStartTime = millis();  // 記錄開始播放時間
    }
  } else {
    // 如果已經在播放音樂，這裡可以選擇跳過或者等待播放結束
    Serial.println("音頻正在播放，跳過播放。");
  }
}

// 繪製圓形和中間橫線
void drawCircleWithLine(bool on) {
  FastLED.clear();
  CRGB color = on ? CRGB::Red : CRGB::Black;     // 紅色圓形
  CRGB white = on ? CRGB::White : CRGB::Black;   // 白色橫線

  // 外圍圓形框架 (紅色)
  leds[XY(2, 0)] = color;
  leds[XY(3, 0)] = color;
  leds[XY(4, 0)] = color;
  leds[XY(5, 0)] = color;
  leds[XY(3, 1)] = color;
  leds[XY(8, 1)] = color;
  leds[XY(1, 2)] = color;
  leds[XY(6, 2)] = color;
  leds[XY(3, 3)] = color;
  leds[XY(8, 3)] = color;
  leds[XY(2, 4)] = color;
  leds[XY(3, 4)] = color;
  leds[XY(4, 4)] = color;
  leds[XY(5, 4)] = color;

  // **中間橫線 (白色)**
  for (int x = 2; x < 6; x++) { 
    leds[XY(x, 2)] = white;
  }

  FastLED.show();
}

void setup() {
  Serial.begin(115200);

  // WiFi連接
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi已連接");

  // MQTT設置
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);  // 設置MQTT回調函數

  mySerial.begin(115200, SERIAL_8N1, TXD2, RXD2);
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);

  // 初始化MPU6050
  Wire.begin();
  mpu.initialize();

  // 初始化 SD 卡
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  SPI.setFrequency(1000000);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD卡初始化失敗！");
    return;
  }

  // 初始化 I2S 音頻
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(currentVolume);  // 設置初始音量

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 連接失敗！");
    while (1)
      ;  // 停止執行
  }
}

void loop() {
  // MQTT保持連接
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  audio.loop();
  
  // 更新閃爍LED
  if (ledFlashing && (millis() - lastLedToggle >= ledFlashInterval)) {
    lastLedToggle = millis();
    ledState = !ledState;
    drawCircleWithLine(ledState);
  }

  // 檢查音頻是否已經播放完畢 - 每次循環都檢查
  if (audioPlaying && !audio.isRunning()) {
    // 音頻播放結束，重設播放狀態
    audioPlaying = false;
    Serial.println("停止播放");
  }

  // 讀取 MPU6050 數據
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  String mpuData = "ACC: " + String(ax) + "," + String(ay) + "," + String(az) + " | GYR: " + String(gx) + "," + String(gy) + "," + String(gz);
  mySerial.println(mpuData);

  // 檢查跌倒是否已超時
  if (fallDetected && (millis() - fallDetectedTime > fallDetectionTimeout)) {
    fallDetected = false;
    Serial.println("Fall detection timeout, returning to normal operation");
    stopLedFlashing();
  }

  // 接收 UART 數據
  if (!fallDetected && !audioPlaying && mySerial.available() > 0) {
    counter = 0;
    Data = "";

    while (mySerial.available() > 0 && counter < 3) {
      char str = mySerial.read();
      Data += str;
      counter++;
    }

    if (Data.length() >= 3) {
      Serial.println("Data = " + Data);

      // 解析物體類型 (第一個數字)
      int objectType = Data.substring(0, 1).toInt();

      // 檢查這個物體類型是否在冷卻期間內
      bool isWithinCooldown = false;
      if (objectType >= 1 && objectType <= 6) {
        isWithinCooldown = (millis() - lastObjectTypeDetectionTime[objectType] < objectTypeTimeout);
      }

      // 檢查是否是與上次相同的辨識結果，且在5秒時間範圍內
      bool isSameObject = (Data == lastDetectedObject);
      bool isWithinTimeFrame = (millis() - lastDetectionTime < sameObjectTimeout);

      if ((isSameObject && isWithinTimeFrame) || isWithinCooldown) {
        Serial.println("相同物件或相同類型物件在冷卻時間內，跳過播放");
      } else {
        // 更新最後辨識的物件和時間
        lastDetectedObject = Data;
        lastDetectionTime = millis();

        // 更新該物體類型的最後偵測時間
        if (objectType >= 1 && objectType <= 6) {
          lastObjectTypeDetectionTime[objectType] = millis();
        }

        // 播放對應的音頻
        if (voice_gender) {
          // 原始語音播放邏輯
          if (Data == "1,M") {
            playAudio("FrontPerson.mp3");
          } else if (Data == "1,R") {
            playAudio("Lperson.mp3");
          } else if (Data == "1,L") {
            playAudio("Rperson.mp3");
          }
          // 汽車
          else if (Data == "2,M") {
            playAudio("Mtruck.mp3");
          } else if (Data == "2,R") {
            playAudio("Ltruck.mp3");
          } else if (Data == "2,L") {
            playAudio("Rtruck.mp3");
          }
          // 機車
          else if (Data == "4,M") {
            playAudio("Mcar.mp3");
          } else if (Data == "4,R") {
            playAudio("Lcar.mp3");
          } else if (Data == "4,L") {
            playAudio("Rcar.mp3");
          } else if (Data == "5,M") {
            playAudio("Mmoto.mp3");
          } else if (Data == "5,R") {
            playAudio("Rmoto.mp3");
          } else if (Data == "5,L") {
            playAudio("Lmoto.mp3");
          } else if (Data == "6,M") {
            playAudio("Mlight.mp3");
          } else if (Data == "6,R") {
            playAudio("Llight.mp3");
          } else if (Data == "6,L") {
            playAudio("Rlight.mp3");
          }
        } else {  //男聲
          if (Data == "1,M") {
            playAudio("BMperson.mp3");
          } else if (Data == "1,R") {
            playAudio("BLperson.mp3");
          } else if (Data == "1,L") {
            playAudio("BRperson.mp3");
          }
          // 汽車
          else if (Data == "2,M") {
            playAudio("BMtruck.mp3");
          } else if (Data == "2,R") {
            playAudio("BLtruck.mp3");
          } else if (Data == "2,L") {
            playAudio("BRtruck.mp3");
          }
          // 機車
          else if (Data == "4,M") {
            playAudio("BMcar.mp3");
          } else if (Data == "4,R") {
            playAudio("BLcar.mp3");
          } else if (Data == "4,L") {
            playAudio("BRcar.mp3");
          } else if (Data == "5,M") {
            playAudio("BMmoto.mp3");
          } else if (Data == "5,R") {
            playAudio("BRmoto.mp3");
          } else if (Data == "5,L") {
            playAudio("BLmoto.mp3");
          } else if (Data == "6,M") {
            playAudio("BMlight.mp3");
          } else if (Data == "6,R") {
            playAudio("BLlight.mp3");
          } else if (Data == "6,L") {
            playAudio("BRlight.mp3");
          }
        }
      }

      counter = 0;
    }

    delay(20);
  }
  // 檢測跌倒
  if (az < -1000 && !fallDetected) {
    Serial.print("陳勞板收到:");
    Serial.println(fallDetected);
    Serial.println("有人跌倒");
    fallDetected = true;
    fallDetectedTime = millis();

    // 發布MQTT警報
    publishFallAlarm(ax, ay, az);

    startLedFlashing();
    // 跌倒警報應該總是播放，不考慮重複檢測
    if (voice_gender) {
      if (!audioPlaying) {
        playAudio("Warning.mp3");
      } else {
        Serial.println("有問題啦:) 陳勞板");
      }
    } else {
      if (!audioPlaying) {
        playAudio("B_Warning.mp3");
      } else {
        Serial.println("有問題啦:) 陳勞板");
      }
    }
  } else {
    Serial.print("陳勞板收到:");
    Serial.print(az);
    Serial.println(fallDetected);
  }

  delay(100);
}

// 啟動LED閃爍
void startLedFlashing() {
  ledFlashing = true;
  lastLedToggle = 0;  // 強制第一次立即閃爍
  ledState = true;
  drawCircleWithLine(true);  // 初始顯示
}

// 停止LED閃爍
void stopLedFlashing() {
  ledFlashing = false;
  FastLED.clear();
  FastLED.show();
}