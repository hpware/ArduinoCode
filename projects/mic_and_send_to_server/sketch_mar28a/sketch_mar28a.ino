#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "Audio.h"
#include "FS.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SD.h>
#include "Fetch.h"
#include <SPI.h>
// WIFI.h
// Makes the device connect to a WiFi router or hotspot.
void connectWiFi(const char* ssid, const char* passphrase) {
    // Disconnecting WiFi if it"s already connected.
    WiFi.disconnect();
    // Setting it to Station mode which basically scans for nearby WiFi routers or hotspots.
    WiFi.mode(WIFI_STA);
    // Begin connecting to WiFi using the provided ssid and passphrase.
    WiFi.begin(ssid, passphrase);
    // Print a debug log to Serial.
    Serial.printf("\nDevice is connecting to WiFi using SSID %s and Passphrase %s.\n", ssid, passphrase);
    // Keep looping until the WiFi is not connected.
    while (WiFi.status() != WL_CONNECTED) {
        // Print dots in a horizontal line to the Serial, showing the WiFi is trying to connect.
        Serial.print(".");
        delay(1000);
        // Blink LED very fast, showing the WiFi is trying to connect.
        // blinkN(10);
    }
    // Stop the LED blinking, showing the WiFi is successfully connected.
    // blinkStop();
    // Print debug logs to Serial.
    Serial.println("WiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP()); // Local IP Address of the ESP8266 device.
}

// 腳位定義
#define RX_PIN 16
#define TX_PIN 17
#define BAUD_RATE 115200
#define YELLOW_LED_PIN 2
#define RED_LED_PIN 4

// SD 卡和音訊腳位
#define SD_CS 5
#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCK 18
#define I2S_BCLK 27  // PCM5102 的 BCK 腳位
#define I2S_LRCK 25  // PCM5102 的 LCK 腳位
#define I2S_DOUT 26  // PCM5102 的 DIN 腳位

// 16 進制命令
uint8_t command[] = { 0xEF, 0xAA, 0x12, 0x00, 0x02, 0x00, 0x0A, 0x1A };

// 音訊物件
Audio audio;

const int response_length = 23;
unsigned long previousMillis = 0;
const long interval = 10000;
const long blinkInterval = 200;
unsigned long lastBlinkMillis = 0;
bool ledState = LOW;
bool identificationSuccess = false;
bool startIdentificationAudioPlayed = false;

// NETWORK INFO
const char *ssid = "hel";
const char *password = "1234567890";
const char *serverUrl = "http://49jfc.api.go64.cc/";

void setupAudio() {
  audio.setPinout(I2S_BCLK, I2S_LRCK, I2S_DOUT);
  audio.setVolume(21);  // 最大音量 (0-21)
  
  // 額外的調試輸出
  Serial.println("Audio Setup Completed");
}

void playAudio(const char *filename) {
  Serial.print("Attempting to play: ");
  Serial.println(filename);
  
  // 檢查檔案是否存在
  if (SD.exists(filename)) {
    Serial.println("File exists!");
    audio.connecttoFS(SD, filename);
    
    // 等待音訊播放完成
    while(audio.isRunning()) {
      audio.loop();
    }
  } else {
    Serial.print("File not found: ");
    Serial.println(filename);
    
    // 列出所有可用檔案以進行偵錯
    File root = SD.open("/");
    while (true) {
      File entry = root.openNextFile();
      if (!entry) break;
      Serial.print("Available file: ");
      Serial.println(entry.name());
      entry.close();
    }
    root.close();
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // SD 卡初始化
  if (!SD.begin(SD_CS)) {
    Serial.println("SD 卡初始化失敗");
    while(1); // 停止程式執行
  }
  Serial.println("SD 卡初始化成功");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".")
  }
    Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // 列出SD卡根目錄檔案
  File root = SD.open("/");
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    Serial.print("檔案: ");
    Serial.println(entry.name());
    entry.close();
  }
  root.close();

  setupAudio();

  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  delay(1000);
}

void loop() {
  audio.loop(); // 重要：持續處理音訊任務

  unsigned long currentMillis = millis();
  static uint8_t response[response_length];
  static int index = 0;

  // LED 閃爍邏輯
  if (!identificationSuccess) {
    if (currentMillis - lastBlinkMillis >= blinkInterval) {
      lastBlinkMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(YELLOW_LED_PIN, ledState);
    }
  }

  // 定期發送指令
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    identificationSuccess = false;
    digitalWrite(RED_LED_PIN, LOW);

    // 播放開始辨識的語音提示
    if (!startIdentificationAudioPlayed) {
      playAudio("/start.mp3");  // 開始辨識時的語音檔
      startIdentificationAudioPlayed = true;
    }

    Serial.println("準備發送指令...");
    Serial2.write(command, sizeof(command));
    Serial2.flush();

    index = 0;
    memset(response, 0, sizeof(response));
  }

  // 接收回應
  while (Serial2.available() > 0) {
    uint8_t incomingByte = Serial2.read();

    if (index == 0 && incomingByte != 0xEF) continue;
    if (index == 1 && incomingByte != 0xAA) {
      index = 0;
      continue;
    }

    response[index] = incomingByte;
    index++;

    if (index >= response_length) {
      Serial.print("收到資料: ");
      for (int i = 0; i < response_length; i++) {
        Serial.printf("%02X ", response[i]);
      }
      Serial.println();

      // 辨識成功判斷
      bool isSuccessResponse = (
        response[0] == 0xEF && 
        response[1] == 0xAA && 
        response[5] == 0x12 && 
        (response[7] == 0x00 && (
          response[8] == 0x03 || 
          response[8] == 0x04 || 
          response[8] == 0x05 || 
          response[8] == 0x06 || 
          response[8] == 0x09 || 
          response[8] == 0x0B || 
          response[8] == 0x11
        ))
      );

      if (isSuccessResponse) {
        digitalWrite(YELLOW_LED_PIN, LOW);
        digitalWrite(RED_LED_PIN, HIGH);
        Serial.println("辨識成功！紅色 LED 亮起");

        // 嘗試播放辨識成功的音訊檔案
        playAudio("/2.mp3");
        
        sendFileToServer("/2.mp3");
        // 重置開始辨識的語音播放標誌
        startIdentificationAudioPlayed = false;
      }

      index = 0;
      memset(response, 0, sizeof(response));
    }
  }
}

void sendFileToServer(File &file) {
    if (SD.exists(filename)) {
    Serial.println("File exists!");
    audio.connecttoFS(SD, filename);
      RequestOptions options;
  options.method = "POST";
  options.headers["Content-Type"] = "audio/mpeg";

  // Read file content
  size_t fileSize = file.size();
  uint8_t *fileContent = new uint8_t[fileSize];
  file.read(fileContent, fileSize);

  // Convert file content to String
  String fileContentString = "";
  for (size_t i = 0; i < fileSize; i++) {
    fileContentString += (char)fileContent[i];
  }

  // Set request body
  options.body = fileContentString;

  // Send POST request
  Response response = fetch(serverUrl, options);

  // Check response
  if (response.status == 200) {
    Serial.println("File uploaded successfully");
  } else {
    Serial.println("Failed to upload file");
    Serial.println("Response status: " + String(response.status));
    Serial.println("Response body: " + response.text());
  }

  // Free allocated memory
  delete[] fileContent;
  } else {
    Serial.print("File not found: ");
    Serial.println(filename);
    
    // 列出所有可用檔案以進行偵錯
    File root = SD.open("/");
    while (true) {
      File entry = root.openNextFile();
      if (!entry) break;
      Serial.print("Available file: ");
      Serial.println(entry.name());
      entry.close();
    }
    root.close();
}
