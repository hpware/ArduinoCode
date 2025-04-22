#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "Fetch.h"
#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>

#define DHT_SENSOR_PIN 33
#define DHT_SENSOR_TYPE DHT11
#define JIPOWER_PIN 25
#define LED_PIN 32

// 設定紅外線 PIN 與 地址
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define OTI602_ADDR 0x10 

// WiFi 設定
const char *ssid = "hel";
const char *password = "1234567890";
// API 網址
const char *serverUrl1 = "https://hpg7.sch2.top/weather/v2/";
const char *serverUrl2 = "https://zb-logger.sch2.top/logger/store";
String data = "";
bool sendData = false;
bool isJiPowerOn = false;
bool initJiPower  = true;
int JipowerTime = 0;
// 預設 GPS
String defaultlat = "25.134393";
String defaultlong = "121.469968";
String gpsLat = "";
String gpsLong = "";
String gpsTime = "";
bool shouldSendData = false;

// TaskHandle
TaskHandle_t MainTask;
TaskHandle_t SendTask;

TinyGPSPlus gps;
HardwareSerial GPS_Serial(1); // GPS 連接
HardwareSerial H87_Serial(2); // 8735 連接
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8IN1, 16, 17);
  H87_Serial.begin(9600, SERIAL_8IN1, 26, 27);
  Serial.println("Setup");
  // setup hum and temp sensor
  dht_sensor.begin();
  // init wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(JIPOWER_PIN, OUTPUT);
  digitalWrite(JIPOWER_PIN, LOW);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  // init task
  xTaskCreatePinnedToCore(MainTask, "MainTask", 10000, NULL, 1, &MainTask, 0);
  delay(500);
  xTaskCreatePinnedToCore(SendTask, "SendTask", 10000, NULL, 1, &SendTask, 0);
  delay(500);
}

void MainTask((void *pvParameters)) {
  while(true) {
    Serial.println("MainTask");
    delay(1000);
  }
}

void SendTask((void *pvParameters)) {
    Serial.println("SendTask");
    delay(1000);
}

// keep empty
void loop() {

}