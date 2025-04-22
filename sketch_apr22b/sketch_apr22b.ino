#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "Fetch.h"
#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>

/**
 * Sources: 
 * https://randomnerdtutorials.com/esp32-dual-core-arduino-ide/ 
 * https://randomnerdtutorials.com/esp32-esp8266-publish-sensor-readings-to-google-sheets/
 * https://github.com/hpware/ArduinoCode/blob/main/sketch_apr7a/sketch_apr7a.ino

*/

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
String h87data = "";
bool sendData = false;
bool isJiPowerOn = false;
bool initJiPower = true;
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
HardwareSerial GPS_Serial(1);  // GPS 連接
HardwareSerial H87_Serial(2);  // 8735 連接
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  H87_Serial.begin(9600, SERIAL_8N1, 26, 27);
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
  xTaskCreatePinnedToCore(
    MainTaskC,   // Task func that it should be using (also don't use the same name as the task handle name, it won't work at all)
    "MainTask",  // Task name
    10000,       // Stack size
    NULL,        // param of the task
    1,           // priority of the task
    &MainTask,   // task hnalde to keep track
    0            // use core 0
  );
  delay(500);
  xTaskCreatePinnedToCore(
    SendTaskC,
    "SendTask",
    10000,
    NULL,
    1,
    &SendTask,
    1);
  delay(500);
}
// Keep loop empty. And do not use it to do anything, as it will go wrong.
void loop() {}

// use while(true) or while(1) to loop. (and not crash)
void MainTaskC(void *pvParameters) {
  while (true) {
    // Read Hub 8735 serial data
    if (H87_Serial.available()) {
      String data = H87_Serial.readStringUntil('\n');
      Serial.print("Hub 8735 data: ");
      h87data = data;
      Serial.println(h87data);
    }
    // Read GPS serial data
    if (GPS_Serial.available()) {
      if (GPS_Serial.read() > 0) {
        if (gps.encode(GPS_Serial.read())) {
          if (gps.location.isValid()) {
            gpsLat = String(gps.location.lat());
            gpsLong = String(gps.location.lng());
          } else {
            Serial.println("GPS location is not valid");
          }
        }
      }
    }
  }
}

void SendTaskC(void *pvParameters) {
  while (true) {
    Serial.println("SendTask");
    delay(1000);
  }
}