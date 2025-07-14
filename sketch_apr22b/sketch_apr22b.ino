/**
 * Sources: 
 * https://randomnerdtutorials.com/esp32-dual-core-arduino-ide/ 
 * https://randomnerdtutorials.com/esp32-esp8266-publish-sensor-readings-to-google-sheets/
 * https://github.com/hpware/ArduinoCode/blob/main/sketch_apr7a/sketch_apr7a.ino
 * https://t3.chat/chat/ba25267b-8f0b-451c-b416-b319eef4cfca
 * https://randomnerdtutorials.com/esp32-http-get-post-arduino/ <- This is for GET requests
*/

// 函示庫
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

// 設定溫濕度、繼電器與紅外線的PIN
#define DHT_SENSOR_PIN 33
#define DHT_SENSOR_TYPE DHT11
#define JIPOWER_PIN 25
#define LED_PIN 32

// 設定紅外線 PIN 與 地址
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define OTI602_ADDR 0x10

// 設定
// WiFi 設定
const char *ssid = "hel";
const char *password = "1234567890";
// API 網址 (必須要是 HTTPS!!!)
// 氣象局伺服器
const char *serverUrl1 = "https://hpg7.sch2.top/weather/v2/";  //　網址應該是 https://<<你的主機>>/weather/
// 主要 Nuxt 網頁與 API 伺服器
const char *serverHost2 = "livenet.sch2.top";                   // 主機
const char *deviceId = "ec24af39-81a6-43fa-a90a-96a98da9cc6f";  // 裝置 ID
// 開啟接收資料 (如果全關 WatchDog 會一直強制 Reset 裝置)
const bool tempHumInfo = true;
const bool enableHub8735 = true;  // 如 HUB8735 未開機，請設定為 false  不然 ESP32 的 Watchdog 會一直強制 Reset 裝置
const bool enableGPS = true;

// 下方資料不要改!!!!
// 資料
String data = "";
String h87data = "";
bool sendData = false;
bool isJiPowerOn = false;
bool isLedPowerOn = false;
// 預設 GPS
String defaultlat = "25.134393";
String defaultlong = "121.469968";
String gpsLat = "";
String gpsLong = "";
String gpsTime = "";
// Set Global temp & humidity
float temp = 0;
float hum = 0;
// Set global cwa data
DynamicJsonDocument cwa_data(512);
String cwa_testing_station = "";
String cwa_wtype = "";
float cwa_temp = 0;
float cwa_hum = 0;
float cwa_temp_high = 0;
float cwa_temp_low = 0;
// Do Stuff
const unsigned long TEMP_INTERVAL = 60000;
unsigned long lastTempCheck = 0;
bool initSystem = false;
const char *serverUrl2 = "";  // 舊版

// TaskHandle
TaskHandle_t MainTask;
TaskHandle_t SendTask;

TinyGPSPlus gps;
//WiFiClient client;
HardwareSerial GPS_Serial(1);  // GPS 連接
HardwareSerial H87_Serial(2);  // 8735 連接
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  H87_Serial.begin(115200, SERIAL_8N1, 26, 27);
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
    MainTaskC,   // Task function that it should be using (also don't use the same name as the task handle name, it won't work at all)
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
  cwa_data.clear();
}

// Keep loop empty. And do not use it to do anything, as it will go wrong.
void loop() {}

// use while(true) or while(1) to loop. (and not crash)
void MainTaskC(void *pvParameters) {
  while (true) {
    // Read DHT sensor
    if (tempHumInfo == true) {
      temp = dht_sensor.readTemperature();
      hum = dht_sensor.readHumidity();
    }
    // Read Hub 8735 serial data
    if (enableHub8735 == true) {
      if (H87_Serial.available()) {
        String data = H87_Serial.readStringUntil('\n');
        if (data.length() > 0 && isPrintable(data[0])) {
          Serial.print("Hub 8735 data: ");
          h87data = data;
          Serial.println(h87data);
        }
      }
    }
    // Read GPS serial data
    if (enableGPS == true) {
      if (GPS_Serial.available()) {
        if (GPS_Serial.read() > 0) {
          if (gps.encode(GPS_Serial.read())) {
            if (gps.location.isValid()) {
              gpsLat = String(gps.location.lat());
              gpsLong = String(gps.location.lng());
            }
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void SendTaskC(void *pvParameters) {
  while (true) {
    unsigned long currentMillis = millis();
    // Send weather request
    if (currentMillis - lastTempCheck >= TEMP_INTERVAL || initSystem == false) {
      getWeatherData();
      lastTempCheck = currentMillis;
    }
    if (initSystem == false) {
      initSystem = true;
    }
    // Send local data
    sssdata();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void sssdata() {
  // 設定預設值
  String cwaType = cwa_data.containsKey("weather") ? cwa_data["weather"].as<String>() : "陰有雨";
  String cwaLocation = cwa_data.containsKey("location") ? cwa_data["location"].as<String>() : "臺北市士林區";
  float cwaTemp = 23.5;
  int cwaHum = 89;
  int cwaDailyHigh = 28;
  int cwaDailyLow = 22;
  int localTemp = temp;
  int localHum = hum;
  String localGpsLat = gpsLat.length() > 0 ? gpsLat : defaultlat;
  String localGpsLong = gpsLong.length() > 0 ? gpsLong : defaultlong;
  String localTime = "2025-07-12 10:15:00";
  bool localJistatus = isJiPowerOn;
  bool localLedStatus = isLedPowerOn;

  if (cwa_data.containsKey("location")) {
    if (!cwa_data["temperature"].isNull() && cwa_data["temperature"] != -99) {
      cwaTemp = cwa_data["temperature"].as<float>();
    }
    if (!cwa_data["humidity"].isNull() && cwa_data["humidity"] != -99) {
      cwaHum = cwa_data["humidity"].as<int>();
    }
    if (!cwa_data["dailyHigh"].isNull() && cwa_data["dailyHigh"] != -99) {
      cwaDailyHigh = cwa_data["dailyHigh"].as<int>();
    }
    if (!cwa_data["daliyLow"].isNull() && cwa_data["daliyLow"] != -99) {
      cwaDailyLow = cwa_data["daliyLow"].as<int>();
    }
  }
  // 開始傳送
  WiFiClientSecure client;
  client.setInsecure();                   // 可以接收尚未被接受的SSL憑證
  unsigned long connectStart = millis();  // 時間
  while (!client.connect(serverHost2, 443)) {
    if (millis() - connectStart > 5000) {  // 5秒
      Serial.println("Connection failed");
      return;
    }
    delay(100);
  }
  StaticJsonDocument<1024> doc;
  doc["cwa_type"] = cwaType;
  doc["cwa_location"] = cwaLocation;
  doc["cwa_temp"] = cwaTemp;
  doc["cwa_hum"] = cwaHum;
  doc["cwa_daliyHigh"] = cwaDailyHigh;
  doc["cwa_daliyLow"] = cwaDailyLow;
  doc["local_temp"] = (int)temp;
  doc["local_hum"] = (int)hum;
  doc["local_gps_lat"] = gpsLat.length() > 0 ? gpsLat : defaultlat;
  doc["local_gps_long"] = gpsLong.length() > 0 ? gpsLong : defaultlong;
  doc["local_time"] = "2024-03-20 15:30:00";
  doc["local_jistatus"] = isJiPowerOn;
  doc["local_detect"] = JsonArray();
  doc["testing"] = h87data;
  String jsonString;
  serializeJson(doc, jsonString);
  client.println("POST /api/device_store/" + String(deviceId) + " HTTP/1.1");
  client.println("Host: " + String(serverHost2));
  client.println("Connection: close");
  client.println("Content-Type: application/json");
  client.print("Content-Length: ");
  client.println(jsonString.length());
  client.println();
  client.print(jsonString);
  unsigned long timeout = millis();
  while (!client.available()) {
    if (millis() - timeout > 5000) {
      Serial.println("Response timeout");
      client.stop();
      return;
    }
    delay(10);
  }

  // Read response
  String response = "";
  while (client.available()) {
    char c = client.read();
    response += c;
  }

  // Parse response
  int bodyStart = response.indexOf("\r\n\r\n") + 4;
  if (bodyStart > 4) {
    String body = response.substring(bodyStart);
    DynamicJsonDocument respDoc(512);
    DeserializationError error = deserializeJson(respDoc, body);

    if (!error) {
      if (respDoc.containsKey("jistatus")) {
        isJiPowerOn = respDoc["jistatus"].as<bool>();
        digitalWrite(JIPOWER_PIN, isJiPowerOn);
      }
      if (respDoc.containsKey("ledstatus")) {
        isLedPowerOn = respDoc["ledstatus"].as<bool>();
        digitalWrite(LED_PIN, isLedPowerOn);
      }
      Serial.println("✅");
    }
  }

  client.stop();
}

// 存取氣象局資料
void getWeatherData() {
  HTTPClient http;
  String latlat = (gpsLat.length() > 0) ? gpsLat : defaultlat;
  String lnglng = (gpsLong.length() > 0) ? gpsLong : defaultlong;
  String serverPath = serverUrl1 + latlat + "/" + lnglng;
  http.begin(serverPath.c_str());
  int httpResponseCode = http.GET();
  if (httpResponseCode > 0) {
    String payload = http.getString();
    Serial.println(payload);
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  http.end();
}