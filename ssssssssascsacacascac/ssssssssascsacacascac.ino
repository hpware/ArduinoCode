#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>


/**
 * Sources: 
 * https://randomnerdtutorials.com/esp32-dual-core-arduino-ide/ 
 * https://randomnerdtutorials.com/esp32-esp8266-publish-sensor-readings-to-google-sheets/
 * https://github.com/hpware/ArduinoCode/blob/main/sketch_apr7a/sketch_apr7a.ino
 * https://t3.chat/chat/ba25267b-8f0b-451c-b416-b319eef4cfca
 * https://randomnerdtutorials.com/esp32-http-get-post-arduino/ <- This is for GET requests
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
const char *serverHost1 = "hpg7.sch2.top";
const char *serverHost2 = "logger-v2.vercel
.app";
const char *deviceId = "b9186021-40da-40f4-83c0-af06396cccb7";
String ImageId = "";
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
TinyGPSPlus gps;
//WiFiClient client;
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
  delay(500);
  cwa_data.clear();
}

// Keep loop empty. And do not use it to do anything, as it will go wrong.
void loop() {
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
}


void sssdata() {
  getId();
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
  String localTime = "2024-03-20 15:30:00";
  bool localJistatus = isJiPowerOn;
  bool localLedStatus = isLedPowerOn;
  String localDetect = "獨角仙";

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
  WiFiClientSecure client;
  client.setInsecure();

  // Try to connect with timeout
  unsigned long connectStart = millis();
  while (!client.connect(serverHost2, 443)) {
    if (millis() - connectStart > 5000) {  // 5 second timeout
      Serial.println("Connection failed");
      return;
    }
    delay(100);
  }

  // Create the JSON object using ArduinoJson instead of string concatenation
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
  JsonArray imageArray = doc.createNestedArray("image");
  imageArray.add(ImageId);

  String jsonString;
  serializeJson(doc, jsonString);

  // Send HTTP request
  client.println("POST /api/device_store/" + String(deviceId) + " HTTP/1.1");
  client.println("Host: " + String(serverHost2));
  client.println("Connection: close");
  client.println("Content-Type: application/json");
  client.print("Content-Length: ");
  client.println(jsonString.length());
  client.println();
  client.print(jsonString);

  // Wait for response with proper timeout
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
      Serial.println("✅ Response processed successfully");
    }
  }

  client.stop();
}

void getId() {
  HTTPClient http;
  String serverPath = "https://hpg7.sch2.top/imgIdDl.txt";
  http.begin(serverPath.c_str());
  int httpResponseCode = http.GET();
  if (httpResponseCode > 0) {
    String payload = http.getString();
    Serial.println(payload);
    ImageId = payload;
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  http.end();
}

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