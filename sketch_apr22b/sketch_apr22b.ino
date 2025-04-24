#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "Fetch.h"
#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

/**
 * Sources: 
 * https://randomnerdtutorials.com/esp32-dual-core-arduino-ide/ 
 * https://randomnerdtutorials.com/esp32-esp8266-publish-sensor-readings-to-google-sheets/
 * https://github.com/hpware/ArduinoCode/blob/main/sketch_apr7a/sketch_apr7a.ino
 * https://t3.chat/chat/ba25267b-8f0b-451c-b416-b319eef4cfca
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
const char *serverUrl1 = "https://edu.yhw.tw/weather/v2/";
const char *serverHost1 = "edu.yhw.tw";
const char *serverUrl2 = "https://zb-logger.zeabur.app/logger/store";
const char *serverHost2 = "zb-logger.zeabur.app";
String data = "";
String h87data = "";
bool sendData = false;
bool isJiPowerOn = false;
bool initJiPower = true;
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
  cwa_data.clear();
}
// Keep loop empty. And do not use it to do anything, as it will go wrong.
void loop() {}

// use while(true) or while(1) to loop. (and not crash)
void MainTaskC(void *pvParameters) {
  while (true) {
    // Read DHT sensor
    temp = dht_sensor.readTemperature();
    hum = dht_sensor.readHumidity();
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
    Serial.println("Main: ✅");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void SendTaskC(void *pvParameters) {
  while (true) {
    // Send weather request
    ssdata();
    // Send local data
    sssdata();
  }
}

void sssdata() {
    WiFiClientSecure client;
     client.setInsecure();

    int retries = 5;
  while(!client.connect(serverHost2, 443) && (retries-- > 0)) {
    Serial.print(".");
  }
  Serial.println();
  if(!client.connected()) {
    Serial.println("Failed to connect...");
  }
  String cwaType = "多雲";
String cwaLocation = "台北市";
float cwaTemp = 23.5;
int cwaHum = 89;
int cwaDailyHigh = 28;
int cwaDailyLow = 22;
int localTemp = 26;
int localHum = 72;
String localGpsLat = "25.134393";
String localGpsLong = "121.469968";
String localTime = "2024-03-20 15:30:00";
bool localJistatus = true;
String localDetect = "獨角仙";

String jsonObject = String("{\"cwa_type\":\"") + cwaType + "\",\"cwa_location\":\"" +
                      cwaLocation + "\",\"cwa_temp\":" + String(cwaTemp) +
                      ",\"cwa_hum\":" + String(cwaHum) + ",\"cwa_daliyHigh\":" +
                      String(cwaDailyHigh) + ",\"cwa_daliyLow\":" +
                      String(cwaDailyLow) + ",\"local_temp\":" + String(localTemp) +
                      ",\"local_hum\":" + String(localHum) + ",\"local_gps_lat\":\"" +
                      localGpsLat + "\",\"local_gps_long\":\"" + localGpsLong +
                      "\",\"local_time\":\"" + localTime + "\",\"local_jistatus\":" +
                      (localJistatus ? "true" : "false") + ",\"local_detect\":[\"" +
                      localDetect + "\"]}";

  client.println(String("POST ") + "/logger/store" + " HTTP/1.1");
  client.println(String("Host: ") + serverHost2); 
  client.println("Connection: close\r\nContent-Type: application/json");
  client.print("Content-Length: ");
  client.println(jsonObject.length());
  client.println();
  client.println(jsonObject);
        
  int timeout = 5 * 10; // 5 seconds             
  while(!client.available() && (timeout-- > 0)){
    delay(100);
  }
  if(!client.available()) {
    Serial.println("No response...");
  }
  while(client.available()){
    Serial.write(client.read());
  }
  
  Serial.println("\nclosing connection");
  client.stop(); 
}


void ssdata() {
  cwa_data.clear();
  String lat = gpsLat;
  String lng = gpsLong;
  WiFiClientSecure client1;
     client1.setInsecure();

    int retries = 5;
  while(!client1.connect(serverHost1, 443) && (retries-- > 0)) {
    Serial.print(".");
  }
  Serial.println();
  if(!client1.connected()) {
    Serial.println("Failed to connect...");
  }
  String servoUrl21 = serverUrl1 + lat + "/" + lng;

  client1.println(String("GET ") + servoUrl21 + " HTTP/1.1");
  client1.println(String("Host: ") + serverHost1); 
  client1.println("Connection: close\r");
        
  int timeout = 5 * 10; // 5 seconds             
  while(!client1.available() && (timeout-- > 0)){
    delay(100);
  }
  if(!client1.available()) {
    Serial.println("No response...");
  }
  String restext = "";
  while(client1.available()){
    char c = client1.read();
    restext += c;
  }
      Serial.println("✅");
    DeserializationError error = deserializeJson(cwa_data, restext);
    if (error) {
      Serial.println(error.f_str());
    }
  Serial.println("\nclosing connection");
  client1.stop(); 
}

void sendRequest() {
  cwa_data.clear();
  String lat = gpsLat;
  String lng = gpsLong;
  RequestOptions options;
  options.method = "GET";

  if (lat.length() > 0) {
  } else {
    Serial.println("No lat");
    lat = defaultlat;
  }
  if (lng.length() > 0) {
  } else {
    Serial.println("No long");
    lng = defaultlong;
  }
  // 網址包含
  String serverUrl = serverUrl1 + lat + "/" + lng;

  Serial.println("Requesting weather data from: " + serverUrl);
  const char *urlChar = serverUrl.c_str();
  Response response = fetch(urlChar, options);

  // Check response
  if (response.status == 200) {
    Serial.println("✅");
    String restext = response.text();
    DeserializationError error = deserializeJson(cwa_data, restext);
    if (error) {
      Serial.println(error.f_str());
    }
  } else {
    Serial.println("❌");
    Serial.println("Response status: " + String(response.status));
    Serial.println("Response body: " + response.text());
  }
}