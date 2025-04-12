#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "Fetch.h"
#include <ArduinoJson.h>
#include <SD.h>
#include <SPI.h>
#include <DHT.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#define DHT_SENSOR_PIN 21
#define DHT_SENSOR_TYPE DHT11
#define JIPOWER_PIN 22

const char *ssid = "a";
const char *password = "pi!=7.188";
const char *serverUrl1 = "https://edu.yhw.tw/weather/v2/";
const char *serverUrl2 = "https://edu.yhw.tw/logger/store";
String data = "";
bool sendData = false;
bool isJiPowerOn = false;
int JipowerTime = 0;
String defaultlat = "25.134393";
String defaultlong = "121.469968";
String gpsLat = "";
String gpsLong = "";
String gpsTime = "";
bool shouldSendData = false;

TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);
HardwareSerial H87_Serial(2);

DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  H87_Serial.begin(115200, SERIAL_8N1, 26, 27); 
  Serial.println("GPS 模組啟動中...");
    WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  dht_sensor.begin();
  pinMode(JIPOWER_PIN, OUTPUT);
  digitalWrite(JIPOWER_PIN, LOW);
  SetJiPower();
  // OTA
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.setPort(3232);
    ArduinoOTA.setHostname("fsdfgf");
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
  delay(3000);
}
void loop() {
    ArduinoOTA.handle();
    GetTemp();
    SetJiPower();
    sendData22();
    if (H87_Serial.available()) {
    String receivedData = H87_Serial.readStringUntil('\n');  // 一次讀一筆直到換行
    Serial.print("來自 HUB8735: ");
    Serial.println(receivedData);
    }
  delay(10);
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
        ArduinoOTA.handle();
    GetTemp();
    SetJiPower();
    sendData22();
    if (H87_Serial.available()) {
    String receivedData = H87_Serial.readStringUntil('\n');  // 一次讀一筆直到換行
    Serial.print("來自 HUB8735: ");
    Serial.println(receivedData);
  }
    if (gps.location.isUpdated()) {
      Serial.print("經度: ");
      Serial.println(gps.location.lng(), 6);
      gpsLong = gps.location.lng(), 6;
      Serial.print("緯度: ");
      Serial.println(gps.location.lat(), 6);
      gpsLat = gps.location.lat(), 6;
      Serial.print("速度: ");
      Serial.println(gps.speed.kmph());
      Serial.print("高度: ");
      Serial.println(gps.altitude.meters());
      Serial.print("時間: ");
      Serial.print(gps.time.hour());
      Serial.print(":");
      Serial.print(gps.time.minute());
      Serial.print(":");
      Serial.println(gps.time.second());
      gpsTime = String(gps.time.hour()) + ":" + 
         String(gps.time.minute()) + ":" + 
         String(gps.time.second());
      Serial.println(gpsTime);
      // Send request to get weather data based on GPS coordinates
      sendRequest(String(gps.location.lng(), 6), String(gps.location.lat(), 6));
      
      Serial.println("-----------------------------");
    }
  }
}

void sendRequest(String lng, String lat) {
  RequestOptions options;
  options.method = "GET";
  String serverUrl = serverUrl1 + lat + "/" + lng;
  
  Serial.println("Requesting weather data from: " + serverUrl);
  // Make sure we're converting String to const char* here
  const char* urlChar = serverUrl.c_str();
  Response response = fetch(urlChar, options);
  
  // Check response
  if (response.status == 200) {
    Serial.println("Data received successfully");
    
    // Parse the JSON response
    DynamicJsonDocument doc(2048); // doc(YOUR_JSON_SIZE_THAT_CAN_BE_STORED_IN_RAM), I should check how big the json *SHOULD be* and put the correct number into it.
    DeserializationError error = deserializeJson(doc, response.text());
    
    if (error) {
      Serial.print("JSON parsing failed: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Process and display the received data
    Serial.println("Weather data:");
    if (doc.containsKey("temperature")) {
      Serial.print("Temperature: ");
      Serial.println(doc["temperature"].as<float>());
    }
    if (doc.containsKey("humidity")) {
      Serial.print("Humidity: ");
      Serial.println(doc["humidity"].as<float>());
    }
    if (doc.containsKey("weather")) {
      Serial.print("Weather: ");
      Serial.println(doc["weather"].as<String>());
    }
    
    // Store data for later use if needed
    data = response.text();
  } else {
    Serial.println("Failed to get data");
    Serial.println("Response status: " + String(response.status));
    Serial.println("Response body: " + response.text());
  }
}

void GetTemp() {
  float temp = dht_sensor.readTemperature();
  float hum = dht_sensor.readHumidity();
  Serial.print("hum: ");
  Serial.print(hum);
  Serial.print(" Temp: ");
  Serial.println(temp);
}

void SetJiPower() {
  if (isJiPowerOn == false && JipowerTime == 300) {
    digitalWrite(JIPOWER_PIN, HIGH);
    isJiPowerOn = true;
    JipowerTime = 0;
  }
  if (isJiPowerOn == true && JipowerTime == 300) {
      digitalWrite(JIPOWER_PIN, LOW);
      JipowerTime = 0;
      isJiPowerOn = false;
  }
    JipowerTime += 1;
}
void sendData22() {
    Serial.println("Raw data: " + data);
    static unsigned long lastFetchTime = 0;
    const unsigned long FETCH_INTERVAL = 60000; // 1 minute

    if (data.length() == 0 || millis() - lastFetchTime >= FETCH_INTERVAL) {
        sendRequest(defaultlong, defaultlat);
        delay(1000); // Wait for response
        lastFetchTime = millis();
        return;
    }

    DynamicJsonDocument weatherData(2048);
    DeserializationError error = deserializeJson(weatherData, data);

    if (error) {
        Serial.println("Weather data parse failed, retrying fetch...");
        sendRequest(defaultlong, defaultlat);
        delay(1000);
        return;
    }

    // Rest of the existing code remains the same
    DynamicJsonDocument doc(2048);
    
        if (weatherData.containsKey("cwbData")) {
        doc["cwa_type"] = weatherData["cwbData"]["weather"] | "雲";
        doc["cwa_location"] = "新北市";
        doc["cwa_temp"] = weatherData["cwbData"]["temperature"] | 25.5;
        doc["cwa_hum"] = weatherData["cwbData"]["humidity"] | 60;
        doc["cwa_daliyHigh"] = weatherData["cwbData"]["dailyHigh"] | 26;
        doc["cwa_daliyLow"] = weatherData["cwbData"]["dailyLow"] | 15;
    } else {
        Serial.println("No weather data available, retrying fetch...");
        sendRequest(defaultlong, defaultlat);
        delay(1000);
    }

    doc["local_temp"] = dht_sensor.readTemperature();
    doc["local_hum"] = dht_sensor.readHumidity();
    doc["local_gps_lat"] = gpsLat.length() > 0 ? gpsLat : defaultlat;
    doc["local_gps_long"] = gpsLong.length() > 0 ? gpsLong : defaultlong;
    doc["local_time"] = gpsTime.length() > 0 ? gpsTime : "2024-03-20 15:30:00";
    doc["local_jistatus"] = isJiPowerOn;

    JsonArray detect = doc.createNestedArray("local_detect");
    detect.add("person");
    detect.add("car");

    String jsonString;
    serializeJson(doc, jsonString);

    RequestOptions options;
    options.method = "POST";
    options.headers["Content-Type"] = "application/json";
    options.headers["Content-Length"] = String(jsonString.length());
    options.body = jsonString;

    Response response = fetch(serverUrl2, options);
    Serial.println("Sent: " + jsonString);
}