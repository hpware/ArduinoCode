#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "recipes/WiFi.h"
#include "Fetch.h"
#include <ArduinoJson.h>
#include <SD.h>
#include <SPI.h>
#include <DHT.h>
#include <ArduinoOTA.h>
#define DHT_SENSOR_PIN 21
#define DHT_SENSOR_TYPE DHT11
#define JIPOWER_PIN 22

const char *ssid = "hel";
const char *password = "1234567890";
const char *serverUrl1 = "https://edu.yhw.tw/weather/v2/";
String data = "";
bool sendData = false;

TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);

DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
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
}
void loop() {
  ArduinoOTA.handle();
  digitalWrite(JIPOWER_PIN, HIGH);
  GetTemp();
  delay(500);
  digitalWrite(JIPOWER_PIN, LOW);
    delay(500);
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
    GetTemp();
    if (gps.location.isUpdated()) {
      Serial.print("經度: ");
      Serial.println(gps.location.lng(), 6);
      Serial.print("緯度: ");
      Serial.println(gps.location.lat(), 6);
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
  float hum = dht_sensor.readHumidity();
  float temp = dht_sensor.readTemperature();
  Serial.print("hum: ");
  Serial.print(hum);
  Serial.print(" Temp: ");
  Serial.println(temp);
}