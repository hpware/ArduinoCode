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
bool initJiPower  = true;
int JipowerTime = 0;
String defaultlat = "25.134393";
String defaultlong = "121.469968";
String gpsLat = "";
String gpsLong = "";
String gpsTime = "";
bool shouldSendData = false;
const unsigned long JI_POWER_INTERVAL = 30;  // Power cycle interval
const unsigned long FETCH_INTERVAL = 60000;  // Weather data fetch interval (1 minute)
const unsigned long RESPONSE_DELAY = 1000;   // Delay waiting for response
// Task intervals
const unsigned long TEMP_INTERVAL = 2000;      // 2 seconds
const unsigned long GPS_INTERVAL = 3600000;     // 1 hour
const unsigned long WEATHER_INTERVAL = 300000;  // 5 minutes
const unsigned long POWER_INTERVAL = 30000;     // 30 seconds
const unsigned long SENDDATA_INTERVAL = 10000; // 10 seconds

// Task timers
unsigned long lastTempCheck = 0;
unsigned long lastGPSCheck = 0;
unsigned long lastWeatherCheck = 0;
unsigned long lastPowerCheck = 0;
unsigned long lastSentData = 0;

// State flags
bool isRequestPending = false;
unsigned long requestStartTime = 0;
const unsigned long REQUEST_TIMEOUT = 10000; // 10 second timeout
bool initSystem = false;


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
    unsigned long currentMillis = millis();
    // Temperature check
    if (currentMillis - lastTempCheck >= TEMP_INTERVAL || initSystem == false) {
        GetTemp();
        lastTempCheck = currentMillis;
    }

    if (currentMillis - lastGPSCheck >= GPS_INTERVAL || initSystem == false) {
        checkGPS();
        lastGPSCheck = currentMillis;
    }

    // Weather check
    if ((isRequestPending && currentMillis - requestStartTime >= REQUEST_TIMEOUT) || initSystem == false) {
        if (isRequestPending) { // Only print timeout if it actually was pending
           isRequestPending = false;
           Serial.println("Weather request timeout");
        }
    }
    if (currentMillis - lastWeatherCheck >= WEATHER_INTERVAL || initSystem == false) {
         if (!isRequestPending) {
             startWeatherRequest(); 
             lastWeatherCheck = currentMillis; 
         }
    }


    // Power management
    if (currentMillis - lastPowerCheck >= POWER_INTERVAL || initSystem == false) {
        SetJiPower();
        lastPowerCheck = currentMillis;
    }

    if (currentMillis - lastSentData >= SENDDATA_INTERVAL || initSystem == false) {
        sendData22();
        lastSentData = currentMillis;
    }
    processSerial();
    ArduinoOTA.handle(); 

    if (initSystem == false) {
        initSystem = true; // Set initSystem to true after the first pass
        Serial.println("Initial system checks complete.");
    }
}

void startWeatherRequest() {
    // Removed the check from here, it's handled before calling now
    isRequestPending = true;
    requestStartTime = millis();
    // Use current GPS or defaults if GPS isn't valid yet
    String latToUse = (gpsLat.length() > 0) ? gpsLat : defaultlat;
    String lonToUse = (gpsLong.length() > 0) ? gpsLong : defaultlong;
    sendRequest(lonToUse, latToUse);
}
void processSerial() {
        if (H87_Serial.available()) {
            String receivedData = H87_Serial.readStringUntil('\n');
            Serial.print("來自 HUB8735: ");
            Serial.println(receivedData);
    }
}

void checkGPS() {
    if (GPS_Serial.available()) {
        while (GPS_Serial.available() > 0) {
            if (gps.encode(GPS_Serial.read())) {
                getSignal();
            }
        }
    }
}

void getSignal() {
    if (gps.location.isValid()) {
        Serial.println("GPS Data Updated:");
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
        
        // Update global GPS variables
        gpsLat = String(gps.location.lat(), 6);
        gpsLong = String(gps.location.lng(), 6);
        gpsTime = String(gps.time.hour()) + ":" + 
                  String(gps.time.minute()) + ":" + 
                  String(gps.time.second());
                  
        // Get weather data with new coordinates
        sendRequest(gpsLong, gpsLat);
    } else {
        Serial.println("Waiting for valid GPS data...");
    }
}

void sendRequest(String lng, String lat) {
  RequestOptions options;
  options.method = "GET";
  // No need for default checks here if handled in startWeatherRequest
  String serverUrl = serverUrl1 + lat + "/" + lng;

  Serial.println("Requesting weather data from: " + serverUrl);
  const char* urlChar = serverUrl.c_str();
  Response response = fetch(urlChar, options);

  // Check response
  if (response.status == 200) {
    Serial.println("Data received successfully");

    // Parse the JSON response
    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, response.text());

    if (error) {
      Serial.print("JSON parsing failed: ");
      Serial.println(error.c_str());
      // Don't reset isRequestPending here on parse failure, maybe retry logic needed?
      return; // Exit if parsing fails
    }

    // Process and display the received data
    Serial.println("Weather data:");
    // Use safer access methods
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
    data = response.text(); // Store the raw JSON string
    isRequestPending = false; // Request successful and parsed (or attempted parse)
  } else {
    Serial.println("Failed to get data");
    Serial.println("Response status: " + String(response.status));
    Serial.println("Response body: " + response.text());
    isRequestPending = false; // Request failed
    data = ""; // Clear old data on failure
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
  if (isJiPowerOn == false && JipowerTime == 30 || initJiPower == false) {
    digitalWrite(JIPOWER_PIN, HIGH);
    isJiPowerOn = true;
    JipowerTime = 0;
    initJiPower = true;
  }
  if (isJiPowerOn == true && JipowerTime == 30) {
      digitalWrite(JIPOWER_PIN, LOW);
      JipowerTime = 0;
      isJiPowerOn = false;
  }
    JipowerTime += 1;
    Serial.println(JipowerTime);
}

void sendData22() {
      Serial.println("Starting sendData22...");

    // Debug current values
    Serial.println("Current GPS Data:");
    Serial.println("Lat: " + gpsLat);
    Serial.println("Long: " + gpsLong);
    Serial.println("Time: " + gpsTime);
    Serial.println("Raw data: " + data);
      static unsigned long lastFetchTime = 0;
    
    Serial.println("Current weather data: " + data);

    // Check if we need to fetch new data
    if (data.length() == 0 || millis() - lastFetchTime >= FETCH_INTERVAL) {
        Serial.println("Fetching new weather data...");
        sendRequest(defaultlong, defaultlat);
        delay(RESPONSE_DELAY);
        lastFetchTime = millis();
        return;
    }

    // Parse weather data
    DynamicJsonDocument weatherData(2048);
    DeserializationError error = deserializeJson(weatherData, data);

    if (error) {
        Serial.println("Error parsing weather data: " + String(error.c_str()));
        Serial.println("Retrying data fetch...");
        sendRequest(defaultlong, defaultlat);
        delay(RESPONSE_DELAY);
        return;
    }
    // Rest of the existing code remains the same
    DynamicJsonDocument doc(2048);
        doc["cwa_type"] = "雲";
        doc["cwa_location"] = " 新北市";
        doc["cwa_temp"] = 25.5;
        doc["cwa_hum"] = 60;
        doc["cwa_daliyHigh"] = 26;
        doc["cwa_daliyLow"] = 15;
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

    if (response.status == 200) {
      Serial.println("Done :)");
    } else {
      Serial.println("Failed to send data");
      Serial.println("Response status: " + String(response.status));
      Serial.println("Response body: " + response.text());
    }
}