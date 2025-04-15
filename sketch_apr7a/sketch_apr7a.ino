#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "Fetch.h"
#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#define DHT_SENSOR_PIN 21
#define DHT_SENSOR_TYPE DHT11
#define JIPOWER_PIN 22

#define I2C_SDA_PIN 25 
#define I2C_SCL_PIN 33
#define OTI602_ADDR 0x10 

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
const unsigned long JI_POWER_INTERVAL = 30;
const unsigned long FETCH_INTERVAL = 60000;
const unsigned long RESPONSE_DELAY = 1000; 
const unsigned long TEMP_INTERVAL = 2000;
const unsigned long GPS_INTERVAL = 1000000;
const unsigned long WEATHER_INTERVAL = 300000;
const unsigned long POWER_INTERVAL = 30000;
const unsigned long SENDDATA_INTERVAL = 10000;
unsigned long lastTempCheck = 0;
unsigned long lastGPSCheck = 0;
unsigned long lastWeatherCheck = 0;
unsigned long lastPowerCheck = 0;
unsigned long lastSentData = 0;
bool isRequestPending = false;
unsigned long requestStartTime = 0;
const unsigned long REQUEST_TIMEOUT = 10000;
bool initSystem = false;

#define MAX_ARRAY_SIZE 10
String receivedArray[MAX_ARRAY_SIZE];

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
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000); // 100kHz
  Serial.print("I2C Initialized for OTI602 on SDA=");
  Serial.print(I2C_SDA_PIN);
  Serial.print(", SCL=");
  Serial.println(I2C_SCL_PIN);

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
    Serial.print("Loop; InitSystem: ");
    Serial.println(initSystem);
    // Temperature check
    if (currentMillis - lastTempCheck >= TEMP_INTERVAL || initSystem == false) {
        GetTemp();
        lastTempCheck = currentMillis;
    }


    if (currentMillis - lastOTI602Check >= OTI602_INTERVAL || !initSystem) {
      ReadOTI602Temp();
      lastOTI602Check = currentMillis;
    }

    if (currentMillis - lastGPSCheck >= GPS_INTERVAL || initSystem == false || gpsLat.length() == 0 || gpsLong.length() == 0) {
        checkGPS();
        lastGPSCheck = currentMillis;
          Serial.println("GPS");
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
    Serial.println(millis());
    // Use current GPS or defaults if GPS isn't valid yet
    String latToUse = (gpsLat.length() > 0) ? gpsLat : defaultlat;
    String lonToUse = (gpsLong.length() > 0) ? gpsLong : defaultlong;
    sendRequest(lonToUse, latToUse);
}
void processSerial() {
    Serial.println(millis());
        if (H87_Serial.available()) {
            String receivedData = H87_Serial.readStringUntil('\n');
            Serial.print("來自 HUB8735: ");
            Serial.println(receivedData);
                    
        // Split the received data into array
        int arrayIndex = 0;
        int prevIndex = 0;
        int commaIndex = receivedData.indexOf(',');
        
        // Clear previous array contents
        for(int i = 0; i < MAX_ARRAY_SIZE; i++) {
            receivedArray[i] = "";
        }
        
        // Split string by comma
        while(commaIndex >= 0 && arrayIndex < MAX_ARRAY_SIZE) {
            receivedArray[arrayIndex] = receivedData.substring(prevIndex, commaIndex);
            prevIndex = commaIndex + 1;
            commaIndex = receivedData.indexOf(',', prevIndex);
            arrayIndex++;
        }
        // Add the last part
        if(arrayIndex < MAX_ARRAY_SIZE) {
            receivedArray[arrayIndex] = receivedData.substring(prevIndex);
        }
        
        checkStuff(receivedArray);
    }
}

void checkStuff(String received[]) {
  Serial.println(received[0]);
  Serial.println(received[1]);
  Serial.println(received[2]);

}

void checkGPS() {
    if (GPS_Serial.available()) {
      Serial.println("GPS Serial is available");
      // WELP THIS IS A STUPID FUCKING FIX, IT IS JUST IN FRONT OF ME THIS ENTIRE TIME!!!!!1!!!
         if (GPS_Serial.available() > 0) {
              Serial.println("GPS Serial is available x2");
            if (gps.encode(GPS_Serial.read())) {
                  Serial.println("Reading GPS Serial");
                getSignal();
            }
        }
    } else {
      Serial.println("GPS Serial is not available");
    }
}

// Reads OTI602 Temperatures via I2C
bool readOTI602Temperatures(float *ambientTemp, float *objectTemp) {
  byte i2cData[6];

  // Send read command (0x80)
  Wire.beginTransmission(OTI602_ADDR);
  Wire.write(0x80);
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.print("OTI602: Error sending read command: ");
    Serial.println(error);
    return false;
  }

  // Request 6 bytes of data
  byte bytesReceived = Wire.requestFrom(OTI602_ADDR, 6);

  if (bytesReceived != 6) {
    Serial.print("OTI602: Error receiving data. Expected 6, got: ");
    Serial.println(bytesReceived);
    // Clear buffer if partial data received
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  // Read the 6 bytes
  for (int i = 0; i < 6; i++) {
    if (Wire.available()) {
      i2cData[i] = Wire.read();
    } else {
      Serial.println("OTI602: Error reading byte during data reception.");
      return false;
    }
  }

  // Calculate Ambient Temperature (first 3 bytes, LSB first)
  int32_t rawAmbient = i2cData[0] | (i2cData[1] << 8) | (i2cData[2] << 16);
  // Sign extend if negative (check MSB of the 24 bits)
  if (i2cData[2] & 0x80) {
    rawAmbient |= 0xFF000000;
  }
  *ambientTemp = rawAmbient / 200.0f;

  // Calculate Object Temperature (last 3 bytes, LSB first)
  int32_t rawObject = i2cData[3] | (i2cData[4] << 8) | (i2cData[5] << 16);
  // Sign extend if negative
  if (i2cData[5] & 0x80) {
    rawObject |= 0xFF000000;
  }
  *objectTemp = rawObject / 200.0f;

  return true;
}

// Wrapper function to read and print OTI602 data
void ReadOTI602Temp() {
  if (readOTI602Temperatures(&oti602AmbientTemp, &oti602ObjectTemp)) {
    Serial.print("OTI602 Sensor -> Ambient: ");
    Serial.print(oti602AmbientTemp, 2);
    Serial.print(" *C, Object: ");
    Serial.print(oti602ObjectTemp, 2);
    Serial.println(" *C");
  } else {
    Serial.println("Failed to read from OTI602 sensor!");
    // Optionally set temps to NaN or a specific error value
    oti602AmbientTemp = NAN;
    oti602ObjectTemp = NAN;
  }
}


void getSignal() {
    Serial.println(gps.location.isValid());
    Serial.println(gps.location.lng());
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
        gpsLat = String(gps.location.lat());
        gpsLong = String(gps.location.lng());
        Serial.println(gpsLat + "  " + gpsLong);
        gpsTime = String(gps.time.hour()) + ":" + 
                  String(gps.time.minute()) + ":" + 
                  String(gps.time.second());
        
        Serial.println("Weather");
        sendRequest(gpsLong, gpsLat);
    } else {
        Serial.println("Waiting for valid GPS data...");
        Serial.println("Weather");
        sendRequest(gpsLong, gpsLat);
    }
}

void sendRequest(String lng, String lat) {
  RequestOptions options;
  options.method = "GET";
  // No need for default checks here if handled in startWeatherRequest
  
  if (lat.length() > 0) {} else {
    Serial.println("No lat");
    lat = defaultlat;
  }
  if (lng.length() > 0) {} else {
    Serial.println("No long");
    lng = defaultlong;
  }
  
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
      return; 
    }

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

    data = response.text();
    isRequestPending = false;
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
    if (data.length() == 0) {
        Serial.println("Fetching new weather data...");
        sendRequest(defaultlong, defaultlat);
        delay(RESPONSE_DELAY);
        lastFetchTime = millis();
    }

    // Parse weather data
    DynamicJsonDocument weatherData(2048);
    DeserializationError error = deserializeJson(weatherData, data);

    DynamicJsonDocument doc(2048);
        doc["cwa_type"] = weatherData["weather"] | "晴";
        doc["cwa_location"] = weatherData["location"] | "臺北市士林區";
        doc["cwa_temp"] = weatherData["temprature"] | 27.7;
        doc["cwa_hum"] = weatherData["humidity"]| 60;
        doc["cwa_daliyHigh"] = weatherData["daliyHigh"] | 26;
        doc["cwa_daliyLow"] = weatherData["daliyLow"] | 15;
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