/**
 * Sources:
 * https://randomnerdtutorials.com/esp32-dual-core-arduino-ide/
 * https://randomnerdtutorials.com/esp32-esp8266-publish-sensor-readings-to-google-sheets/
 * https://github.com/hpware/ArduinoCode/blob/main/sketch_apr7a/sketch_apr7a.ino
 * https://t3.chat/chat/ba25267b-8f0b-451c-b416-b319eef234dfca
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
#include <SoftwareSerial.h>

// 設定溫濕度、繼電器與紅外線的PIN
#define DHT_SENSOR_PIN 0
#define DHT_SENSOR_TYPE DHT11
#define JIPOWER_PIN 1

// 設定紅外線 PIN 與 地址
#define I2C_SDA_PIN 8  // 21
#define I2C_SCL_PIN 9  // 22
#define OTI602_ADDR 0x10

// 設定
// DEBUG
const bool debug = false;
const bool h87debug = true;
const bool plottingMode = false;
// WiFi 設定
const char *ssid = "hel";
const char *password = "1234567890";
// API 網址 (必須要是 HTTPS!!!)
// 氣象局伺服器
const char *weatherUrl1 = "https://hpg7.sch2.top/weather/v2/";     // Server 1 (不穩定)
const char *weatherUrl2 = "https://c5d5bb.a.srv.yhw.tw/weather/";  // Server 2 (較穩定)
const char *serverUrl1 = weatherUrl2;                              // 網址應該是 https://<<你的主機>>/weather/
// MQTT Server info & topic data. (this is not used btw, but in the future the system will change to a much more MQTT focused system instead of the current api. However the weather info will still be sent via the api.)
const char *mqttHost = "66.179.242.171";                                      // your mqtt server
const uint8_t mqttPort = 1883;                                                // your mqtt server's port
const char *mqttUser = "";                                                    // your mqtt user account, if you don't have one, you can leave it blank.
const char *mqttPassword = "";                                                // your mqtt user password, if you don't have an account, you can leave it blank.
const char *publishTopic = "eco29pass/6e92ff0d-adbe-43d8-b228-e4bc6f948506";  // eco29pass/${裝置 ID}
// 主要 Nuxt 網頁與 API 伺服器
const char *testingApiHost = "livenet.sch2.top";
const char *prodApiHost = "3m4ft6.a.srv.yhw.tw";
const char *serverHost2 = prodApiHost;                          // 主機
const char *deviceId = "6e92ff0d-adbe-43d8-b228-e4bc6f948506";  // 裝置 ID

// 開啟接收資料 (如果全關 WatchDog 會一直強制 Reset 裝置)
const bool tempHumInfo = true;
const bool enableHub8735 = true;  // 如 HUB8735 未開機，請設定為 false  不然 ESP32 的 Watchdog 會一直強制 Reset 裝置
const bool enableGPS = true;
const bool irTempDetect = true;

// Values (DO NOT CHANGE!)
// Boolean data
bool isJiPowerOn = false;
bool itemEntered = false;
bool imageCaptured = false;
bool initSystem = false;
bool pullingHub8735Data = true;
bool autoCapture = false;
bool base64DataInProgress = false;

// Default data
String defaultlat = "25.134393";
String defaultlong = "121.469968";
// Dynamic data.
String gpsLat = "";
String gpsLong = "";
float temp = 0;
float hum = 0;
float oti602AmbientTemp = 0.0;
float oti602ObjectTemp = 0.0;
float prevOti602JbjectTemp = NAN;
int currentFlashLightLevel = 0;     // current flash level
DynamicJsonDocument cwa_data(512);  // set dynamic json document for the cwa_data system which does not require that much space.
String accumulatedData = "";

// Static values
const float tempChangeThreshold = 0.24;
// Intervals
const unsigned long TEMP_INTERVAL = 60000;
const unsigned long captureInterval = 1000 * 2;
// Remember the last interval data.
unsigned long lastCaptureTime = 0;
unsigned long lastTempCheck = 0;
// Base64 Array values
const int MAX_BASE64_ARRAY = 5;  // Keep last 5 images
String base64Array[MAX_BASE64_ARRAY];
int base64ArrayIndex = 0;

// TaskHandle
TaskHandle_t MainTask;
TaskHandle_t SendTask;

// Redefine Serial Connections
SoftwareSerial GPS_Serial(3, 10);  // GPS 連接 (Use software connections)
HardwareSerial H87_Serial(1);      // 8735 連接 (Use the navtive ESP32 Hardware Serial thingy)
// Setup DHT11 sensor and TinyGPS
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
TinyGPSPlus gps;


// Setup
void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600);
  H87_Serial.begin(115200, SERIAL_8N1, 5, 6);
  if (debug) {
    Serial.println("Setup");
  }
  // setup hum and temp sensor
  dht_sensor.begin();
  // init wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  pinMode(JIPOWER_PIN, OUTPUT);
  digitalWrite(JIPOWER_PIN, LOW);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  // init task
  xTaskCreate(
    MainTaskC,   // Task function
    "MainTask",  // Task name
    16384,       // Stack size
    NULL,        // Parameters
    2,           // Priority
    &MainTask    // Task handle
  );

  delay(10);  // This should be fine.

  xTaskCreate(
    SendTaskC,   // Task function
    "SendTask",  // Task name
    16384,       // Stack size
    NULL,        // Parameters
    1,           // Priority
    &SendTask    // Task handle
  );

  cwa_data.clear();
}

// Keep loop empty. And do not use it to do anything, as it will go wrong.
void loop() {}

// use while(true) or while(1) to loop. (and not crash), but you can have some stuff running from the start.
void MainTaskC(void *pvParameters) {
  while (true) {
    if (debug) {
      Serial.print("MainTaskC running on core: ");
      Serial.println(xPortGetCoreID());
    }
    unsigned long currentMillis = millis();
    if (tempHumInfo == true) {
      temp = dht_sensor.readTemperature();
      vTaskDelay(pdMS_TO_TICKS(10));
      hum = dht_sensor.readHumidity();
      vTaskDelay(pdMS_TO_TICKS(10));
      if (debug) {
        Serial.println("Reading DHT sensor...");
        Serial.print("Data: Temp:");
        Serial.print(temp);
        Serial.print(", Hum:");
        Serial.println(hum);
      }
    }
    if (irTempDetect == true) {
      if (debug) {
        Serial.println("Reading OTI602 sensor...");
      }
      if (readOTI602Temperatures(&oti602AmbientTemp, &oti602ObjectTemp)) {
        if (debug) {
          Serial.print("OTI602 Sensor -> Ambient: ");
          Serial.print(oti602AmbientTemp, 2);
          Serial.print(" *C, Object: ");
          Serial.print(oti602ObjectTemp, 2);
          Serial.print(" *C itemEntered: ");
          Serial.println(itemEntered);
        }
        if (!isnan(prevOti602JbjectTemp)) {
          float tempDelta = oti602ObjectTemp - prevOti602JbjectTemp;
          if (tempDelta > tempChangeThreshold) {
            itemEntered = true;
          } else if (tempDelta < -tempChangeThreshold) {
            itemEntered = false;
          }
        }
        prevOti602JbjectTemp = oti602ObjectTemp;
      } else {
        Serial.println("Failed to read from OTI602 sensor!");
        oti602AmbientTemp = NAN;
        oti602ObjectTemp = NAN;
        prevOti602JbjectTemp = NAN;
        itemEntered = false;
      }
    }

    if (enableHub8735 == true) {
      if (itemEntered) {
        if (lastCaptureTime + captureInterval < currentMillis && autoCapture) {
          if (h87debug) Serial.println("Taking pics!");  // debug
          H87_Serial.println("<!CAPTURE /!>");
          lastCaptureTime = currentMillis;  // Update the last capture time
        } else if (!autoCapture && !imageCaptured) {
          if (debug) Serial.println("Auto recapture is disabled.");  // debug
          H87_Serial.println("<!CAPTURE /!>");
          imageCaptured = true;
        }
      }
      if (H87_Serial.available()) {
        pullingHub8735Data = true;
        if (h87debug) {
          Serial.println("H87_Serial data available");
        }
        String chunk = "";
        // compelete the action :)
        while (H87_Serial.available()) {
          //process image capture
          if (lastCaptureTime + captureInterval < currentMillis && autoCapture && itemEntered) {
            if (h87debug) Serial.println("Taking pics!");  // debug
            H87_Serial.println("<!CAPTURE /!>");
            lastCaptureTime = currentMillis;  // Update the last capture time
          }
          // main info
          chunk += H87_Serial.readStringUntil('\n');  // yeah changing the readStringUntil thingy does not work well.
          // this is such an easy fix, i'm dumb
        }
        chunk.trim();
        // check statements
        if (h87debug) {
          Serial.println(chunk);  // so most of the time the chucks just sends everything in one go? nothing more, ouch.
          // Yeah, I love debugging (no)
          Serial.print("Does it start correctly? ");
          Serial.println(chunk.startsWith("<!START BLOCK!>"));

          Serial.print("Does it end correctly? ");
          Serial.println(chunk.endsWith("</!END BLOCK!>"));
          Serial.print("Does it end with !>? ");
          Serial.println(chunk.endsWith("!>"));
          Serial.print("This ends with: ");
          // print the last 10 chars
          for (int i = 20; i >= 0; i--) {
            Serial.print(chunk[chunk.length() - i]);
          }
          Serial.println();
          Serial.print("Will the chunk.startsWith(\"<!START BLOCK!>\") && chunk.endsWith(\"</!END BLOCK!> \") run? ");
          Serial.println(chunk.startsWith("<!START BLOCK!>") && chunk.endsWith("</!END BLOCK!>"));
          Serial.println("------------------");
        }
        if (chunk.startsWith("<!START BLOCK!>") && chunk.endsWith("</!END BLOCK!>")) {
          processFile(chunk);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      pullingHub8735Data = false;
    }
    // Read GPS serial data
    if (enableGPS == true) {
      if (GPS_Serial.available()) {
        while (GPS_Serial.available()) {
          if (gps.encode(GPS_Serial.read())) {
            if (gps.location.isValid()) {
              gpsLat = String(gps.location.lat(), 6);
              gpsLong = String(gps.location.lng(), 6);
              if (debug) {
                Serial.print("GPS Lat: ");
                Serial.println(gpsLat);
                Serial.print("GPS Lng: ");
                Serial.println(gpsLong);
              }
            }
          }
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (plottingMode) {
      Serial.print("Ambient:");
      Serial.print(oti602AmbientTemp, 2);
      Serial.print(",");
      Serial.print("Object:");
      Serial.print(oti602ObjectTemp, 2);
      Serial.print(",");
      Serial.print("itemEntered:");
      Serial.print(itemEntered);
      Serial.print(",");
      Serial.print("Temp:");
      Serial.print(temp);
      Serial.print(",");
      Serial.print("Hum:");
      Serial.print(hum);
      Serial.println("");
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void SendTaskC(void *pvParameters) {
  while (true) {
    if (debug) {
      Serial.print("SendTaskC running on core: ");
      Serial.println(xPortGetCoreID());
    }
    unsigned long currentMillis = millis();
    // Send weather request
    if (currentMillis - lastTempCheck >= TEMP_INTERVAL || initSystem == false) {
      getWeatherData();
      vTaskDelay(pdMS_TO_TICKS(100));
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
  float localTempFloat = temp;
  float localHumFloat = hum;
  String localGpsLat = gpsLat.length() > 0 ? gpsLat : defaultlat;
  String localGpsLong = gpsLong.length() > 0 ? gpsLong : defaultlong;
  bool localJistatus = isJiPowerOn;

  if (cwa_data.containsKey("location")) {  // Check for a valid weather data presence
    if (cwa_data["temperature"].is<float>() && cwa_data["temperature"] != -99) {
      cwaTemp = cwa_data["temperature"].as<float>();
    }
    if (cwa_data["humidity"].is<int>() && cwa_data["humidity"] != -99) {  // Ensure it's an int and not -99
      cwaHum = cwa_data["humidity"].as<int>();
    }
    if (cwa_data["dailyHigh"].is<int>() && cwa_data["dailyHigh"] != -99) {
      cwaDailyHigh = cwa_data["dailyHigh"].as<int>();
    }
    if (cwa_data["dailyLow"].is<int>() && cwa_data["dailyLow"] != -99) {  // Corrected key name from "daliyLow"
      cwaDailyLow = cwa_data["dailyLow"].as<int>();
    }
  }
  // 開始傳送
  WiFiClientSecure client;
  client.setInsecure();                   // Can receive unsigned SSL certificates
  unsigned long connectStart = millis();  // Time
  while (!client.connect(serverHost2, 443)) {
    if (millis() - connectStart > 5000) {  // 5 seconds timeout
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  StaticJsonDocument<8192> doc;
  doc["cwa_type"] = cwaType;
  doc["cwa_location"] = cwaLocation;
  doc["cwa_temp"] = cwaTemp;
  doc["cwa_hum"] = cwaHum;
  doc["cwa_daliyHigh"] = cwaDailyHigh;
  doc["cwa_daliyLow"] = cwaDailyLow;
  doc["local_temp"] = (int)localTempFloat;
  doc["local_hum"] = (int)localHumFloat;
  doc["local_gps_lat"] = localGpsLat;
  doc["local_gps_long"] = localGpsLong;
  doc["local_time"] = "2024-03-20 15:30:00";
  doc["local_jistatus"] = isJiPowerOn;
  vTaskDelay(pdMS_TO_TICKS(10));

  // Create array of base64 data - FIXED VERSION
  bool hasImagesToSend = false;
  for (int i = 0; i < MAX_BASE64_ARRAY; i++) {
    if (base64Array[i].length() > 0) {
      hasImagesToSend = true;
      break;
    }
  }

  JsonArray imageArray = doc.createNestedArray("image");
  if (hasImagesToSend) {
    for (int i = 0; i < MAX_BASE64_ARRAY; i++) {
      if (base64Array[i].length() > 0) {
        imageArray.add(base64Array[i]);
        Serial.print("Adding image to JSON, length: ");
        Serial.println(base64Array[i].length());
        // DON'T clear here - wait until after successful send
      }
    }
  }

  String jsonString;
  size_t json_size = measureJson(doc);
  if (json_size > 8192) {
    Serial.print("WARNING: JSON document too large! Size: ");
    Serial.println(json_size);
  }

  serializeJson(doc, jsonString);

  // Send the data
  client.println("POST /api/device_store/" + String(deviceId) + " HTTP/1.1");
  client.println("Host: " + String(serverHost2));
  client.println("Connection: close");
  client.println("Content-Type: application/json");
  client.print("Content-Length: ");
  client.println(jsonString.length());
  client.println();
  client.print(jsonString);

  client.flush();

  unsigned long timeout = millis();
  while (!client.available()) {
    if (millis() - timeout > 5000) {
      Serial.println("Response timeout");
      client.stop();
      return;  // Don't clear array on timeout
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Changed from delay to vTaskDelay
  }

  // Read response
  String response = "";
  while (client.available()) {
    char c = client.read();
    response += c;
  }

  // Parse response and handle server commands
  int bodyStart = response.indexOf("\r\n\r\n") + 4;
  if (bodyStart > 4) {
    String body = response.substring(bodyStart);
    DynamicJsonDocument respDoc(512);
    DeserializationError error = deserializeJson(respDoc, body);

    if (!error) {
      if (respDoc.containsKey("jistatus")) {
        isJiPowerOn = respDoc["jistatus"].as<bool>();
        //Serial.println(isJiPowerOn);
        digitalWrite(JIPOWER_PIN, isJiPowerOn);
      }
      if (respDoc.containsKey("newledstatus")) {
        int ledPowerOnPoint = respDoc["newledstatus"].as<int>();
        if (ledPowerOnPoint != currentFlashLightLevel) {
          H87_Serial.print("<!FLASHLIGHT!>");
          H87_Serial.print(ledPowerOnPoint);
          H87_Serial.println("</!FLASHLIGHT!>");
          currentFlashLightLevel = ledPowerOnPoint;
        }
      }
      if (respDoc.containsKey("autocapture")) {
        autoCapture = respDoc["autocapture"].as<bool>();
      }

      // Only clear images after successful response
      if (hasImagesToSend) {
        for (int i = 0; i < MAX_BASE64_ARRAY; i++) {
          base64Array[i] = "";
        }
        base64ArrayIndex = 0;
        Serial.println("Base64 array cleared after successful send.");
      }
    } else {
      Serial.print("Error parsing server response JSON: ");
      Serial.println(error.f_str());
    }
  } else {
    Serial.println("Invalid HTTP response format (no body detected).");
    Serial.println(response);
  }

  vTaskDelay(pdMS_TO_TICKS(10));
  client.stop();

  if (debug) {
    Serial.println("✅✅✅✅✅");
  }
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
    DynamicJsonDocument fetchedCwaData(512);
    DeserializationError error = deserializeJson(fetchedCwaData, payload);
    if (!error) {
      cwa_data.clear();
      cwa_data.set(fetchedCwaData);
    } else {
      Serial.print("Error parsing CWA HTTP GET JSON: ");
      Serial.println(error.f_str());
    }

  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  http.end();
}

bool readOTI602Temperatures(float *ambientTemp, float *objectTemp) {
  byte data[6];
  Wire.beginTransmission(OTI602_ADDR);
  Wire.write(0x80);
  byte error = Wire.endTransmission();

  if (error != 0) {
    if (debug) {
      Serial.print("OTI602 Error!!: ");
      Serial.println(error);
    }
    return false;
  }

  byte bytesReceived = Wire.requestFrom(OTI602_ADDR, 6);

  if (debug) {
    Serial.print("Data Received from OTI602: ");
    Serial.println(bytesReceived);
  }

  if (bytesReceived != 6) {
    return false;
  }

  for (int i = 0; i < 6; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
      if (debug) {
        Serial.print("數據[");
        Serial.print(i);
        Serial.print("]: 0x");
        Serial.println(data[i], HEX);
      }
    } else {
      if (debug) {
        Serial.println("讀取數據時發生錯誤");
      }
      return false;
    }
  }

  int32_t rawAmbient = data[0] + (data[1] << 8) + (data[2] << 16);
  if (data[2] >= 0x80) {
    rawAmbient -= 0x1000000;
  }
  *ambientTemp = rawAmbient / 200.0f;

  int32_t rawObject = data[3] + (data[4] << 8) + (data[5] << 16);
  if (data[5] >= 0x80) {
    rawObject -= 0x1000000;
  }
  *objectTemp = rawObject / 200.0f;

  return true;
}

void processFile(const String &chunkText) {
  int startIndex = accumulatedData.indexOf("<!START BLOCK!>");
  int endIndex = accumulatedData.indexOf("</!END BLOCK!>");
  if (startIndex != -1 && endIndex != -1 && endIndex > startIndex) {
    // Found both markers and END is after START
    String completeBase64Image = accumulatedData.substring(startIndex + 16, endIndex);

    // Validate the extracted string (optional but recommended)
    base64ArrayIndex = (base64ArrayIndex + 1) % MAX_BASE64_ARRAY;
    base64Array[base64ArrayIndex] = completeBase64Image;
    imageCaptured = true;  // Mark that an image is ready for sending

    if (debug) {
      Serial.printf("📷 Stored Base64 image in slot %d, length %d\n", base64ArrayIndex, completeBase64Image.length());
    }
  }
}