#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "Fetch.h"
#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>

#define DHT_SENSOR_PIN 21
#define DHT_SENSOR_TYPE DHT11
#define JIPOWER_PIN 22

// 設定紅外線 PIN 與 地址
#define I2C_SDA_PIN 25 
#define I2C_SCL_PIN 33
#define OTI602_ADDR 0x10 

// WiFi 設定
const char *ssid = "hel";
const char *password = "1234567890";
// API 網址
const char *serverUrl1 = "https://hpg7.sch2.top/weather/v2/";
const char *serverUrl2 = "https://hpg7.sch2.top/logger/store";
String data = "";
bool sendData = false;
bool isJiPowerOn = false;
bool initJiPower  = true;
int JipowerTime = 0;
// 預設 GPS
String defaultlat = "25.134393";
String defaultlong = "121.469968";
// GPS
String gpsLat = "";
String gpsLong = "";
String gpsTime = "";
bool shouldSendData = false;

const unsigned long FETCH_INTERVAL = 60000;
const unsigned long TEMP_INTERVAL = 2000;
const unsigned long GPS_INTERVAL = 1000000;
const unsigned long WEATHER_INTERVAL = 30000;
const unsigned long SENDDATA_INTERVAL = 10000;

unsigned long lastTempCheck = 0;
unsigned long lastGPSCheck = 0;
unsigned long lastWeatherCheck = 0;
unsigned long lastSentData = 0;

bool isRequestPending = false;
unsigned long requestStartTime = 0;
const unsigned long REQUEST_TIMEOUT = 10000;

bool initSystem = false;

String receivedItem = "";

float oti602AmbientTemp = 0.0;
float oti602ObjectTemp = 0.0;

TinyGPSPlus gps; // GPS 程式初始化
HardwareSerial GPS_Serial(1); // GPS 連接
HardwareSerial H87_Serial(2); // 8735 連接

DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE); // 初始化溫度感應器
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
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000); // 100kHz
  Serial.print("I2C Initialized for OTI602 on SDA=");
  Serial.print(I2C_SDA_PIN);
  Serial.print(", SCL=");
  Serial.println(I2C_SCL_PIN);
}

/**
 * 問題:
 * API 要資訊要非常久
 * 還有 IR 沒有資訊
 */
void loop() {
    unsigned long currentMillis = millis();
    // Temperature check
    if (currentMillis - lastTempCheck >= TEMP_INTERVAL || initSystem == false) {
        GetTemp(); 
        lastTempCheck = currentMillis;
    }
    if (currentMillis - lastGPSCheck >= GPS_INTERVAL || initSystem == false || gpsLat.length() == 0 || gpsLong.length() == 0) {
        checkGPS();
        lastGPSCheck = currentMillis;
          Serial.println("GPS");
    }
    if (currentMillis - lastSentData >= SENDDATA_INTERVAL || initSystem == false) {
        sendData22();
        lastSentData = currentMillis;
    }
    ReadOTI602Temp();
    processSerial();

    if (initSystem == false) {
        initSystem = true; // 在開機時把所有程式都跑一遍
        Serial.println("Initial system checks complete.");
    }
}

void processSerial() {      
        if (H87_Serial.available()) {
            String receivedData = H87_Serial.readStringUntil('\n'); // 讀直到下一行
            Serial.print("來自 HUB8735: ");
            receivedItem = receivedData;
        }
}
void checkGPS() {
    if (GPS_Serial.available()) {
      Serial.println("GPS Serial is available");
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


bool readOTI602Temperatures(float *ambientTemp, float *objectTemp) {
  byte data[6];
  
  // 步驟1和2: 發送寫地址和讀取指令
  Wire.beginTransmission(OTI602_ADDR);
  Wire.write(0x80);  // 讀取指令
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print("發送讀取指令錯誤: ");
    Serial.println(error);
    return false;
  }
  
  // 步驟3和4: 讀取數據
  // Arduino的Wire庫在requestFrom中自動處理重複開始條件和讀取位址
  byte bytesReceived = Wire.requestFrom(OTI602_ADDR, 6);
  
  Serial.print("接收到的數據量: ");
  Serial.println(bytesReceived);
  
  if (bytesReceived != 6) {
    return false;
  }
  
  // 讀取6個位元組
  for (int i = 0; i < 6; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
      Serial.print("數據[");
      Serial.print(i);
      Serial.print("]: 0x");
      Serial.println(data[i], HEX);
    } else {
      Serial.println("讀取數據時發生錯誤");
      return false;
    }
  }
  
  // 計算環境溫度 (前3個位元組)
  int32_t rawAmbient = data[0] + (data[1] << 8) + (data[2] << 16);
  if (data[2] >= 0x80) {
    rawAmbient -= 0x1000000;  // 處理負溫度
  }
  *ambientTemp = rawAmbient / 200.0f;
  
  // 計算物體溫度 (後3個位元組)
  int32_t rawObject = data[3] + (data[4] << 8) + (data[5] << 16);
  if (data[5] >= 0x80) {
    rawObject -= 0x1000000;  // 處理負溫度
  }
  *objectTemp = rawObject / 200.0f;
  
  return true;
}

float prevOti602JbjectTemp = NAN;
const float tempChangeThreshold = 0.5;
void ReadOTI602Temp() {
  if (readOTI602Temperatures(&oti602AmbientTemp, &oti602ObjectTemp)) {
    Serial.print("OTI602 Sensor -> Ambient: ");
    Serial.print(oti602AmbientTemp, 2);
    Serial.print(" *C, Object: ");
    Serial.print(oti602ObjectTemp, 2);
    Serial.println(" *C");
    if (!isnan(prevOti602JbjectTemp)) {
      float tempDelta = abs(oti602ObjectTemp - prevOti602JbjectTemp);
      if (tempDelta > tempChangeThreshold) {
        Serial.println("!!!!!!!!!!!!!!!");
        H87_Serial.println("true");
      } else { 
        H87_Serial.println("false");
      }
    }
  } else {
    Serial.println("Failed to read from OTI602 sensor!");
    // Optionally set temps to NaN or a specific error value
    // 如果沒有把 OTI602 設成 NAN
    oti602AmbientTemp = NAN;
    oti602ObjectTemp = NAN;
    prevOti602JbjectTemp = NAN;
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
        // 更新 GPS 資訊
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

// 傳送要氣象局資料的程式
void sendRequest(String lng, String lat) {
  RequestOptions options;
  options.method = "GET";
  
  if (lat.length() > 0) {} else {
    Serial.println("No lat");
    lat = defaultlat;
  }
  if (lng.length() > 0) {} else {
    Serial.println("No long");
    lng = defaultlong;
  }
  // 網址包含 經緯度
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

// 要本地溫度與濕度
void GetTemp() {
  float temp = dht_sensor.readTemperature();
  float hum = dht_sensor.readHumidity();
  Serial.print("hum: ");
  Serial.print(hum);
  Serial.print(" Temp: ");
  Serial.println(temp);
}


// 傳資料給伺服器並把繼電器的設定也從伺服器拉下來
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
        lastFetchTime = millis();
    }

    // Parse weather 
    // 要傳給伺服器的資料 json 初始化
    DynamicJsonDocument weatherData(2048); // 大小
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

    // 檢查 8735 給的資料
    JsonArray detect = doc.createNestedArray("local_detect");
    if (!receivedItem) {} else {
      if (receivedItem == "1") {
          detect.add("Psilopogon nuchalis");
      }
      if (receivedItem == "2") {
        detect.add("Passer montanus");
      }
      if (receivedItem == "3") {
        detect.add("Gorsachius melanolophus");
      }
      if (receivedItem == "4") {
        detect.add("Cheirotonus formosanus");
      }
      if (receivedItem == "5") {
        detect.add("Trypoxylus dichotomus");
      }
    }

    String jsonString;
    serializeJson(doc, jsonString);
    // 發送資料
    RequestOptions options;
    options.method = "POST";
    options.headers["Content-Type"] = "application/json";
    options.headers["Content-Length"] = String(jsonString.length());
    options.body = jsonString;

    // 要資料
    Response response = fetch(serverUrl2, options);
    Serial.println("Sent: " + jsonString);

    if (response.status == 200) {
      Serial.println("完成");

      // 把資訊變成文字檔
      String responseText = response.text();
      // 新增 JSON
      StaticJsonDocument<200> responseDoc;
      // 文字檔變 JSON
      DeserializationError error = deserializeJson(responseDoc, responseText);
      
      if (!error) {
        bool powerState = responseDoc["jistatus"];
        if (powerState == true) {
          digitalWrite(JIPOWER_PIN, HIGH);
          isJiPowerOn = true;
          Serial.println("Power ON requested by server");
        } else if (powerState == false) {
          digitalWrite(JIPOWER_PIN, LOW);
          isJiPowerOn = false;
          Serial.println("Power OFF requested by server");
        } else {
          Serial.println("Oops");
        }
      } else {
        Serial.print("JSON parse error: ");
        Serial.println(error.c_str());
      }
    } else {
      Serial.println("失敗");
      Serial.println("Response status: " + String(response.status));
      Serial.println("Response body: " + response.text());
    }
}