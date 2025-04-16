#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "Fetch.h"
#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>

// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define DHT_SENSOR_PIN 33
#define DHT_SENSOR_TYPE DHT11
#define JIPOWER_PIN 25

// 設定紅外線 PIN 與 地址
#define I2C_SDA_PIN 21 // 將SDA改為與DHT11共用引腳21
#define I2C_SCL_PIN 22
#define OTI602_ADDR 0x10

#define LED_PIN 32

// WiFi 設定
const char *ssid = "hel";
const char *password = "1234567890";
// API 網址
const char *serverUrl1 = "https://hpg7.sch2.top/weather/v2/";
const char *serverUrl2 = "https://zb-logger.sch2.top/logger/store";

// --- Re-introduced global 'data' string to cache raw weather JSON ---
String data = "{\"weather\":\"晴\",\"temperature\":25,\"humidity\":60,\"location\":\"臺北市士林區\",\"daliyHigh\":26,\"daliyLow\":15}"; // Default value

bool isJiPowerOn = false;
// Removed potentially unused variables: sendData, initJiPower, JipowerTime, shouldSendData

// 預設 GPS
String defaultlat = "25.134393";
String defaultlong = "121.469968";
// GPS
String gpsLat = "";
String gpsLong = "";
String gpsTime = "";

// 時間間隔設定
const unsigned long TEMP_INTERVAL = 5000;
const unsigned long GPS_INTERVAL = 10000;
const unsigned long WEATHER_INTERVAL = 60000;
const unsigned long SENDDATA_INTERVAL = 30000;
const unsigned long OTI602_INTERVAL = 3000;

unsigned long lastTempCheck = 0;
unsigned long lastGPSCheck = 0;
unsigned long lastWeatherCheck = 0;
unsigned long lastSentData = 0;
unsigned long lastOTI602Check = 0;

// API非阻塞請求控制 (Internal tracking for weather request)
unsigned long apiRequestStartTime = 0;
bool apiRequestInProgress = false;
const unsigned long API_TIMEOUT = 15000;

bool initSystem = false;

String receivedItem = "";

// 紅外線溫度計數據
float oti602AmbientTemp = NAN;
float oti602ObjectTemp = NAN;

unsigned long weatherDataTimestamp = 0;

TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);
HardwareSerial H87_Serial(2);

DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

StaticJsonDocument<768> jsonDoc; // For sending data

// FreeRTOS Handles
QueueHandle_t networkRequestQueue;
SemaphoreHandle_t sensorDataMutex; // Protects sensor/GPS/status variables
SemaphoreHandle_t weatherDataMutex; // Protects current_weather_data_json
typedef enum {
  REQ_FETCH_WEATHER,
  REQ_SEND_DATA
} NetworkRequestType;
// Enum for defining network request types
typedef struct {
  NetworkRequestType type;
} NetworkRequest;
// Enum for the str for queue messages

void networkTask(void *pvParameters); //ASK
void requestWeatherData_task(); //ASK
void sendData22_task(); //THIS IS A TASK THAT THE DEVICE FOLLOWS

void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  H87_Serial.begin(115200, SERIAL_8N1, 26, 27);
  Serial.println("系統初始化中...");

  // 連接WiFi
  WiFi.begin(ssid, password);
  Serial.print("連接WiFi");

  unsigned long wifiStartTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStartTime < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.println("IP address: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi 連接失敗，將重試");
  }

  dht_sensor.begin();
  pinMode(JIPOWER_PIN, OUTPUT);
  digitalWrite(JIPOWER_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  Serial.println("I2C 初始化完成，OTI602 SDA=" + String(I2C_SDA_PIN) +
                 ", SCL=" + String(I2C_SCL_PIN));

  // Inti FreeRTOS objs
  networkRequestQueue = xQueueCreate(NETWORK_QUEUE_LENGTH, sizeof(NetworkRequest));
    if (networkRequestQueue == NULL) {
        Serial.println("Error creating network queue");
    }  sensorDataMutex = xSemaphoreCreateMutex();
  weatherDataMutex = xSemaphoreCreateMutex();

  if (networkRequestQueue == NULL || sensorDataMutex == NULL || weatherDataMutex == NULL) {
    Serial.println("FATAL ERROR: Failed to create FreeRTOS objects!");
    while(1) {
      
    };
}

// loop() function remains the same as the previous optimized version
void loop() {
  unsigned long currentMillis = millis();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi重新連接中...");
    WiFi.reconnect();
    delay(500); // Give some time to reconnect
    return;     // Skip the rest of the loop iteration while reconnecting
  }

  // 溫度檢查
  if (currentMillis - lastTempCheck >= TEMP_INTERVAL || !initSystem) {
    GetTemp(); // Note: GetTemp only prints, doesn't store value for JSON yet
    lastTempCheck = currentMillis;
  }

  // GPS檢查
  if (currentMillis - lastGPSCheck >= GPS_INTERVAL || !initSystem ||
      gpsLat.length() == 0) {
    checkGPS();
    lastGPSCheck = currentMillis;
  }

  // OTI602檢查
  if (currentMillis - lastOTI602Check >= OTI602_INTERVAL) {
    ReadOTI602Temp();
    lastOTI602Check = currentMillis;
  }

  // 天氣API檢查 (Only request if data is old)
  if (currentMillis - lastWeatherCheck >= WEATHER_INTERVAL || !initSystem) {
    // Check if enough time has passed since the last successful update
    if (currentMillis - weatherDataTimestamp > WEATHER_INTERVAL) {
      requestWeatherData();
    }
    lastWeatherCheck = currentMillis;
  }

  // 發送數據到服務器
  if (currentMillis - lastSentData >= SENDDATA_INTERVAL || !initSystem) {
    sendData22();
    lastSentData = currentMillis;
  }

  // 處理序列通信
  processSerial();

  if (!initSystem) {
    initSystem = true;
    Serial.println("系統初始化完成");
  }

  delay(10); // Small delay to prevent busy-waiting
}


// checkI2CDevices() remains the same
void checkI2CDevices() {
  Serial.println("掃描I2C設備...");
  byte error, address;
  int deviceCount = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("找到I2C設備在地址: 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      if (address == OTI602_ADDR) Serial.print(" (OTI602)");
      Serial.println();
      deviceCount++;
    } else if (error == 4) {
      Serial.print("未知錯誤在地址 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (deviceCount == 0) {
    Serial.println("沒有找到I2C設備");
  } else {
    Serial.println("找到 " + String(deviceCount) + " 個I2C設備");
  }
}

// processSerial() remains the same
void processSerial() {
  if (H87_Serial.available()) {
    String receivedData = H87_Serial.readStringUntil('\n');
    receivedData.trim(); // Remove potential whitespace/newlines
    Serial.println("來自 HUB8735: " + receivedData);
    receivedItem = receivedData;
  }
}

// checkGPS() remains the same
void checkGPS() {
  while (GPS_Serial.available() > 0) {
    if (gps.encode(GPS_Serial.read())) {
      getSignal(); // Process if a full sentence is parsed
    }
  }

  // Check if GPS location is still invalid after processing available data
  if (!gps.location.isValid()) {
    unsigned long currentMillis = millis();
    if (currentMillis - weatherDataTimestamp > 30000 && !apiRequestInProgress) {
      Serial.println("無有效GPS數據，嘗試使用默認位置獲取天氣");
      requestWeatherData();
    }
  }
}

// readTemperatures() remains the same
bool readTemperatures(float *ambientTemp, float *objectTemp) {
  byte data[6];
  byte retryCount = 0;
  const byte MAX_RETRIES = 3;

  while (retryCount < MAX_RETRIES) {
    Wire.beginTransmission(OTI602_ADDR);
    Wire.write(0x80); // Read command
    byte error = Wire.endTransmission();

    if (error != 0) {
      Serial.print("OTI602 Tx Error (Try ");
      Serial.print(retryCount + 1);
      Serial.print("): ");
      Serial.println(error);
      retryCount++;
      delay(50);
      continue;
    }

    delay(50); // Wait for measurement
    byte bytesReceived = Wire.requestFrom(OTI602_ADDR, 6);

    if (bytesReceived != 6) {
      Serial.print("OTI602 Rx Error (Try ");
      Serial.print(retryCount + 1);
      Serial.print("): Expected 6 bytes, got ");
      Serial.println(bytesReceived);
      while(Wire.available()) { Wire.read(); }
      retryCount++;
      delay(50);
      continue;
    }

    for (int i = 0; i < 6; i++) {
      data[i] = Wire.read();
    }

    int32_t rawAmbient = data[0] | (data[1] << 8) | (data[2] << 16);
    if (rawAmbient & 0x800000) { rawAmbient |= 0xFF000000; }
    *ambientTemp = rawAmbient / 200.0f;

    int32_t rawObject = data[3] | (data[4] << 8) | (data[5] << 16);
     if (rawObject & 0x800000) { rawObject |= 0xFF000000; }
    *objectTemp = rawObject / 200.0f;

    if (*ambientTemp < -40 || *ambientTemp > 125 || *objectTemp < -40 || *objectTemp > 125) {
      Serial.print("OTI602 Unreasonable Temp (Try ");
      Serial.print(retryCount + 1);
      Serial.print("): Amb="); Serial.print(*ambientTemp);
      Serial.print(", Obj="); Serial.println(*objectTemp);
      retryCount++;
      delay(50);
      continue;
    }
    return true; // Success
  }
  return false; // Failed after retries
}

// ReadOTI602Temp() remains the same
void ReadOTI602Temp() {
  float ambTemp, objTemp;
  if (readTemperatures(&ambTemp, &objTemp)) {
    oti602AmbientTemp = ambTemp;
    oti602ObjectTemp = objTemp;
    Serial.print("OTI602 -> 環境溫度: ");
    Serial.print(oti602AmbientTemp, 2);
    Serial.print(" °C, 物體溫度: ");
    Serial.print(oti602ObjectTemp, 2);
    Serial.println(" °C");
  } else {
    Serial.println("無法讀取OTI602傳感器 (多次嘗試失敗)");
  }
}

// getSignal() remains the same
void getSignal() {
  if (gps.location.isUpdated() && gps.location.isValid()) {
    gpsLat = String(gps.location.lat(), 6);
    gpsLong = String(gps.location.lng(), 6);
    Serial.println("GPS定位更新: " + gpsLat + ", " + gpsLong);
  }

  if (gps.time.isUpdated() && gps.time.isValid()) {
    char timeBuffer[9];
    snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", gps.time.hour(),
             gps.time.minute(), gps.time.second());
    gpsTime = String(timeBuffer);
  }

  if (gps.location.isValid() &&
      millis() - weatherDataTimestamp > WEATHER_INTERVAL &&
      !apiRequestInProgress) {
    requestWeatherData();
  }
}

// Modified weather request function to update global 'data' string
void requestWeatherData() {
  if (apiRequestInProgress) {
    if (millis() - apiRequestStartTime > API_TIMEOUT) {
      Serial.println("API請求超時");
      apiRequestInProgress = false;
    } else {
      return;
    }
  }

  String lat = (gpsLat.length() > 0 && gps.location.isValid()) ? gpsLat : defaultlat;
  String lng = (gpsLong.length() > 0 && gps.location.isValid()) ? gpsLong : defaultlong;

  RequestOptions options;
  options.method = "GET";

  String serverUrl = serverUrl1 + lat + "/" + lng;
  Serial.println("請求天氣數據: " + serverUrl);

  apiRequestStartTime = millis();
  apiRequestInProgress = true;

  const char *urlChar = serverUrl.c_str();
  Response response = fetch(urlChar, options);

  apiRequestInProgress = false;
  unsigned long requestTime = millis() - apiRequestStartTime;
  Serial.print("API請求耗時: ");
  Serial.print(requestTime);
  Serial.println(" ms");

  if (response.status == 200) {
    Serial.println("天氣數據接收成功");
    // --- Store raw JSON response in global 'data' string ---
    data = response.text();
    weatherDataTimestamp = millis(); // Update timestamp

    // --- Optional: Print key values after update ---
    // Use a temporary document to parse for printing confirmation
    StaticJsonDocument<512> tempParseDoc;
    DeserializationError error = deserializeJson(tempParseDoc, data);
    if (!error) {
         Serial.println("天氣更新: " + tempParseDoc["weather"].as<String>() + ", " +
                     String(tempParseDoc["temperature"].as<float>()) + "°C, " +
                     String(tempParseDoc["humidity"].as<float>()) + "%");
    } else {
         Serial.println("天氣JSON解析失敗 (用於打印): " + String(error.c_str()));
    }
    // --- End Optional Print ---

  } else {
    Serial.println("天氣數據獲取失敗: HTTP " + String(response.status));
    // Do not update 'data' or timestamp, keep old weather values
  }
}

// GetTemp() remains the same
void GetTemp() {
  float temp = dht_sensor.readTemperature();
  float hum = dht_sensor.readHumidity();

  if (!isnan(temp) && !isnan(hum)) {
    Serial.println("DHT11 -> 濕度: " + String(hum, 1) + "%, 溫度: " +
                   String(temp, 1) + "°C");
  } else {
    Serial.println("DHT11讀取失敗");
  }
}

// Modified sendData22 function using the user-provided logic
void sendData22() {
  Serial.println("準備發送數據到伺服器...");

  // --- Use the global StaticJsonDocument for building the output ---
  jsonDoc.clear(); // Clear the document before reuse

  // --- Parse the cached weather data string ---
  // Use a temporary StaticJsonDocument for parsing within this scope
  StaticJsonDocument<512> weatherData; // Adjust size if needed
  DeserializationError error = deserializeJson(weatherData, data); // Parse global 'data'

  if (error) {
    Serial.print("錯誤: 無法解析緩存的天氣數據 JSON: ");
    Serial.println(error.c_str());
    Serial.println("緩存數據: " + data);
    // Decide how to proceed: send default values, skip sending, etc.
    // For now, we'll proceed using the default values provided in the population logic
  }

  // --- Populate JSON using the requested logic ---
  // It uses the parsed 'weatherData' object (or defaults if parsing failed)
  // and reads sensors directly.
  jsonDoc["cwa_type"] = weatherData["weather"] | "晴";
  jsonDoc["cwa_location"] = weatherData["location"] | "臺北市士林區";
  jsonDoc["cwa_temp"] = weatherData["temperature"] | 27.7;
  jsonDoc["cwa_hum"] = weatherData["humidity"] | 60;
  jsonDoc["cwa_daliyHigh"] = weatherData["daliyHigh"] | 26;
  jsonDoc["cwa_daliyLow"] = weatherData["daliyLow"] | 15;
  jsonDoc["local_temp"] = dht_sensor.readTemperature(); // Direct read
  jsonDoc["local_hum"] = dht_sensor.readHumidity();     // Direct read
  jsonDoc["local_gps_lat"] = gpsLat.length() > 0 ? gpsLat : defaultlat;
  jsonDoc["local_gps_long"] = gpsLong.length() > 0 ? gpsLong : defaultlong;
  jsonDoc["local_time"] =
      gpsTime.length() > 0 ? gpsTime : "2024-03-20 15:30:00"; // Default time if GPS time invalid
  jsonDoc["local_jistatus"] = isJiPowerOn;
  // Use isnan check and default to 0 as per user's snippet
  jsonDoc["ir_ambient"] = isnan(oti602AmbientTemp) ? 0.0 : oti602AmbientTemp;
  jsonDoc["ir_object"] = isnan(oti602ObjectTemp) ? 0.0 : oti602ObjectTemp;
  // Note: Sending 0 for NaN might be ambiguous. Sending null (as in previous version)
  // might be better if your server can handle it. Change '0.0' to 'nullptr' if needed.


  // Process received item for detection array
  JsonArray detect = jsonDoc.createNestedArray("local_detect");
  if (receivedItem.length() > 0) {
    switch (receivedItem.charAt(0)) {
    case '1': detect.add("Psilopogon nuchalis"); break;
    case '2': detect.add("Passer montanus"); break;
    case '3': detect.add("Gorsachius melanolophus"); break;
    case '4': detect.add("Cheirotonus formosanus"); break;
    case '5': detect.add("Trypoxylus dichotomus"); break;
    }
    receivedItem = ""; // Clear item after processing
  }

  // Serialize JSON to string
  String jsonString;
  serializeJson(jsonDoc, jsonString);

  // Prepare POST request
  RequestOptions options;
  options.method = "POST";
  options.headers["Content-Type"] = "application/json";
  options.body = jsonString;

  Serial.println("發送數據 (" + String(jsonString.length()) + " bytes): " + jsonString);

  unsigned long sendStart = millis();
  Response response = fetch(serverUrl2, options);
  unsigned long sendTime = millis() - sendStart;

  Serial.print("數據發送耗時: ");
  Serial.print(sendTime);
  Serial.println(" ms");

  if (response.status == 200) {
    Serial.println("數據發送成功");
    // ... (response handling remains the same) ...
     String responseText = response.text();
    StaticJsonDocument<200> responseDoc;
    DeserializationError error = deserializeJson(responseDoc, responseText);

    if (!error && responseDoc.containsKey("jistatus")) {
      bool powerState = responseDoc["jistatus"];
      if (powerState != isJiPowerOn) {
        digitalWrite(JIPOWER_PIN, powerState ? HIGH : LOW);
        isJiPowerOn = powerState;
        Serial.println(powerState ? "繼電器開啟" : "繼電器關閉");
      }
    } else if (error) {
      Serial.println("解析服務器響應失敗: " + String(error.c_str()));
      Serial.println("響應內容: " + responseText);
    }

  } else {
    Serial.println("數據發送失敗: HTTP " + String(response.status));
    Serial.println("響應內容: " + response.text());
  }
}
