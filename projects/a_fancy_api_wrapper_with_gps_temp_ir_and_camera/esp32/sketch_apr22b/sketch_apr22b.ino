/**
 * Sources:
 * https://randomnerdtutorials.com/esp32-dual-core-arduino-ide/
 * https://randomnerdtutorials.com/esp32-esp8266-publish-sensor-readings-to-google-sheets/
 * https://github.com/hpware/ArduinoCode/blob/main/sketch_apr7a/sketch_apr7a.ino
 * https://t3.chat/chat/ba25267b-8f0b-451c-b416-b319eef234dfca (Updated chat URL)
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
const char *weatherUrl1 = "https://hpg7.sch2.top/weather/v2/";     // Server 1
const char *weatherUrl2 = "https://c5d5bb.a.srv.yhw.tw/weather/";  // Server 2
const char *serverUrl1 = weatherUrl2;                              //　網址應該是 https://<<你的主機>>/weather/
// 主要 Nuxt 網頁與 API 伺服器
const char *testingApiHost = "hpg7.sch2.top";
const char *prodApiHost = "3m4ft6.a.srv.yhw.tw";
const char *serverHost2 = prodApiHost;                          // 主機
const char *deviceId = "6e92ff0d-adbe-43d8-b228-e4bc6f948506";  // 裝置 ID

// 開啟接收資料 (如果全關 WatchDog 會一直強制 Reset 裝置)
const bool tempHumInfo = true;
const bool enableHub8735 = true;  // 如 HUB8735 未開機，請設定為 false  不然 ESP32 的 Watchdog 會一直強制 Reset 裝置
const bool enableGPS = true;

// 下方資料不要改!!!!
// 資料
// String data = ""; // This 'data' variable is now locally scoped in MainTaskC
String h87data = "";
bool sendData = false;
bool isJiPowerOn = false;
// 預設 GPS
String defaultlat = "25.134393";
String defaultlong = "121.469968";
String gpsLat = "";
String gpsLong = "";
// Set Global temp & humidity
float temp = 0;
float hum = 0;
// Set global cwa data
DynamicJsonDocument cwa_data(512);
// Do Stuff
const unsigned long TEMP_INTERVAL = 60000;
unsigned long lastTempCheck = 0;
bool initSystem = false;
bool pullingHub8735Data = false;
int currentFlashLightLevel = 0;
// Removed bool base64DataSendDone = true; as it's replaced by base64DataInProgress

// === Base64 handling variables - moved to global scope ===
String accumulatedData = "";  // Moved from static local to global
bool base64DataInProgress = false;
const int MAX_BASE64_ARRAY = 5;  // Keep last 5 images
String base64Array[MAX_BASE64_ARRAY];
int base64ArrayIndex = 0;
// === End Base64 handling variables ===

// TaskHandle
TaskHandle_t MainTask;
TaskHandle_t SendTask;
// INIT Hardware
//HardwareSerial GPS_Serial(1);  // GPS 連接
//HardwareSerial H87_Serial(2);  // 8735 連接
#define GPS_Serial Serial1
#define H87_Serial Serial2
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
TinyGPSPlus gps;


// Setup
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
        pullingHub8735Data = true;  // Flag for this task iteration
        Serial.println("Reading HUB 8735 Data!");

        // Read until a newline. This assumes ALL messages (base64 chunks, JSON) are newline terminated.
        String data = H87_Serial.readStringUntil('\n');
        data.trim();  // Important: Remove any leading/trailing whitespace, especially for JSON

        // Debug prints
        Serial.print("Received data length: ");
        Serial.println(data.length());
        Serial.print("Received data: '");  // Use quotes to see if there are extra spaces/newlines
        Serial.print(data);
        Serial.println("'");

        // --- Start of refined base64/JSON handling logic ---

        // 1. Check for the start of a base64 block
        if (data.startsWith("<!START BLOCK!>")) {
          if (!base64DataInProgress) {
            accumulatedData = data;       // Start accumulating this first chunk
            base64DataInProgress = true;  // Set the flag to true
            Serial.println("Started accumulating base64 data.");
          } else {
            // This is an unexpected START BLOCK while one is already in progress.
            // It could mean a previous image was incomplete or an error.
            // You might want to discard the old one or handle this as an error.
            Serial.println("WARNING: Received START BLOCK while already accumulating. Discarding old data.");
            accumulatedData = data;  // Start fresh with the new block
          }
        }
        // 2. Check for the end of a base64 block (must be in progress for this to be valid)
        else if (data.endsWith("</!END BLOCK!>")) {
          if (base64DataInProgress) {
            accumulatedData += data;  // Append the last chunk which contains the end marker

            // Now, process the complete base64 block
            Serial.println("Complete base64 image received!");
            int startIndex = accumulatedData.indexOf("<!START BLOCK!>");
            int endIndex = accumulatedData.indexOf("</!END BLOCK!>");

            if (startIndex != -1 && endIndex != -1 && endIndex > startIndex) {
              String completeBase64Image = accumulatedData.substring(startIndex + 16, endIndex);  // 16 is length of "<!START BLOCK!>"

              base64ArrayIndex = (base64ArrayIndex + 1) % MAX_BASE64_ARRAY;
              base64Array[base64ArrayIndex] = completeBase64Image;

              Serial.print("Stored in slot: ");
              Serial.println(base64ArrayIndex);
              Serial.print("Total base64 data length (after removing markers): ");
              Serial.println(completeBase64Image.length());
              Serial.println("Full base64 string (truncated for display):");
              Serial.println(completeBase64Image.substring(0, min((int)completeBase64Image.length(), 100)) + "...");  // Print a snippet

            } else {
              Serial.println("Error: Start or end block markers not found correctly in the accumulated data.");
              // Optionally, reset state on error to prevent endless accumulation of bad data
              accumulatedData = "";
              base64DataInProgress = false;
            }

            // Reset state after successful processing (or detected error in markers)
            accumulatedData = "";
            base64DataInProgress = false;
          } else {
            // Received END BLOCK without a preceding START BLOCK. This is unexpected.
            Serial.println("WARNING: Received END BLOCK without START BLOCK. Discarding.");
            // Also discard any current accumulated data if there was any (though base64DataInProgress should be false)
            accumulatedData = "";
          }
        }
        // 3. If a base64 block is in progress, accumulate intermediate data chunks
        else if (base64DataInProgress) {
          accumulatedData += data;
          Serial.print("Continuing base64 accumulation. Current length: ");
          Serial.println(accumulatedData.length());
        }
        // 5. Anything else (unhandled/unknown data)
        else {
          Serial.println("Received unhandled data (neither base64 nor known JSON): ");
          Serial.println(data);
        }
        // --- End of refined base64/JSON handling logic ---
      }
      delay(10);                   // Small delay to allow serial buffer to fill
      pullingHub8735Data = false;  // Reset the flag for this task iteration
    }
    // Read GPS serial data
    if (enableGPS == true) {
      if (GPS_Serial.available()) {
        // Read bytes directly into TinyGPS++
        // Note: You might need to read all available bytes in a loop for GPS to parse effectively
        while (GPS_Serial.available()) {
          if (gps.encode(GPS_Serial.read())) {
            if (gps.location.isValid()) {
              gpsLat = String(gps.location.lat(), 6);  // Add precision for GPS
              gpsLong = String(gps.location.lng(), 6);
              // Serial.print("GPS Lat: "); Serial.println(gpsLat);
              // Serial.print("GPS Lng: "); Serial.println(gpsLong);
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
  // Use .c_str() for String when accessing DynamicJsonDocument for safety if needed,
  // but generally String comparison with const char* works ok.
  String cwaType = cwa_data.containsKey("weather") ? cwa_data["weather"].as<String>() : "陰有雨";
  String cwaLocation = cwa_data.containsKey("location") ? cwa_data["location"].as<String>() : "臺北市士林區";
  float cwaTemp = 23.5;
  int cwaHum = 89;
  int cwaDailyHigh = 28;
  int cwaDailyLow = 22;
  // Use float for local temp/hum to preserve precision before casting to int for JSON
  float localTempFloat = temp;
  float localHumFloat = hum;
  String localGpsLat = gpsLat.length() > 0 ? gpsLat : defaultlat;
  String localGpsLong = gpsLong.length() > 0 ? gpsLong : defaultlong;
  String localTime = "2025-07-12 10:15:00";  // Consider using a real-time clock for accuracy
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
    delay(100);
  }
  StaticJsonDocument<1024> doc;  // Reconsider size: 1024 is tight for even one base64 image
                                 // You might need larger, e.g., 8192 or 16384 for several images.
                                 // Or consider sending base64 images separately.
  doc["cwa_type"] = cwaType;
  doc["cwa_location"] = cwaLocation;
  doc["cwa_temp"] = cwaTemp;
  doc["cwa_hum"] = cwaHum;
  doc["cwa_daliyHigh"] = cwaDailyHigh;      // Key matches your server's expectation ("daliyLow")
  doc["cwa_daliyLow"] = cwaDailyLow;        // Key matches your server's expectation ("daliyLow")
  doc["local_temp"] = (int)localTempFloat;  // Cast to int here
  doc["local_hum"] = (int)localHumFloat;    // Cast to int here
  doc["local_gps_lat"] = localGpsLat;
  doc["local_gps_long"] = localGpsLong;
  doc["local_time"] = "2024-03-20 15:30:00";  // Update with real time if possible
  doc["local_jistatus"] = isJiPowerOn;

  // Create array of base64 data
  // Only add if there are images to send
  bool hasImagesToSend = false;
  for (int i = 0; i < MAX_BASE64_ARRAY; i++) {
    if (base64Array[i].length() > 0) {
      hasImagesToSend = true;
      break;
    }
  }
  JsonArray testingArray = doc.createNestedArray("image");
  if (hasImagesToSend) {
    for (int i = 0; i < MAX_BASE64_ARRAY; i++) {
      if (base64Array[i].length() > 0) {
        testingArray.add(base64Array[i]);
        // IMPORTANT: If you want to send only once, clear the slot after adding it to JSON
        // base64Array[i] = ""; // You can clear it here or after sending HTTP request
      }
    }
  }


  String jsonString;
  // Check serialization size before sending
  size_t json_size = measureJson(doc);
  if (json_size > 1024) {  // 1024 is the size of StaticJsonDocument
    Serial.print("WARNING: JSON document too large for StaticJsonDocument! Size: ");
    Serial.println(json_size);
    // You might need to use DynamicJsonDocument here, or handle large payloads differently.
    // For now, it will likely be truncated.
  }

  serializeJson(doc, jsonString);
  client.println("POST /api/device_store/" + String(deviceId) + " HTTP/1.1");
  client.println("Host: " + String(serverHost2));
  client.println("Connection: close");
  client.println("Content-Type: application/json");
  client.print("Content-Length: ");
  client.println(jsonString.length());
  client.println();
  client.print(jsonString);

  client.flush();  // Ensure all buffered data is sent over the network

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
        digitalWrite(JIPOWER_PIN, isJiPowerOn ? LOW : HIGH);
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
    } else {
      Serial.print("Error parsing server response JSON: ");
      Serial.println(error.f_str());
    }
  } else {
    Serial.println("Invalid HTTP response format (no body detected).");
    Serial.println(response);  // Print full response for debugging
  }

  client.stop();
  if (hasImagesToSend) {  // Only clear if there were images to send in this packet
    for (int i = 0; i < MAX_BASE64_ARRAY; i++) {
      base64Array[i] = "";
    }
    base64ArrayIndex = 0;  // Reset index to start from beginning
    Serial.println("Base64 array cleared after sending.");
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