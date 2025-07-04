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
const char *serverUrl2 = "https://logger-v2.sch2.top/device_store/c21ba5ce-92a9-4505-a40f-8084a4d61565";
const char *serverHost2 = "logger.sch2.top";
const char *deviceId = "c21ba5ce-92a9-4505-a40f-8084a4d61565";
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
void loop() {
}

// use while(true) or while(1) to loop. (and not crash)
void MainTaskC(void *pvParameters) {
  while (true) {
    // Read DHT sensor
    temp = dht_sensor.readTemperature();
    hum = dht_sensor.readHumidity();
    // Read Hub 8735 serial data
    if (H87_Serial.available()) {
      String data = H87_Serial.readStringUntil('\n');
      if (data.length() > 0 && isPrintable(data[0])) {
        Serial.print("Hub 8735 data: ");
        h87data = data;
        Serial.println(h87data);
      }
    }
    // Read GPS serial data
    if (GPS_Serial.available()) {
      if (GPS_Serial.read() > 0) {
        if (gps.encode(GPS_Serial.read())) {
          if (gps.location.isValid()) {
            gpsLat = String(gps.location.lat());
            gpsLong = String(gps.location.lng());
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
  WiFiClientSecure client;
  client.setInsecure();

  int retries = 5;
  while (!client.connect(serverHost2, 443) && (retries-- > 0)) {
    Serial.print(".");
  }
  Serial.println();
  if (!client.connected()) {
    Serial.println("Failed to connect...");
  }
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

  String jsonObject = String("{\"cwa_type\":\"") + cwaType + "\",\"cwa_location\":\"" + cwaLocation + "\",\"cwa_temp\":" + String(cwaTemp) + ",\"cwa_hum\":" + String(cwaHum) + ",\"cwa_daliyHigh\":" + String(cwaDailyHigh) + ",\"cwa_daliyLow\":" + String(cwaDailyLow) + ",\"local_temp\":" + String(localTemp) + ",\"local_hum\":" + String(localHum) + ",\"local_gps_lat\":\"" + localGpsLat + "\",\"local_gps_long\":\"" + localGpsLong + "\",\"local_time\":\"" + localTime + "\",\"local_jistatus\":" + (localJistatus ? "true" : "false") + ",\"local_detect\":[";
  /*
// Add elements to the local_detect array
for (size_t i = 0; i < localDetect.size(); ++i) {
    jsonObject += "\"" + localDetect[i] + "\"";
    if (i < localDetect.size() - 1) {
        jsonObject += ","; // Add a comma between elements
    }
}*/

  jsonObject += "]}";  // Close the array and the main object

  Serial.println(String("POST ") + "/api/device_store/" + deviceId + " HTTP/1.1");
  client.println(String("POST ") + "/api/device_store/" + deviceId + " HTTP/1.1");
  client.println(String("Host: ") + serverHost2);
  client.println("Connection: close\r\nContent-Type: application/json");
  client.print("Content-Length: ");
  client.println(jsonObject.length());
  client.println();
  client.println(jsonObject);
  Serial.println(jsonObject);
  int timeout = 5 * 10;  // 5 seconds
  while (!client.available() && (timeout-- > 0)) {
    delay(100);
  }
  //sSerial.println(client.available() ? "true" : "false");
  String restext = "";
  bool headersPassed = false;
  String line;
  while (client.available()) {
    line = client.readStringUntil('\n');
    if (line == "\r") {
      headersPassed = true;
      continue;
    }
    if (headersPassed) {
      restext += line + "\n";
    }
  }
  Serial.println("✅");
  DynamicJsonDocument sssdata11(2048);
  Serial.println(restext);
  DeserializationError error = deserializeJson(sssdata11, restext);
  if (sssdata11.containsKey("jistatus")) {
    isJiPowerOn = sssdata11["ledstatus"];
    digitalWrite(JIPOWER_PIN, sssdata11["jistatus"]);
  }
  if (sssdata11.containsKey("ledstatus")) {
    isLedPowerOn = sssdata11["ledstatus"];
    digitalWrite(LED_PIN, sssdata11["ledstatus"]);
  }
  if (error) {
    Serial.print("sssdata() error: ");
    Serial.println(error.f_str());
  }
  client.stop();
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