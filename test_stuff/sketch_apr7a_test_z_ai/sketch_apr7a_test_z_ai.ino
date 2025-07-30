#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>
#include <FreeRTOS.h>
#include <task.h>
#include <ArduinoHttpClient.h>

// Pin Definitions
#define DHT_SENSOR_PIN 33
#define DHT_SENSOR_TYPE DHT11
#define JIPOWER_PIN 25
#define LED_PIN 32
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define OTI602_ADDR 0x10

// WiFi Settings
const char *ssid = "hel";
const char *password = "1234567890";

// API URLs
const char *weatherApiHost = "hpg7.sch2.top";
const int weatherApiPort = 443; // HTTPS
const char *weatherApiPath = "/weather/v2/"; // Lat and Long will be appended

const char *loggerApiHost = "zb-logger.sch2.top";
const int loggerApiPort = 443; // Assuming HTTPS
const char *loggerApiPath = "/logger/store";

// Default GPS Location
String defaultlat = "25.134393";
String defaultlong = "121.469968";

// Global Variables for Inter-Task Communication
volatile String gpsLat = "";
volatile String gpsLong = "";
volatile String gpsTime = "";
volatile float dhtTemp = NAN;
volatile float dhtHum = NAN;
volatile float oti602AmbientTemp = NAN;
volatile float oti602ObjectTemp = NAN;
volatile String receivedItem = "";
volatile bool isJiPowerOn = false;
volatile bool isLedOn = false;

// Task Handles
TaskHandle_t gpsTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t httpTaskHandle = NULL;
TaskHandle_t serialProcessTaskHandle = NULL;

// Mutex for protecting Serial output if multiple tasks try to write at once
SemaphoreHandle_t xSerialSemaphore;

// Sensor Objects
TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);
HardwareSerial H87_Serial(2);
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
WiFiClientSecure wifiClient; // Use WiFiClientSecure for HTTPS

// Intervals (in milliseconds)
const unsigned long TEMP_INTERVAL = 2000;
const unsigned long GPS_INTERVAL = 1000; // Check GPS more frequently
const unsigned long WEATHER_FETCH_INTERVAL = 60000; // Fetch weather data every 60 seconds
const unsigned long DATA_SEND_INTERVAL = 10000; // Send data to logger every 10 seconds
const unsigned long IR_SENSOR_INTERVAL = 1000; // Read IR sensor every 1 second

// Function prototypes for tasks
void GpsTask(void *pvParameters);
void SensorTask(void *pvParameters);
void HttpTask(void *pvParameters);
void SerialProcessTask(void *pvParameters);

// Helper function to safely print to Serial
void safePrint(const String &message) {
  if (xSerialSemaphore != NULL) {
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t)10) == pdTRUE) {
      Serial.println(message);
      xSemaphoreGive(xSerialSemaphore);
    }
  } else {
    Serial.println(message); // Fallback if semaphore is not initialized
  }
}

void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  H87_Serial.begin(115200, SERIAL_8N1, 26, 27);

  safePrint("System starting...");
  safePrint("Connecting to WiFi...");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  safePrint("\nWiFi connected");
  safePrint("IP address: " + WiFi.localIP().toString());

  // Initialize sensors
  dht_sensor.begin();
  pinMode(JIPOWER_PIN, OUTPUT);
  digitalWrite(JIPOWER_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

  safePrint("I2C Initialized for OTI602");
  safePrint("DHT Sensor Initialized");
  safePrint("GPS Serial Initialized");
  safePrint("H87 Serial Initialized");

  // Create the serial semaphore
  xSerialSemaphore = xSemaphoreCreateMutex();
  if (xSerialSemaphore == NULL) {
    safePrint("Failed to create serial semaphore!");
  }

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(
      GpsTask,          // Task function
      "GpsTask",        // Name of the task
      4096,             // Stack size (words)
      NULL,             // Task input parameter
      1,                // Priority of the task
      &gpsTaskHandle,   // Task handle
      1);               // Core where the task should run

  xTaskCreatePinnedToCore(
      SensorTask,
      "SensorTask",
      4096,
      NULL,
      2, // Higher priority for sensors
      &sensorTaskHandle,
      1);

  xTaskCreatePinnedToCore(
      HttpTask,
      "HttpTask",
      8192, // Larger stack for HTTP operations and JSON
      NULL,
      1,
      &httpTaskHandle,
      1);

  xTaskCreatePinnedToCore(
      SerialProcessTask,
      "SerialProcessTask",
      2048,
      NULL,
      1,
      &serialProcessTaskHandle,
      1);
  
  safePrint("FreeRTOS tasks created.");
}

void loop() {
  // Nothing to do here, all work is done by tasks
  vTaskDelay(portMAX_DELAY);
}

// --- Task Implementations ---

void GpsTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(GPS_INTERVAL);

  for (;;) {
    // Feed the GPS serial data to the TinyGPS++ object
    while (GPS_Serial.available() > 0) {
      gps.encode(GPS_Serial.read());
    }

    if (gps.location.isUpdated() && gps.location.isValid()) {
      gpsLat = String(gps.location.lat(), 6);
      gpsLong = String(gps.location.lng(), 6);
      // safePrint("GPS Updated: Lat=" + gpsLat + ", Long=" + gpsLong);
    }
    if (gps.time.isUpdated() && gps.time.isValid()) {
      char timeBuffer[10];
      snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      gpsTime = String(timeBuffer);
      // safePrint("GPS Time Updated: " + gpsTime);
    }
    
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

bool readOTI602Temperatures(float *ambientTemp, float *objectTemp) {
  byte data[6];
  Wire.beginTransmission(OTI602_ADDR);
  Wire.write(0x80);
  byte error = Wire.endTransmission();
  if (error != 0) return false;
  
  byte bytesReceived = Wire.requestFrom(OTI602_ADDR, 6);
  if (bytesReceived != 6) return false;
  
  for (int i = 0; i < 6; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
    } else {
      return false;
    }
  }
  
  int32_t rawAmbient = data[0] + (data[1] << 8) + (data[2] << 16);
  if (data[2] >= 0x80) rawAmbient -= 0x1000000;
  *ambientTemp = rawAmbient / 200.0f;
  
  int32_t rawObject = data[3] + (data[4] << 8) + (data[5] << 16);
  if (data[5] >= 0x80) rawObject -= 0x1000000;
  *objectTemp = rawObject / 200.0f;
  
  return true;
}

void SensorTask(void *pvParameters) {
  TickType_t xLastWakeTimeTemp = xTaskGetTickCount();
  const TickType_t xPeriodTemp = pdMS_TO_TICKS(TEMP_INTERVAL);

  TickType_t xLastWakeTimeIR = xTaskGetTickCount();
  const TickType_t xPeriodIR = pdMS_TO_TICKS(IR_SENSOR_INTERVAL);

  float prevOti602ObjectTemp = NAN;
  const float tempChangeThreshold = 0.5;

  for (;;) {
    // DHT Sensor Reading
    if (xTaskGetTickCount() - xLastWakeTimeTemp >= xPeriodTemp) {
      float newTemp = dht_sensor.readTemperature();
      float newHum = dht_sensor.readHumidity();
      if (!isnan(newTemp)) dhtTemp = newTemp;
      if (!isnan(newHum)) dhtHum = newHum;
      // safePrint("DHT: Temp=" + String(dhtTemp) + ", Hum=" + String(dhtHum));
      xLastWakeTimeTemp = xTaskGetTickCount();
    }

    // IR Sensor Reading
    if (xTaskGetTickCount() - xLastWakeTimeIR >= xPeriodIR) {
      float ambient, object;
      if (readOTI602Temperatures(&ambient, &object)) {
        oti602AmbientTemp = ambient;
        oti602ObjectTemp = object;
        // safePrint("IR: Ambient=" + String(oti602AmbientTemp) + ", Object=" + String(oti602ObjectTemp));

        if (!isnan(prevOti602ObjectTemp)) {
          float tempDelta = abs(oti602ObjectTemp - prevOti602ObjectTemp);
          if (tempDelta > tempChangeThreshold) {
            H87_Serial.println("true");
          } else {
            H87_Serial.println("false");
          }
        }
        prevOti602ObjectTemp = oti602ObjectTemp;
      } else {
        oti602AmbientTemp = NAN;
        oti602ObjectTemp = NAN;
        prevOti602ObjectTemp = NAN;
      }
      xLastWakeTimeIR = xTaskGetTickCount();
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to prevent starvation
  }
}

void SerialProcessTask(void *pvParameters) {
  for (;;) {
    if (H87_Serial.available()) {
      String data = H87_Serial.readStringUntil('\n');
      receivedItem = data;
      // safePrint("Received from HUB8735: " + receivedItem);
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // Check serial periodically
  }
}

void HttpTask(void *pvParameters) {
  TickType_t xLastWakeTimeWeather = xTaskGetTickCount();
  const TickType_t xPeriodWeather = pdMS_TO_TICKS(WEATHER_FETCH_INTERVAL);

  TickType_t xLastWakeTimeSendData = xTaskGetTickCount();
  const TickType_t xPeriodSendData = pdMS_TO_TICKS(DATA_SEND_INTERVAL);

  String currentWeatherData = ""; // Stores the last fetched weather JSON

  for (;;) {
    // Fetch Weather Data
    if (xTaskGetTickCount() - xLastWakeTimeWeather >= xPeriodWeather) {
      String lat = (gpsLat.length() > 0) ? gpsLat : defaultlat;
      String lng = (gpsLong.length() > 0) ? gpsLong : defaultlong;
      String path = String(weatherApiPath) + lat + "/" + lng;

      HttpClient httpClient = HttpClient(wifiClient, weatherApiHost, weatherApiPort);
      safePrint("Fetching weather from: " + String(weatherApiHost) + path);
      
      httpClient.get(path);

      int statusCode = httpClient.responseStatusCode();
      String response = httpClient.responseBody();

      if (statusCode == 200) {
        currentWeatherData = response;
        safePrint("Weather data fetched successfully.");
      } else {
        safePrint("Failed to fetch weather. Status: " + String(statusCode) + ", Response: " + response);
      }
      httpClient.stop();
      xLastWakeTimeWeather = xTaskGetTickCount();
    }

    // Send Data to Logger
    if (xTaskGetTickCount() - xLastWakeTimeSendData >= xPeriodSendData) {
      if (currentWeatherData.length() == 0) {
        safePrint("No weather data yet, skipping send to logger.");
        // Optionally, fetch default weather data here if needed immediately
      } else {
        DynamicJsonDocument weatherDoc(1024);
        DeserializationError error = deserializeJson(weatherDoc, currentWeatherData);

        DynamicJsonDocument sendDoc(2048);
        if (error) {
          safePrint("Failed to parse weather JSON for logger. Using default values.");
          sendDoc["cwa_type"] = "N/A";
          sendDoc["cwa_location"] = "N/A";
          sendDoc["cwa_temp"] = 0;
          sendDoc["cwa_hum"] = 0;
        } else {
          sendDoc["cwa_type"] = weatherDoc["weather"] | "N/A";
          sendDoc["cwa_location"] = weatherDoc["location"] | "N/A";
          sendDoc["cwa_temp"] = weatherDoc["temperature"] | 0;
          sendDoc["cwa_hum"] = weatherDoc["humidity"] | 0;
        }
        
        sendDoc["local_temp"] = dhtTemp;
        sendDoc["local_hum"] = dhtHum;
        sendDoc["local_gps_lat"] = (gpsLat.length() > 0) ? gpsLat : defaultlat;
        sendDoc["local_gps_long"] = (gpsLong.length() > 0) ? gpsLong : defaultlong;
        sendDoc["local_time"] = (gpsTime.length() > 0) ? gpsTime : "N/A";
        sendDoc["local_jistatus"] = isJiPowerOn;

        JsonArray detect = sendDoc.createNestedArray("local_detect");
        String item = receivedItem; // Make a local copy of volatile variable
        if (item == "1") detect.add("Psilopogon nuchalis");
        else if (item == "2") detect.add("Passer montanus");
        else if (item == "3") detect.add("Gorsachius melanolophus");
        else if (item == "4") detect.add("Cheirotonus formosanus");
        else if (item == "5") detect.add("Trypoxylus dichotomus");

        String jsonString;
        serializeJson(sendDoc, jsonString);
        
        HttpClient httpClient = HttpClient(wifiClient, loggerApiHost, loggerApiPort);
        safePrint("Sending data to logger: " + String(loggerApiHost) + loggerApiPath);
        
        httpClient.post(loggerApiPath, "application/json", jsonString);

        int statusCode = httpClient.responseStatusCode();
        String response = httpClient.responseBody();

        if (statusCode == 200) {
          safePrint("Data sent to logger successfully.");
          // Parse response for actuator control
          StaticJsonDocument<200> responseDoc;
          DeserializationError respError = deserializeJson(responseDoc, response);
          if (!respError) {
            if (responseDoc.containsKey("jistatus")) {
              bool jiPowerState = responseDoc["jistatus"];
              digitalWrite(JIPOWER_PIN, jiPowerState ? HIGH : LOW);
              isJiPowerOn = jiPowerState;
              safePrint("JiPower set to: " + String(jiPowerState));
            }
            if (responseDoc.containsKey("ledstatus")) {
              bool ledState = responseDoc["ledstatus"];
              digitalWrite(LED_PIN, ledState ? HIGH : LOW);
              isLedOn = ledState;
              safePrint("LED set to: " + String(ledState));
            }
          } else {
            safePrint("Failed to parse logger response JSON.");
          }
        } else {
          safePrint("Failed to send data to logger. Status: " + String(statusCode) + ", Response: " + response);
        }
        httpClient.stop();
      }
      xLastWakeTimeSendData = xTaskGetTickCount();
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Small delay
  }
}
