#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "Fetch.h" // Assuming this library is thread-safe or used carefully
#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>

// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// --- Pin Definitions (Adopted from Second Version) ---
#define DHT_SENSOR_PIN 33
#define DHT_SENSOR_TYPE DHT11
#define JIPOWER_PIN 25
#define I2C_SDA_PIN 21 // Ensure no conflict with other peripherals on 21
#define I2C_SCL_PIN 22
#define OTI602_ADDR 0x10 // Address from second version
#define LED_PIN 32
// GPS Serial1 pins
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
// H87 Serial2 pins
#define H87_RX_PIN 26
#define H87_TX_PIN 27

// --- WiFi Settings ---
const char *ssid = "hel";
const char *password = "1234567890";

// --- API URLs ---
const char *serverUrl1 = "https://hpg7.sch2.top/weather/v2/";
const char *serverUrl2 = "https://zb-logger.sch2.top/logger/store";

// --- Timing Intervals ---
const unsigned long TEMP_INTERVAL = 5000;
const unsigned long GPS_PROCESS_INTERVAL = 100; // How often to check Serial buffer
const unsigned long OTI602_INTERVAL = 3000;
const unsigned long WEATHER_REQ_INTERVAL = 60000; // How often to request weather
const unsigned long SENDDATA_INTERVAL = 30000; // How often to send data
const unsigned long WIFI_CHECK_INTERVAL = 15000; // How often to check WiFi

// --- Global Variables for Sensor Data (Shared between tasks, protected by Mutex) ---
volatile float current_dht_temp = NAN;
volatile float current_dht_hum = NAN;
volatile float current_oti_ambient = NAN;
volatile float current_oti_object = NAN;
String current_gps_lat = "";
String current_gps_long = "";
String current_gps_time = "";
String current_received_item = ""; // From H87
volatile bool current_isJiPowerOn = false;
volatile bool current_isLedOn = false; // Added for LED status
String current_weather_data_json =
    "{\"weather\":\"晴\",\"temperature\":25,\"humidity\":60,\"location\":\"臺北市士林區\",\"daliyHigh\":26,\"daliyLow\":15}"; // Default/cached weather JSON

// --- Default GPS ---
const char *defaultlat = "25.134393";
const char *defaultlong = "121.469968";

// --- Hardware Handles ---
TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);
HardwareSerial H87_Serial(2);
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

// --- FreeRTOS Handles ---
QueueHandle_t networkRequestQueue;
SemaphoreHandle_t sensorDataMutex; // Protects sensor/GPS/status variables
SemaphoreHandle_t weatherDataMutex; // Protects current_weather_data_json

// --- Network Task Definitions ---
#define NETWORK_TASK_STACK_SIZE 8192 // Increased stack size for network/JSON
#define NETWORK_TASK_PRIORITY 1
#define NETWORK_QUEUE_LENGTH 5

// Enum to define network request types
typedef enum { REQ_FETCH_WEATHER, REQ_SEND_DATA } NetworkRequestType;

// Structure for queue messages
typedef struct {
  NetworkRequestType type;
} NetworkRequest;


// --- JSON Structure & Defaults ---
struct SensorData {
    // CWA (Central Weather Administration) data
    String cwa_type = "多雲";
    String cwa_location = "台北市";
    float cwa_temp = 23.5;
    float cwa_hum = 89;
    float cwa_daliyHigh = 28;
    float cwa_daliyLow = 22;
    
    // Local sensor data
    float local_temp = 26;
    float local_hum = 72;
    String local_gps_lat = "25.134393";
    String local_gps_long = "121.469968";
    String local_time = "2024-03-20 15:30:00";
    bool local_jistatus = false;
    String local_detect[1] = {"獨角仙"};
} sensorData;

// JSON helper function
String createJsonString() {
    StaticJsonDocument<512> doc;
    
    // CWA data
    doc["cwa_type"] = sensorData.cwa_type;
    doc["cwa_location"] = sensorData.cwa_location;
    doc["cwa_temp"] = sensorData.cwa_temp;
    doc["cwa_hum"] = sensorData.cwa_hum;
    doc["cwa_daliyHigh"] = sensorData.cwa_daliyHigh;
    doc["cwa_daliyLow"] = sensorData.cwa_daliyLow;
    
    // Local data
    doc["local_temp"] = sensorData.local_temp;
    doc["local_hum"] = sensorData.local_hum;
    doc["local_gps_lat"] = sensorData.local_gps_lat;
    doc["local_gps_long"] = sensorData.local_gps_long;
    doc["local_time"] = sensorData.local_time;
    doc["local_jistatus"] = sensorData.local_jistatus;
    
    JsonArray detect = doc.createNestedArray("local_detect");
    detect.add(sensorData.local_detect[0]);

    String output;
    serializeJson(doc, output);
    return output;
}

// --- Forward Declarations ---
void networkTask(void *pvParameters);
void requestWeatherData_task();
void sendData22_task();
bool readTemperatures(float *ambientTemp, float *objectTemp);

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for serial connection

  Serial.println("System Booting (FreeRTOS Version)...");

  // --- Initialize Hardware ---
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  H87_Serial.begin(115200, SERIAL_8N1, H87_RX_PIN, H87_TX_PIN);

  dht_sensor.begin();
  pinMode(JIPOWER_PIN, OUTPUT);
  digitalWrite(JIPOWER_PIN, LOW); // Start with relay off
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Start with LED off

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000); // Standard 100kHz
  Serial.println("I2C Initialized. SDA=" + String(I2C_SDA_PIN) +
                 ", SCL=" + String(I2C_SCL_PIN));

  // --- Initialize WiFi ---
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  unsigned long wifiStartTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStartTime < 15000) {
    Serial.print(".");
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi connection FAILED");
    // Consider error handling - maybe restart ESP?
  }

  // --- Initialize FreeRTOS Objects ---
  networkRequestQueue =
      xQueueCreate(NETWORK_QUEUE_LENGTH, sizeof(NetworkRequest));
  sensorDataMutex = xSemaphoreCreateMutex();
  weatherDataMutex = xSemaphoreCreateMutex();

  if (networkRequestQueue == NULL || sensorDataMutex == NULL ||
      weatherDataMutex == NULL) {
    Serial.println("FATAL ERROR: Failed to create FreeRTOS objects!");
    // Halt execution or restart
    while (1) {
      delay(1000);
    };
  } else {
    Serial.println("FreeRTOS Objects (Queue, Mutexes) Created.");
  }

  // --- Create Network Task ---
  Serial.println("Creating Network Task...");
  BaseType_t taskCreated = xTaskCreatePinnedToCore(
      networkTask,           // Task function
      "NetworkTask",         // Name of the task
      NETWORK_TASK_STACK_SIZE, // Stack size
      NULL,                  // Parameter passed to the task
      NETWORK_TASK_PRIORITY, // Priority
      NULL,                  // Task handle
      1);                    // Pin task to core 1

  if (taskCreated != pdPASS) {
    Serial.println("FATAL ERROR: Failed to create Network Task!");
    while (1) {
      delay(1000);
    };
  } else {
    Serial.println("Network Task Created and Pinned to Core 1.");
  }

  Serial.println("Initialization Complete. Starting main loop on Core 0.");
}

// =========================================================================
// MAIN LOOP (Task 0 - Arduino Default on Core 0)
// Responsible for non-blocking sensor reads and queueing network requests
// =========================================================================
void loop() {
  unsigned long currentMillis = millis();
  static unsigned long lastTempCheck = 0;
  static unsigned long lastGPSProcess = 0;
  static unsigned long lastOTI602Check = 0;
  static unsigned long lastWeatherRequest = 0;
  static unsigned long lastDataSend = 0;
  static unsigned long lastWifiCheck = 0;

  // --- Non-Blocking Sensor Reads & Updates (Protected by Mutex) ---

  // DHT Sensor
  if (currentMillis - lastTempCheck >= TEMP_INTERVAL) {
    lastTempCheck = currentMillis;
    float temp = dht_sensor.readTemperature();
    float hum = dht_sensor.readHumidity();
    // Only update if read was successful (not NaN)
    if (!isnan(temp) && !isnan(hum)) {
      if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_dht_temp = temp;
        current_dht_hum = hum;
        xSemaphoreGive(sensorDataMutex);
        // Serial.println("DEBUG: DHT Updated: T=" + String(temp,1) + ", H=" + String(hum,1));
      } else {
        Serial.println("WARN: Failed to get sensor mutex for DHT update.");
      }
    } else {
      // Serial.println("DEBUG: DHT Read Failed");
    }
  }

  // OTI602 Sensor
  if (currentMillis - lastOTI602Check >= OTI602_INTERVAL) {
    lastOTI602Check = currentMillis;
    float ambTemp, objTemp;
    // readTemperatures handles retries internally
    if (readTemperatures(&ambTemp, &objTemp)) {
      if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_oti_ambient = ambTemp;
        current_oti_object = objTemp;
        xSemaphoreGive(sensorDataMutex);
        // Serial.println("DEBUG: OTI Updated: Amb=" + String(ambTemp,2) + ", Obj=" + String(objTemp,2));
      } else {
        Serial.println("WARN: Failed to get sensor mutex for OTI update.");
      }
    } else {
      // Serial.println("DEBUG: OTI Read Failed (after retries)");
      // Keep previous values or set to NaN if desired
      if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_oti_ambient = NAN; // Indicate failure
        current_oti_object = NAN;
        xSemaphoreGive(sensorDataMutex);
      }
    }
  }

  // GPS Processing (Check serial buffer frequently)
  if (currentMillis - lastGPSProcess >= GPS_PROCESS_INTERVAL) {
    lastGPSProcess = currentMillis;
    while (GPS_Serial.available() > 0) {
      if (gps.encode(GPS_Serial.read())) {
        // A new sentence has been parsed, update shared variables if valid
        bool updated = false;
        String tempLat = ""; // Use temps to minimize mutex time
        String tempLong = "";
        String tempTime = "";

        if (gps.location.isValid() && gps.location.isUpdated()) {
          tempLat = String(gps.location.lat(), 6);
          tempLong = String(gps.location.lng(), 6);
          updated = true;
        }
        if (gps.time.isValid() && gps.time.isUpdated()) {
          char timeBuffer[9];
          snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d",
                   gps.time.hour(), gps.time.minute(), gps.time.second());
          tempTime = String(timeBuffer);
          updated = true;
        }

        if (updated) {
          if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (tempLat.length() > 0) current_gps_lat = tempLat;
            if (tempLong.length() > 0) current_gps_long = tempLong;
            if (tempTime.length() > 0) current_gps_time = tempTime;
            xSemaphoreGive(sensorDataMutex);
            // Serial.println("DEBUG: GPS Updated: " + current_gps_lat + ", " + current_gps_long + " @ " + current_gps_time);
          } else {
            Serial.println("WARN: Failed to get sensor mutex for GPS update.");
          }
        }
      }
    }
  }

  // H87 Serial Processing
  if (H87_Serial.available()) {
    String receivedData = H87_Serial.readStringUntil('\n');
    receivedData.trim();
    if (receivedData.length() > 0) {
      if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_received_item = receivedData; // Store the received item
        xSemaphoreGive(sensorDataMutex);
        Serial.println("H87 Received: " + receivedData); // Debug
      } else {
        Serial.println("WARN: Failed to get sensor mutex for H87 update.");
      }
    }
  }

  // --- Trigger Network Operations (Queueing Requests) ---

  // Request Weather Data Periodically
  if (currentMillis - lastWeatherRequest >= WEATHER_REQ_INTERVAL) {
    lastWeatherRequest = currentMillis;
    NetworkRequest request;
    request.type = REQ_FETCH_WEATHER;
    // Serial.println("MainLoop: Queuing Weather Request");
    if (xQueueSend(networkRequestQueue, &request, pdMS_TO_TICKS(10)) !=
        pdPASS) {
      Serial.println("WARN: Failed to queue weather request!");
    }
  }

  // Send Sensor Data Periodically
  if (currentMillis - lastDataSend >= SENDDATA_INTERVAL) {
    lastDataSend = currentMillis;
    NetworkRequest request;
    request.type = REQ_SEND_DATA;
    // Serial.println("MainLoop: Queuing Send Data Request");
    if (xQueueSend(networkRequestQueue, &request, pdMS_TO_TICKS(10)) !=
        pdPASS) {
      Serial.println("WARN: Failed to queue send data request!");
    }
  }

  // --- WiFi Connection Check ---
  if (currentMillis - lastWifiCheck >= WIFI_CHECK_INTERVAL) {
    lastWifiCheck = currentMillis;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("MainLoop: WiFi disconnected. Attempting reconnect...");
      WiFi.reconnect(); // Non-blocking reconnect attempt
    } else {
      // Serial.println("MainLoop: WiFi Status OK."); // Debug
    }
  }

  // --- Short Delay ---
  // Yield time to other tasks (like network task, idle task)
  vTaskDelay(pdMS_TO_TICKS(20));
}

// =========================================================================
// NETWORK TASK (Task 1 - Runs on Core 1)
// Handles blocking network requests (Weather Fetch, Data Send)
// =========================================================================
void networkTask(void *pvParameters) {
  Serial.println("Network Task Started on Core " + String(xPortGetCoreID()));

  NetworkRequest receivedRequest;

  for (;;) {
    // Wait indefinitely for a request from the main loop's queue
    if (xQueueReceive(networkRequestQueue, &receivedRequest, portMAX_DELAY) ==
        pdPASS) {
      // Serial.println("NetworkTask: Received request type " + String(receivedRequest.type));

      // Ensure WiFi is connected before attempting network operations
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("NetworkTask: WiFi not connected. Skipping request.");
        // Optional: Add a delay before checking queue again to avoid spamming logs if WiFi is down
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue; // Go back to waiting for a queue message
      }

      // Process the request
      switch (receivedRequest.type) {
      case REQ_FETCH_WEATHER:
        requestWeatherData_task();
        break;
      case REQ_SEND_DATA:
        sendData22_task();
        break;
      default:
        Serial.println("NetworkTask: Unknown request type received!");
        break;
      }
    }
    // Small delay to allow task switching, although xQueueReceive blocks
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// =========================================================================
// Network Functions (Called ONLY by Network Task on Core 1)
// =========================================================================

// --- Fetch Weather Data (Task Context) ---
void requestWeatherData_task() {
  Serial.println("NetworkTask: Executing Fetch Weather...");
  String lat, lng;

  // Safely get current GPS coordinates from shared variables
  if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    lat = (current_gps_lat.length() > 0) ? current_gps_lat : defaultlat;
    lng = (current_gps_long.length() > 0) ? current_gps_long : defaultlong;
    xSemaphoreGive(sensorDataMutex);
  } else {
    Serial.println(
        "NetworkTask: Failed to get sensor mutex for GPS read (Weather). Using defaults.");
    lat = defaultlat;
    lng = defaultlong;
  }

  RequestOptions options;
  options.method = "GET";
  // options.timeout = 15000; // Set timeout if fetch library supports it

  String serverUrlString = serverUrl1 + lat + "/" + lng;
  Serial.println("NetworkTask: Requesting Weather URL: " + serverUrlString);

  unsigned long reqStart = millis();
  // fetch() is a BLOCKING CALL - OK inside this dedicated task
  Response response = fetch(serverUrlString.c_str(), options);
  unsigned long reqTime = millis() - reqStart;
  Serial.println("NetworkTask: Weather fetch took " + String(reqTime) + " ms");

  if (response.status == 200) {
    Serial.println("NetworkTask: Weather data received OK.");
    String responseText = response.text();
    // Safely update the shared weather data JSON string
    if (xSemaphoreTake(weatherDataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      current_weather_data_json = responseText;
      xSemaphoreGive(weatherDataMutex);
      Serial.println("NetworkTask: Weather data cache updated.");

      // Optional: Print confirmation from the updated cache
      StaticJsonDocument<512> tempParseDoc;
      DeserializationError error =
          deserializeJson(tempParseDoc, responseText); // Parse the new data
      if (!error) {
        Serial.println("  -> Weather: " +
                       tempParseDoc["weather"].as<String>() + ", " +
                       String(tempParseDoc["temperature"].as<float>()) + " C, " +
                       String(tempParseDoc["humidity"].as<float>()) + "%");
      }
    } else {
      Serial.println(
          "NetworkTask: WARN: Failed to get weather mutex to update cache!");
    }
  } else {
    Serial.println("NetworkTask: Weather data fetch FAILED. HTTP Status: " +
                   String(response.status));
    // Keep the old cached data, don't update current_weather_data_json
  }
}
// --- Send Sensor Data (Task Context) ---
void sendData22_task() {
  Serial.println("NetworkTask: Executing Send Data...");

  // --- Safely Read ALL Shared Data needed for the JSON payload ---
  float temp_dht = NAN, hum_dht = NAN, temp_oti_amb = NAN, temp_oti_obj = NAN;
  String gps_lat, gps_long, gps_time, item_to_send;
  bool jistatus, ledstatus; // Read current hardware status

  // Increased timeout slightly for safety, though 200ms should be plenty
  if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
    temp_dht = current_dht_temp;
    hum_dht = current_dht_hum;
    temp_oti_amb = current_oti_ambient;
    temp_oti_obj = current_oti_object;
    gps_lat = current_gps_lat;
    gps_long = current_gps_long;
    gps_time = current_gps_time;
    item_to_send = current_received_item; // Copy the item to send
    jistatus = current_isJiPowerOn;
    ledstatus = current_isLedOn;
    current_received_item = ""; // Clear item *after* copying it, under mutex protection
    xSemaphoreGive(sensorDataMutex);
  } else {
    Serial.println(
        "NetworkTask: WARN: Failed to get sensor mutex for reading data to send! Skipping send.");
    return; // Skip sending if we couldn't get fresh data
  }

  // --- Safely Read Cached Weather Data ---
  String local_weather_json;
  if (xSemaphoreTake(weatherDataMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
    local_weather_json = current_weather_data_json; // Copy the cached JSON
    xSemaphoreGive(weatherDataMutex);
  } else {
    Serial.println(
        "NetworkTask: WARN: Failed to get weather mutex for reading data to send! Using default weather.");
    local_weather_json =
        "{\"weather\":\"晴\",\"temperature\":25,\"humidity\":60,\"location\":\"臺北市士林區\",\"daliyHigh\":26,\"daliyLow\":15}";
  }

  // --- Build JSON Payload ---
  // *** FIX 1: Increased JSON document size ***
  StaticJsonDocument<1024> doc;        // Output JSON doc (Increased size)
  StaticJsonDocument<512> weatherData; // Doc to parse weather cache

  DeserializationError error = deserializeJson(weatherData, local_weather_json);
  if (error) {
    Serial.print("NetworkTask: Error parsing cached weather JSON: ");
    Serial.println(error.c_str());
    // Use defaults in the payload construction below if parsing fails
  }

  // Populate weather fields
  doc["cwa_type"] = weatherData["weather"] | "晴";
  doc["cwa_location"] = weatherData["location"] | "臺北市士林區"; // Use default if parse fails
  doc["cwa_temp"] = weatherData["temperature"] | 27.7;
  doc["cwa_hum"] = weatherData["humidity"] | 60;
  doc["cwa_daliyHigh"] = weatherData["daliyHigh"] | 26;
  doc["cwa_daliyLow"] = weatherData["daliyLow"] | 15;

  // Populate local sensor fields
  if (isnan(temp_dht)) { doc["local_temp"] = nullptr; }
  else { doc["local_temp"] = round(temp_dht * 10.0) / 10.0; } // Round to 1 decimal place if needed

  if (isnan(hum_dht)) { doc["local_hum"] = nullptr; }
  else { doc["local_hum"] = round(hum_dht); } // Round to integer if needed

  doc["local_gps_lat"] = gps_lat.length() > 0 ? gps_lat : defaultlat;
  doc["local_gps_long"] = gps_long.length() > 0 ? gps_long : defaultlong;

  // *** FIX 4: Send null for time if unavailable, otherwise send HH:MM:SS ***
  // Note: Your target example has a full timestamp "YYYY-MM-DD HH:MM:SS".
  // If the server *requires* that format, you'll need an RTC or different logic.
  // Sending just HH:MM:SS or null based on GPS validity.
  if (gps_time.length() > 0) {
      doc["local_time"] = gps_time;
  } else {
      doc["local_time"] = "2024-03-20 15:30:00"; // Hardcoded for testing
      // Or, if you MUST send *something* formatted like the example:
      // doc["local_time"] = "1970-01-01 00:00:00"; // Placeholder timestamp
  }

  doc["local_jistatus"] = jistatus;

  // *** FIX 3: Temporarily comment out IR fields if server doesn't expect them ***
  // If the error persists, uncomment these lines. If removing them fixes it,
  // then the server doesn't want these fields.
  /*
  if (isnan(temp_oti_amb)) { doc["ir_ambient"] = nullptr; }
  else { doc["ir_ambient"] = round(temp_oti_amb * 10.0) / 10.0; } // Round

  if (isnan(temp_oti_obj)) { doc["ir_object"] = nullptr; }
  else { doc["ir_object"] = round(temp_oti_obj * 10.0) / 10.0; } // Round
  */

  // Process received item for detection array
  JsonArray detect = doc.createNestedArray("local_detect");
  if (item_to_send.length() > 0) {
    // Assuming item_to_send contains the name directly or an ID to map
    // Using the switch based on your previous code:
    switch (item_to_send.charAt(0)) {
      case '1': detect.add("Psilopogon nuchalis"); break; // Example mapping
      case '2': detect.add("Passer montanus"); break;
      case '3': detect.add("Gorsachius melanolophus"); break;
      case '4': detect.add("Cheirotonus formosanus"); break;
      case '5': detect.add("Trypoxylus dichotomus"); break; // Matched target example "獨角仙"
      // If item_to_send *is* the name (e.g., "獨角仙"), just do:
      // detect.add(item_to_send);
    }
  }
  // If item_to_send is empty, an empty array [] will be correctly sent.

  // --- Serialize and DEBUG ---
  String jsonString;
  // *** FIX 2: Check serialization result ***
  size_t jsonSize = serializeJson(doc, jsonString);

  Serial.println("NetworkTask: Preparing to send JSON (" + String(jsonSize) + " bytes / " + String(doc.memoryUsage()) + " used):"); // Log size and memory used
  Serial.println("----------------BEGIN JSON------------------");
  Serial.println(jsonString); // Print the exact JSON being sent
  Serial.println("-----------------END JSON-------------------");

  if (jsonSize == 0) {
      Serial.println("NetworkTask: ERROR - serializeJson failed or produced empty string! Skipping send.");
      return; // Don't try to send empty data
  }
  // Optional check if you are close to capacity
  if (doc.overflowed()) {
       Serial.println("NetworkTask: WARNING - JSON document overflowed! Increase StaticJsonDocument size.");
       // Consider returning here as the JSON is likely corrupt/truncated
       // return;
  }


  // --- Send POST Request ---
  RequestOptions options;
  options.method = "POST";
  options.headers["Content-Type"] = "application/json";
  options.body = jsonString; // Use the generated string

  Serial.println("NetworkTask: Sending Data via fetch()...");

  unsigned long reqStart = millis();
  Response response = fetch(serverUrl2, options); // BLOCKING CALL
  unsigned long reqTime = millis() - reqStart;
  Serial.println("NetworkTask: Send data took " + String(reqTime) + " ms");

  // --- Process Response (Keep existing logic) ---
  if (response.status == 200) {
    // ... your existing success handling ...
    Serial.println("NetworkTask: Data send successful.");
    String responseText = response.text();
    // ... rest of parsing jistatus/ledstatus ...
     StaticJsonDocument<200> responseDoc; // Adjust size if response is larger
    DeserializationError err = deserializeJson(responseDoc, responseText);

    if (!err) {
      bool relayStateChanged = false;
      bool ledStateChanged = false;

      // --- Safely Update Relay and LED Status ---
      if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        // Check and update Relay (JiPower)
        if (responseDoc.containsKey("jistatus")) {
          bool desiredPowerState = responseDoc["jistatus"];
          if (desiredPowerState != current_isJiPowerOn) {
            current_isJiPowerOn = desiredPowerState; // Update shared variable
            digitalWrite(JIPOWER_PIN,
                         current_isJiPowerOn ? HIGH : LOW); // Update hardware
            relayStateChanged = true;
          }
        }

        // Check and update LED
        if (responseDoc.containsKey("ledstatus")) {
          bool desiredLedState = responseDoc["ledstatus"];
          if (desiredLedState != current_isLedOn) {
            current_isLedOn = desiredLedState; // Update shared variable
            digitalWrite(LED_PIN,
                         current_isLedOn ? HIGH : LOW); // Update hardware
            ledStateChanged = true;
          }
        }
        xSemaphoreGive(sensorDataMutex); // Release mutex

        // Log changes outside the mutex lock
        if (relayStateChanged) {
          Serial.println("NetworkTask: Relay state changed to " +
                         String(current_isJiPowerOn ? "ON" : "OFF"));
        }
        if (ledStateChanged) {
          Serial.println("NetworkTask: LED state changed to " +
                         String(current_isLedOn ? "ON" : "OFF"));
        }

      } else {
        Serial.println(
            "NetworkTask: WARN: Failed to get sensor mutex to update relay/LED state!");
      }
      // --- End Safe Update ---

    } else {
      Serial.println("NetworkTask: Failed to parse server response JSON: " +
                     String(err.c_str()));
      Serial.println("NetworkTask: Response was: " + responseText);
    }

  } else {
    Serial.println("NetworkTask: Data send FAILED. HTTP Status: " +
                   String(response.status));
    // *** Log the body even on failure ***
    Serial.println("NetworkTask: Response Body: " + response.text());
  }
}

// =========================================================================
// Helper Functions
// =========================================================================

// --- Check I2C Devices ---
void checkI2CDevices() {
    // ... function code ...
}

// --- Read OTI602 Temperatures (with Retries) ---
// Called by main loop()
// **** ENSURE THIS ENTIRE BLOCK IS PRESENT AND NOT COMMENTED OUT ****
bool readTemperatures(float *ambientTemp, float *objectTemp) {
  byte data[6];
  byte retryCount = 0;
  const byte MAX_RETRIES = 3;

  while (retryCount < MAX_RETRIES) {
    Wire.beginTransmission(OTI602_ADDR);
    Wire.write(0x80); // Read command
    byte error = Wire.endTransmission();
    if (error != 0) {
      // Serial.print("OTI Tx Err "); Serial.println(error); // Reduce noise
      retryCount++;
      delay(50);
      continue;
    }

    delay(50); // Wait for measurement according to datasheet/example

    byte bytesReceived = Wire.requestFrom(OTI602_ADDR, 6);
    if (bytesReceived != 6) {
      // Serial.print("OTI Rx Err "); Serial.println(bytesReceived); // Reduce noise
      while (Wire.available()) { Wire.read(); } // Clear buffer
      retryCount++;
      delay(50);
      continue;
    }

    // Read data bytes
    for (int i = 0; i < 6; i++) { data[i] = Wire.read(); }

    // Process Ambient Temperature (Bytes 0, 1, 2)
    int32_t rawAmbient = data[0] | (data[1] << 8) | (data[2] << 16);
    // Sign extend if negative (check MSB of 24 bits)
    if (rawAmbient & 0x800000) { rawAmbient |= 0xFF000000; }
    *ambientTemp = rawAmbient / 200.0f;

    // Process Object Temperature (Bytes 3, 4, 5)
    int32_t rawObject = data[3] | (data[4] << 8) | (data[5] << 16);
    // Sign extend if negative
    if (rawObject & 0x800000) { rawObject |= 0xFF000000; }
    *objectTemp = rawObject / 200.0f;

    // Basic sanity check for temperature ranges
    if (*ambientTemp < -40 || *ambientTemp > 125 || *objectTemp < -40 ||
        *objectTemp > 300) { // Object range might be wider
      // Serial.println("OTI Unreasonable Temp"); // Reduce noise
      retryCount++;
      delay(50);
      continue;
    }

    return true; // Success
  }

  // If loop finishes without returning true, all retries failed
  *ambientTemp = NAN; // Indicate failure
  *objectTemp = NAN;
  return false; // Failed after retries
}
// **** END OF FUNCTION BLOCK ****
