#include <WiFi.h>
#include <ArduinoJson.h>
#include "Fetch.h"
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
// 創立一個 env.h 並使用 env-example.h 的模板
#include "env.h"

// Runner
TaskHandle_t MainTaskC;
TaskHandle_t Task2C;

// 初始化
void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  xTaskCreatePinnedToCore(MainTask, "MainTask", 10000, NULL, 1, &MainTaskC, 0);
  xTaskCreatePinnedToCore(Task2, "Task2", 10000, NULL, 1, &Task2C, 0);
}

// 留空
void loop() {}

// 核心 1
void MainTask(void *pvParameters) {
  while (true) {
    unsigned long currentMillis = millis();
  }
}

// 核心 2
void Task2(void *pvParameters) {
  while (true) {
    unsigned long currentMillis = millis();
    sendAudio();
    vTaskDelay(1000);
  }
}

// 傳送音檔給 Groq
void sendAudio() {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, groqApiUrl);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " + String(groqApiKey));
  String httpRequestData = "{\"messages\": [{\"role\": \"user\", \"content\": \"Tell me a short story about an adventurous cat.\"}]}" int httpResponseCode = http.POST(httpRequestData);
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println(httpResponseCode);
    Serial.println(response);
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, response);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    const char *generatedText = doc["choices"][0]["message"]["content"];
    if (generatedText) {
      Serial.println("Generated Text:");
      Serial.println(generatedText);
    } else {
      Serial.println("Could not find generated text in response.");
    }

  } else {
    Serial.printf("HTTP Error: %s\n", http.errorToString(httpResponseCode).c_str());
  }
  http.end();
}