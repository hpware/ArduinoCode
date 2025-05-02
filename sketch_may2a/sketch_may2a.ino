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
  xTaskCreatePinnedToCore(MainTask, "MainTask", 10000, NULL, 1, &MainTaskC, 0);
  xTaskCreatePinnedToCore(Task2, "Task2", 10000, NULL, 1, &Task2C, 0);
}

// 留空
void loop() {}

// 核心 1
void MainTask(void *pvParameters) {
  while (true) {

  }
}

// 核心 2
void Task2(void *pvParameters) {
  while (true) {

  }
}
