
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