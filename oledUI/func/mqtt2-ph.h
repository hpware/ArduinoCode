#include "mqtt.h";

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("\n[MQTT] 收到訊息: ");
  Serial.print(topic);
  Serial.print(" -> ");

  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  Serial.println(message);

  
  if (strcmp(topic, "test_feeder") == 0 && strcmp(message, "1") == 0) {
    startFeeding();
  }

  if (strcmp(topic, "test_feedmodel") == 0) {
    feedMode = atoi(message);
  } else if (strcmp(topic, "test_feedtime") == 0) {
    feedTime = atoi(message);
  } else if (strcmp(topic, "test_feedinterval") == 0) {
    feedInterval = atoi(message);
  } else if (strcmp(topic, "test_feeding_time") == 0) {
    feedDuration = atoi(message);
  }

  if (strcmp(topic, "test_Feeder_APP") == 0 && strcmp(message, "on") == 0) {
  Serial.println("[MQTT] 收到 test_Feeder_APP on, 回傳 test_Feeder_device on");
  client.publish("test_Feeder_device", "on");
  }

  if (strcmp(topic, "Feeder_APP") == 0 && strcmp(message, "on") == 0) {
  Serial.println("[MQTT] 收到 Feeder_APP on, 回傳 test_Feeder_device on");
  client.publish("test_Feeder_device", "on");
  }
}


void sendFeedResidue() {
  char feedResidueMessage[10];
  dtostrf(feedResidue, 4, 2, feedResidueMessage);  
  client.publish("test_feed_residue", feedResidueMessage);
  Serial.print("[MQTT] 傳送餘餵食量: ");
  Serial.println(feedResidueMessage);
}


void readPhTemp() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  sendFeedResidue();
  client.loop();

  if (millis() - modbusTimer >= interval) {
    modbusTimer = millis();
    uint8_t result = node.readHoldingRegisters(MODBUS_REGISTER, 1);
    if (result == node.ku8MBSuccess) {
      lastPH = node.getResponseBuffer(0) / 100.0;
      modbusErrorCount = 0;
      Serial.print("[Modbus] pH 值: ");
      Serial.println(lastPH);
    } else {
      modbusErrorCount++;
      Serial.println("[Modbus] 讀取失敗");
      if (modbusErrorCount >= modbusErrorThreshold) {
        digitalWrite(LED_32, HIGH);
      }
    }
    node.clearResponseBuffer();
  }

  if (millis() - tempTimer >= interval) {
    tempTimer = millis();
    sensors.requestTemperatures();
    lastTemp = sensors.getTempCByIndex(0);
    Serial.print("[溫度] 水溫: ");
    Serial.println(lastTemp);
  }

  if (millis() - mqttTimer >= interval) {
    mqttTimer = millis();
    sendMQTT();
  }
}


void startFeeding() {
  Serial.println("[Feeder] 開始餵食...");
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, LOW);
  delay(feedDuration * 1000);  
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, LOW);
  Serial.println("[Feeder] 餵食完成！");

  
  float randomPercentage = random(140, 161) / 1000.0;  

  
  feedResidue -= feedResidue * randomPercentage;
  if (feedResidue < 0) {
    feedResidue = 0;  
  }

  
  Serial.print("[Feeder] 餘餵食量: ");
  Serial.println(feedResidue);

  
  client.publish("test_feeder", "0");
  client.publish("test_lastfeed", "1");
  client.publish("test_lastfeed", "0");
  
  sendFeedResidue();  
  delay(500);
  client.publish("test_feeder", "0");

}

