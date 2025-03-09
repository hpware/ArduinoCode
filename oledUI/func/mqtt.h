
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.println("[MQTT] 重新連線中...");
    if (client.connect("ESP32_Client")) {
      Serial.println("[MQTT] 連線成功！");
      client.subscribe("test_feeder");
      client.subscribe("test_feedmodel");
      client.subscribe("test_feedtime");
      client.subscribe("test_feedinterval");
      client.subscribe("test_feeding_time");
    } else {
      Serial.println("[MQTT] 連線失敗，5 秒後重試...");
      delay(5000);
    }
  }
}


void sendMQTT() {
  char phMessage[10], tempMessage[10];
  dtostrf(lastPH, 4, 2, phMessage);
  dtostrf(lastTemp, 4, 2, tempMessage);
  
  Serial.print("發送 pH 值：");
  Serial.println(phMessage);
  client.publish("sensor/ph", phMessage);

  Serial.print("發送水溫值：");
  Serial.println(tempMessage);
  client.publish("sensor/temp", tempMessage);
}
