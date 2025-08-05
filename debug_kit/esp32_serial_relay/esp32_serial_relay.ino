void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 26, 27);
}

void loop() {
    if (Serial2.available()) {
        String data = Serial2.readStringUntil('\n');
        Serial.println(data);
    }

    if (Serial.available()) {
        String data = Serial.readStringUntil('\n');
        Serial2.println(data);
    }
}