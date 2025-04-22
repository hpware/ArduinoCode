#include <WiFi.h>
#include <HTTPClient.h>
#include <driver/i2s.h>
#include "SPIFFS.h"

// Configuration
const char* ssid = "hel";
const char* password = "1234567890";
const char* serverUrl = "http://192.168.1.27:3000";

// Pin definitions
const int BUTTON_PIN = 0;    // Boot button
const int LED_PIN = 2;       // Built-in LED
const int I2S_BCK = 14;      // Bit Clock
const int I2S_LCK = 15;      // Word Select (LCK)
const int I2S_DIN = 32;      // Data In
const int RECORD_TIME = 5;    // Recording time in seconds

// I2S configuration
i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    .use_apll = true
};

i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK,
    .ws_io_num = I2S_LCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DIN
};

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Initialize SPIFFS
    if(!SPIFFS.begin(true)){
        Serial.println("SPIFFS Mount Failed");
        return;
    }

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(500);
        Serial.print(".");
    }
    digitalWrite(LED_PIN, HIGH);
    Serial.println("\nWiFi connected");

    // Initialize I2S
    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Failed installing driver: %d\n", err);
        while (true);
    }
    err = i2s_set_pin(I2S_NUM_0, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Failed setting pin: %d\n", err);
        while (true);
    }
}

void loop() {
    if (digitalRead(BUTTON_PIN) == LOW) {
        digitalWrite(LED_PIN, HIGH);
        recordAndSend();
        digitalWrite(LED_PIN, LOW);
        delay(1000); // Debounce
    }
}

void recordAndSend() {
    const int bufferLen = 1024;
    int32_t audioBuffer[bufferLen];
    File audioFile = SPIFFS.open("/recording.raw", "w");
    
    Serial.println("Recording...");
    
    // Record audio
    unsigned long startTime = millis();
    while ((millis() - startTime) < (RECORD_TIME * 1000)) {
        size_t bytesRead = 0;
        esp_err_t result = i2s_read(I2S_NUM_0, audioBuffer, bufferLen, &bytesRead, portMAX_DELAY);
        
        if (result == ESP_OK && bytesRead > 0) {
            audioFile.write((uint8_t*)audioBuffer, bytesRead);
        }
    }
    
    audioFile.close();
    Serial.println("Recording finished");

    // Send to API
    if(WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(serverUrl);
        
        File file = SPIFFS.open("/recording.raw", "r");
        
        // Create multipart form data
        String boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
        String head = "--" + boundary + "\r\nContent-Disposition: form-data; name=\"audio\"; filename=\"audio.raw\"\r\nContent-Type: audio/raw\r\n\r\n";
        String tail = "\r\n--" + boundary + "--\r\n";
        
        http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
        
        uint32_t fileSize = file.size();
        http.setConnectTimeout(20000); // 20s timeout
        
        WiFiClient* client = http.getStreamPtr();
        client->print(head);
        
        // Send file in chunks
        uint8_t buf[1024];
        while(file.available()) {
            int c = file.read(buf, sizeof(buf));
            client->write(buf, c);
        }
        
        client->print(tail);
        
        int httpCode = http.GET(); // Trigger the sending
        if (httpCode > 0) {
            String response = http.getString();
            Serial.println("Response: " + response);
        } else {
            Serial.println("Error on sending POST: " + String(httpCode));
        }
        
        http.end();
        file.close();
    }
}