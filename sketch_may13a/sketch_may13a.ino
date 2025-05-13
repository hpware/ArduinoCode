/*
  * https://github.com/hpware/ArduinoCode/blob/main/sketch_may3d/sketch_may3d.ino (AI generated)
  * https://github.com/hpware/ArduinoCode/blob/main/sketch_may3d/sketch_may3e.ino (AI generated)
  * https://github.com/hpware/ArduinoCode/blob/main/sketch_may3d/sketch_may3i.ino (AI generated) 
  * https://www.instructables.com/ESP32-Mic-Testing-With-INMP441-and-DumbDisplay/
  * Part of this code is AI generated.
*/

#include <WiFi.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <FS.h>
#include "SD.h"
#include "SPI.h"
#include <driver/i2s.h>

// 創立一個 env.h 並使用 env-example.h 的模板

#define SD_CS_PIN 5
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32

// 設定 I2S
#define I2S_SAMPLE_BIT_COUNT 16
#define SOUND_SAMPLE_RATE 8000
#define SOUND_CHANNEL_COUNT 1
#define I2S_PORT I2S_NUM_0
const int I2S_DMA_BUF_COUNT = 8;
const int I2S_DMA_BUF_LEN = 1024;
const int StreamBufferNumBytes = 256;
const int StreamBufferLen = StreamBufferNumBytes / 2;
int16_t StreamBuffer[StreamBufferLen];
#define NUM_CHANNELS SOUND_CHANNEL_COUNT
const uint16_t WAV_BITS_PER_SAMPLE = 16;
typedef struct {
  char RIFF[4] = { 'R', 'I', 'F', 'F' };
  uint32_t ChunkSize;
  char WAVE[4] = { 'W', 'A', 'V', 'E' };
  char fmt[4] = { 'f', 'm', 't', ' ' };
  uint32_t Subchunk1Size = 16;
  uint16_t AudioFormat = 1;
  uint16_t NumChannels = NUM_CHANNELS;
  uint32_t SampleRate = SOUND_SAMPLE_RATE;
  uint32_t ByteRate;
  uint16_t BlockAlign;
  uint16_t BitsPerSample = WAV_BITS_PER_SAMPLE;
  char Subchunk2ID[4] = { 'd', 'a', 't', 'a' };
  uint32_t Subchunk2Size;
} WavHeader;
File audioFile;
bool isRecording = false;
uint32_t totalBytesWritten = 0;
esp_err_t i2s_install();
esp_err_t i2s_setpin();
void writeWavHeader(File file, uint32_t dataSize);
void updateWavHeader(File file, uint32_t totalDataSize);

// 錄音時間與音量放大
const int RECORD_TIME_SECONDS = 10;
const float GAIN_FACTOR = 7.943;



// Runner
TaskHandle_t MainTaskC;
TaskHandle_t Task2C;

// Values
bool REC_ON = false;
bool FINISHED_RECORDING = false;

// Test Button Config
#define TEST_BUTTON_PIN 13

// 初始化
void setup() {
  Serial.begin(115200);

  // Test Button Setup
  pinMode(TEST_BUTTON_PIN, INPUT_PULLUP);

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // SPI Setup (SD)
  SPI.begin(SCK, MISO, MOSI, SD_CS_PIN);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card Failed");
    while (!SD.begin(SD_CS_PIN)) {
      Serial.print(".");
      delay(100);
    };
    Serial.println();
  }
  Serial.println("SD Card Initialized");

  // I2S
  if (i2s_install() != ESP_OK) {
    Serial.println("XXX failed to install I2S driver. Halting.");
    while (1)
      ;
  }
  if (i2s_setpin() != ESP_OK) {
    Serial.println("XXX failed to set I2S pins. Halting.");
    while (1)
      ;
  }
  if (i2s_start(I2S_PORT) != ESP_OK) {
    Serial.println("XXX failed to start I2S driver. Halting.");
    while (1)
      ;
  }

  // Set Values
  REC_ON = false;
  FINISHED_RECORDING = false;

  // Runners
  xTaskCreatePinnedToCore(MainTask, "MainTask", 10000, NULL, 1, &MainTaskC, 0);
  xTaskCreatePinnedToCore(Task2, "Task2", 10000, NULL, 1, &Task2C, 0);
}

// 留空
void loop() {}

// 核心 1
void MainTask(void* pvParameters) {
  while (true) {
    unsigned long currentMillis = millis();
    if (digitalRead(TEST_BUTTON_PIN) == LOW && REC_ON == false) {
      Serial.println("REC ON");
      REC_ON = true;
    }
    vTaskDelay(10);
  }
}

// 核心 2
void Task2(void* pvParameters) {
  while (true) {
    unsigned long currentMillis = millis();
    if (REC_ON == true) {
      recordAudio();
      Serial.println("SENDING RECORDING");
      //sendAudio();
      REC_ON = false;
    }
    vTaskDelay(100);
  }
}

// 傳送音檔給 Groq
void sendAudio(const uint8_t* audioData, size_t audioDataSize, const char* audioContentType) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, aiChatUrl);
  http.addHeader("Content-Type", "application/json");

  // Create a temporary non-const buffer
  uint8_t* tempBuffer = new uint8_t[audioDataSize];
  memcpy(tempBuffer, audioData, audioDataSize);

  int httpResponseCode = http.POST(tempBuffer, audioDataSize);

  // Clean up
  delete[] tempBuffer;

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("Response Code: " + String(httpResponseCode));
    Serial.println(response);
  } else {
    Serial.printf("Error: %s\n", http.errorToString(httpResponseCode).c_str());
  }
  http.end();
}
// Record Audio Stuff
void recordAudio() {
    String filename = String(fileName + ".wav");
    String audioFileName = "/" + filename;
    SD.remove(audioFileName); // Force remove
    
    audioFile = SD.open(audioFileName, FILE_WRITE);
    if (!audioFile) {
        Serial.println("Failed to open file for writing");
        return;
    }
    
    Serial.println("Writing initial 16-bit WAV header...");
    writeWavHeader(audioFile, 0);
    
    totalBytesWritten = 0;
    isRecording = true;
    unsigned long recordingStartTime = millis();
    
    Serial.printf("Header written. Recording for %d seconds...\n", RECORD_TIME_SECONDS);
    
    while (isRecording) {
        vTaskDelay(pdMS_TO_TICKS(1));  // Removed stray backslash
        
        if (millis() - recordingStartTime >= (RECORD_TIME_SECONDS * 1000)) {
            break; 
        }
        
        size_t bytesRead = 0;
        esp_err_t result = i2s_read(I2S_PORT, &StreamBuffer, StreamBufferNumBytes, &bytesRead, pdMS_TO_TICKS(100));
        
        if (result == ESP_OK && bytesRead > 0) {
            int samplesRead = bytesRead / 2;  // 16-bit samples
            int16_t sampleStreamBuffer[128];  // Temporary buffer for processed samples
            
            // Process the audio samples
            for (int i = 0; i < samplesRead; ++i) {
                float amplified_f = (float)StreamBuffer[i] * GAIN_FACTOR;
                // Clip if necessary
                if (amplified_f > 32767.0f) amplified_f = 32767.0f;
                if (amplified_f < -32768.0f) amplified_f = -32768.0f;
                sampleStreamBuffer[i] = (int16_t)amplified_f;
            }
            
            // Write processed samples to file
            size_t bytesToWrite = samplesRead * sizeof(int16_t);
            size_t bytesWritten = audioFile.write((const uint8_t*)sampleStreamBuffer, bytesToWrite);
            
            if (bytesWritten != bytesToWrite) {
                Serial.println("\nSD Card Write Error!");
                break;
            }
            
            totalBytesWritten += bytesWritten;
            Serial.print(".");
        }
        
        // Feed watchdog timer
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // Cleanup
    Serial.println("\nRecording finished.");
    updateWavHeader(audioFile, totalBytesWritten);
    audioFile.flush();
    audioFile.close();
    Serial.printf("Total audio data written: %u bytes\n", totalBytesWritten);
}

esp_err_t i2s_install() {
  uint32_t mode = I2S_MODE_MASTER | I2S_MODE_RX;
  const i2s_config_t i2s_config_local = {
    .mode = i2s_mode_t(mode),
    .sample_rate = SOUND_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BIT_COUNT),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = I2S_DMA_BUF_COUNT,
    .dma_buf_len = I2S_DMA_BUF_LEN,
    .use_apll = false
  };
  return i2s_driver_install(I2S_PORT, &i2s_config_local, 0, NULL);
}

esp_err_t i2s_setpin() {
  const i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };
  return i2s_set_pin(I2S_PORT, &pin_config);
}

void writeWavHeader(File file, uint32_t dataSize) {
  WavHeader header;
  header.ByteRate = header.SampleRate * header.NumChannels * (header.BitsPerSample / 8);
  header.BlockAlign = header.NumChannels * (header.BitsPerSample / 8);
  header.Subchunk2Size = dataSize;
  header.ChunkSize = 36 + dataSize;
  file.write((const uint8_t*)&header, sizeof(header));
}

void updateWavHeader(File file, uint32_t totalDataSize) {
  if (!file || !file.available()) {
    Serial.println("Error: File not open or invalid for header update.");
    return;
  }
  uint32_t currentPos = file.position();
  uint32_t chunkSize = 36 + totalDataSize;
  uint32_t subchunk2Size = totalDataSize;
  if (!file.seek(4)) {
    Serial.println("FAILED.");
    file.seek(currentPos);
    return;
  }
  size_t written = file.write((const uint8_t*)&chunkSize, sizeof(chunkSize));
  if (written != sizeof(chunkSize)) {
    Serial.println("FAILED.");
    file.seek(currentPos);
    return;
  }
  Serial.println("OK.");
  if (!file.seek(40)) {
    Serial.println("FAILED.");
    file.seek(currentPos);
    return;
  }
  written = file.write((const uint8_t*)&subchunk2Size, sizeof(subchunk2Size));
  if (written != sizeof(subchunk2Size)) {
    Serial.println("FAILED.");
    file.seek(currentPos);
    return;
  }
  Serial.println("OK.");
  if (!file.seek(currentPos)) {
    Serial.println("FAILED.");
  } else {
    Serial.println("OK.");
  }
}