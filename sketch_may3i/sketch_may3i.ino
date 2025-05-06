/*
  * AI generated V3
  * V1: https://github.com/hpware/ArduinoCode/blob/main/sketch_may3d/sketch_may3d.ino (Also AI generated)
  * V2: https://github.com/hpware/ArduinoCode/blob/main/sketch_may3d/sketch_may3e.ino (Also AI generated)
  * Origin: https://www.instructables.com/ESP32-Mic-Testing-With-INMP441-and-DumbDisplay/ <- Give it a look. It's a fine tutorial.
  * AI Chat: https://t3.chat/chat/8b05176e-5139-4266-93ca-87f3d6a477da
  * This code is AI generated. But it works, Diagram: https://content.instructables.com/FG0/7OW9/LDA8Z9XU/FG07OW9LDA8Z9XU.png
*/

#include <Arduino.h>


#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <driver/i2s.h>

#define SD_CS_PIN 5
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define I2S_SAMPLE_BIT_COUNT 16
#define SOUND_SAMPLE_RATE 8000
#define SOUND_CHANNEL_COUNT 1
#define I2S_PORT I2S_NUM_0

const int I2S_DMA_BUF_COUNT = 8;
const int I2S_DMA_BUF_LEN = 1024;
const int StreamBufferNumBytes = 256;  // ~16 ms worth of data
const int StreamBufferLen = StreamBufferNumBytes / 2;  // Number of 16-bit raw samples
int16_t StreamBuffer[StreamBufferLen];                 // Buffer for raw 16-bit samples

const int RECORD_TIME_SECONDS = 10;
#define NUM_CHANNELS SOUND_CHANNEL_COUNT
const float GAIN_FACTOR = 7.943;


const uint16_t WAV_BITS_PER_SAMPLE = 16; 
typedef struct {
  char RIFF[4] = { 'R', 'I', 'F', 'F' };
  uint32_t ChunkSize;
  char WAVE[4] = { 'W', 'A', 'V', 'E' };
  char fmt[4] = { 'f', 'm', 't', ' ' };
  uint32_t Subchunk1Size = 16;
  uint16_t AudioFormat = 1;  // PCM
  uint16_t NumChannels = NUM_CHANNELS;
  uint32_t SampleRate = SOUND_SAMPLE_RATE;       // Use define from I2S block
  uint32_t ByteRate;                             // SampleRate * NumChannels * BitsPerSample/8
  uint16_t BlockAlign;                           // NumChannels * BitsPerSample/8
  uint16_t BitsPerSample = WAV_BITS_PER_SAMPLE;  // Always 16 for output WAV
  char Subchunk2ID[4] = { 'd', 'a', 't', 'a' };
  uint32_t Subchunk2Size;  // Data size
} WavHeader;

File audioFile;
bool isRecording = false;
uint32_t totalBytesWritten = 0;

esp_err_t i2s_install();
esp_err_t i2s_setpin();
void writeWavHeader(File file, uint32_t dataSize);
void updateWavHeader(File file, uint32_t totalDataSize);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;


  Serial.println("Initializing SD card via SPI...");
  Serial.printf("Using SPI pins: MOSI=%d, MISO=%d, SCK=%d, CS=%d\n",
                MOSI, MISO, SCK, SD_CS_PIN);

  SPI.begin(SCK, MISO, MOSI, SD_CS_PIN);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card Mount Failed. Check wiring, power, formatting. Halting.");
    while (1)
      ;
  }
  Serial.println("SD Card initialized successfully.");




  Serial.println("Initializing I2S driver...");
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
  Serial.println("I2S driver started successfully.");


  String filename = "temp" + ".wav";
  String audioFileName;
  audioFileName = "/" + filename;
  Serial.print("Opening file on SD card for writing: ");
  Serial.println(audioFileName);

  if (SD.exists(audioFileName)) {
    Serial.println("Deleting existing recording file...");
    SD.remove(audioFileName);
  }

  audioFile = SD.open(audioFileName, FILE_WRITE);
  if (!audioFile) {
    Serial.println("Failed to open file on SD card for writing. Halting.");
    while (1)
      ;
  }

  Serial.println("Writing initial 16-bit WAV header...");
  writeWavHeader(audioFile, 0);

  totalBytesWritten = 0;

  Serial.printf("Header written. Recording for %d seconds...\n", RECORD_TIME_SECONDS);
  isRecording = true;
}




void loop() {
  if (!isRecording) {
    delay(1000);
    return;
  }

  static unsigned long recordingStartTime = millis();

  // --- Check if recording time is up ---
  if (millis() - recordingStartTime >= (RECORD_TIME_SECONDS * 1000)) {
    Serial.println("\nRecording finished.");
    isRecording = false;
    Serial.println("Attempting to update WAV header...");
    updateWavHeader(audioFile, totalBytesWritten);
    Serial.println("Flushing file buffer...");
    audioFile.flush();
    audioFile.close();
    Serial.println("SD card file closed.");
    Serial.printf("Total audio data written: %u bytes\n", totalBytesWritten);
    Serial.println("Entering idle state. Reset device to record again.");
    while (1) { delay(1000); }
  }

  // --- Read I2S Data ---
  size_t bytesRead = 0;
  esp_err_t result = i2s_read(I2S_PORT,
                              &StreamBuffer,
                              StreamBufferNumBytes,
                              &bytesRead,
                              pdMS_TO_TICKS(100));

  // --- Process Data (Convert to 16-bit & Apply Gain) & Write to SD Card ---
  if (result == ESP_OK && bytesRead > 0) {
    int samplesRead = 0;
    int16_t sampleStreamBuffer[StreamBufferLen];  // Temp buffer for processed 16-bit data

#if I2S_SAMPLE_BIT_COUNT == 32
    samplesRead = bytesRead / 4;
    for (int i = 0; i < samplesRead; ++i) {
      // 1. Convert 32-bit raw sample to intermediate 16-bit
      int32_t val32 = StreamBuffer[i];
      int16_t val16_intermediate = (int16_t)(val32 >> 16);  // Basic shift

      // 2. Apply Gain (use float for intermediate calculation)
      float amplified_f = (float)val16_intermediate * GAIN_FACTOR;

      // 3. Clamp/Clip to 16-bit range
      if (amplified_f > 32767.0f) {
        amplified_f = 32767.0f;
      } else if (amplified_f < -32768.0f) {
        amplified_f = -32768.0f;
      }

      // 4. Store final 16-bit value
      sampleStreamBuffer[i] = (int16_t)amplified_f;
    }
#else  // Raw data is already 16-bit
    samplesRead = bytesRead / 2;
    for (int i = 0; i < samplesRead; ++i) {
      // 1. Get raw 16-bit sample
      int16_t val16_raw = StreamBuffer[i];

      // 2. Apply Gain (use float for intermediate calculation)
      float amplified_f = (float)val16_raw * GAIN_FACTOR;

      // 3. Clamp/Clip to 16-bit range
      if (amplified_f > 32767.0f) {
        amplified_f = 32767.0f;
      } else if (amplified_f < -32768.0f) {
        amplified_f = -32768.0f;
      }

      // 4. Store final 16-bit value
      sampleStreamBuffer[i] = (int16_t)amplified_f;
    }
#endif

    // Write the processed (and amplified) 16-bit data to the SD card file
    size_t bytesToWrite = samplesRead * sizeof(int16_t);
    size_t bytesWritten = audioFile.write((const uint8_t*)sampleStreamBuffer, bytesToWrite);

    if (bytesWritten != bytesToWrite) {
      Serial.println("\nSD Card Write Error! Stopping recording.");
      isRecording = false;
      updateWavHeader(audioFile, totalBytesWritten);
      audioFile.flush();
      audioFile.close();
    } else {
      totalBytesWritten += bytesWritten;
      Serial.print(".");
    }
  } else if (result != ESP_OK) {
    Serial.printf("\nI2S Read Error: %s\n", esp_err_to_name(result));
    Serial.println("Stopping recording.");
    isRecording = false;
    updateWavHeader(audioFile, totalBytesWritten);
    audioFile.flush();
    audioFile.close();
  }
}
// --- End Loop Function ---


// --- I2S Setup Functions (FROM DUMB DISPLAY EXAMPLE) ---
// (i2s_install and i2s_setpin functions remain the same as previous version)
esp_err_t i2s_install() {
  uint32_t mode = I2S_MODE_MASTER | I2S_MODE_RX;
#if I2S_SCK == I2S_PIN_NO_CHANGE  // Specific check for PDM mics
  mode |= I2S_MODE_PDM;
#endif
  const i2s_config_t i2s_config_local = {  // Use local struct to avoid conflict
    .mode = i2s_mode_t(mode),
    .sample_rate = SOUND_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BIT_COUNT),  // Use raw bit count here
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
#if defined(FOR_ESP_SPARKBOT)  // Specific flags for this board/IDF version?
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM,
    .dma_buf_count = 3,
    .dma_buf_len = 1024,  // Note: Overrides global define for this board
    .use_apll = true
#else
    .intr_alloc_flags = 0,  // Default flags
    .dma_buf_count = I2S_DMA_BUF_COUNT,
    .dma_buf_len = I2S_DMA_BUF_LEN,
    .use_apll = false  // Default APLL usage
#endif
    // Ensure all members are initialized if using newer IDF versions
    // .tx_desc_auto_clear = false,
    // .fixed_mclk = 0
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
// --- End I2S Setup Functions ---


// --- WAV Header Functions ---
// (writeWavHeader and updateWavHeader functions remain the same as previous version)
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
  Serial.printf("Updating Header: totalDataSize=%u, chunkSize=%u, subchunk2Size=%u\n",
                totalDataSize, chunkSize, subchunk2Size);

  Serial.print("Seeking to offset 4 (ChunkSize)... ");
  if (!file.seek(4)) {
    Serial.println("FAILED.");
    file.seek(currentPos);
    return;
  }
  Serial.print("OK. Writing ChunkSize... ");
  size_t written = file.write((const uint8_t*)&chunkSize, sizeof(chunkSize));
  if (written != sizeof(chunkSize)) {
    Serial.println("FAILED.");
    file.seek(currentPos);
    return;
  }
  Serial.println("OK.");

  Serial.print("Seeking to offset 40 (Subchunk2Size)... ");
  if (!file.seek(40)) {
    Serial.println("FAILED.");
    file.seek(currentPos);
    return;
  }
  Serial.print("OK. Writing Subchunk2Size... ");
  written = file.write((const uint8_t*)&subchunk2Size, sizeof(subchunk2Size));
  if (written != sizeof(subchunk2Size)) {
    Serial.println("FAILED.");
    file.seek(currentPos);
    return;
  }
  Serial.println("OK.");

  Serial.print("Seeking back to position ");
  Serial.print(currentPos);
  Serial.print("... ");
  if (!file.seek(currentPos)) {
    Serial.println("FAILED.");
  } else {
    Serial.println("OK.");
  }
  Serial.println("Header update attempt finished.");
}
