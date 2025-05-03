/*
  * Origin: https://www.instructables.com/ESP32-Mic-Testing-With-INMP441-and-DumbDisplay/
  * AI Chat: https://t3.chat/chat/8b05176e-5139-4266-93ca-87f3d6a477da
  * This document is AI generated. But it works, Diagram: https://content.instructables.com/FG0/7OW9/LDA8Z9XU/FG07OW9LDA8Z9XU.png?auto=webp&frame=1&width=1024&height=1024&fit=bounds&md=MjAyMy0wMS0yNCAwNzo0OTozMy4w
*/

#include "FS.h"

#include "SD.h"
#include "SPI.h"
#define SD_CS_PIN 5
#include <driver/i2s.h>

#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define I2S_SAMPLE_BIT_COUNT 16
#define SOUND_SAMPLE_RATE 8000
#define I2S_PORT I2S_NUM_0


// --- I2S Configuration Constants ---
const int I2S_DMA_BUF_COUNT = 8;
const int I2S_DMA_BUF_LEN = 1024;  // Bytes per DMA buffer

#if I2S_SAMPLE_BIT_COUNT == 32
const int StreamBufferNumBytes = 512;                  // Bytes to read from I2S at a time
const int StreamBufferLen = StreamBufferNumBytes / 4;  // Number of 32-bit samples
int32_t StreamBuffer[StreamBufferLen];                 // Buffer for 32-bit samples
#else                                                  // Assuming 16-bit
#if SOUND_SAMPLE_RATE == 16000
const int StreamBufferNumBytes = 512;                  // 16 ms worth of data
#else  // Assuming 8000 Hz
const int StreamBufferNumBytes = 256;  // 16 ms worth of data
#endif
const int StreamBufferLen = StreamBufferNumBytes / 2;  // Number of 16-bit samples
int16_t StreamBuffer[StreamBufferLen];                 // Buffer for 16-bit samples
#endif
// --- End I2S Configuration Constants ---


// --- SD Card File Configuration ---
const char* audioFileName = "/audio_recording.raw";  // Filename on SD card
File audioFile;                                      // File object to handle the audio file
bool isRecording = false;                            // Flag to control recording state
// --- End SD Card File Configuration ---


// --- I2S Setup Functions (Keep these as they were) ---
esp_err_t i2s_install() {
  uint32_t mode = I2S_MODE_MASTER | I2S_MODE_RX;
#if I2S_SCK == I2S_PIN_NO_CHANGE
  mode |= I2S_MODE_PDM;
#endif
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(mode),
    .sample_rate = SOUND_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BIT_COUNT),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // Mono microphone
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
#if defined(FOR_ESP_SPARKBOT)
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM,
    .dma_buf_count = 3,
    .dma_buf_len = 1024,  // Note: This overrides I2S_DMA_BUF_LEN above for this board
    .use_apll = true
#else
    .intr_alloc_flags = 0,
    .dma_buf_count = I2S_DMA_BUF_COUNT,
    .dma_buf_len = I2S_DMA_BUF_LEN,
    .use_apll = false
#endif
  };
  return i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
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


// --- SD Card Initialization Function ---
bool initSDCard() {
  Serial.println("Initializing SD card...");

#ifdef USE_SD_MMC
  // Using SD_MMC Interface
  if (!SD_MMC.begin()) {  // Default uses /sdcard mount point, 1-bit mode
                          // if (!SD_MMC.begin("/sdcard", true)) { // Use this for 4-bit mode
    Serial.println("SD_MMC Card Mount Failed");
    return false;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD_MMC card attached");
    return false;
  }
  Serial.print("SD_MMC Card Type: ");
  if (cardType == CARD_MMC) Serial.println("MMC");
  else if (cardType == CARD_SD) Serial.println("SDSC");
  else if (cardType == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("UNKNOWN");
  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);

#else
// Using SPI Interface
#ifndef SD_CS_PIN
#error "SD_CS_PIN is not defined! Please define it for SPI connection."
#endif
  Serial.print("Using SPI pins: MOSI=");
  Serial.print(MOSI);
  Serial.print(", MISO=");
  Serial.print(MISO);
  Serial.print(", SCK=");
  Serial.print(SCK);
  Serial.print(", CS=");
  Serial.println(SD_CS_PIN);

  // Initialize SPI bus if not already done elsewhere
  // SPI.begin(SCK, MISO, MOSI, SD_CS_PIN); // Optional: Explicitly define SPI pins

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card Mount Failed using SPI");
    return false;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached (SPI)");
    return false;
  }
  Serial.print("SD Card Type (SPI): ");
  if (cardType == CARD_MMC) Serial.println("MMC");
  else if (cardType == CARD_SD) Serial.println("SDSC");
  else if (cardType == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("UNKNOWN");
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size (SPI): %lluMB\n", cardSize);
#endif

  return true;  // Initialization successful
}
// --- End SD Card Initialization Function ---


// --- Setup Function ---
void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;  // Wait for Serial connection (optional)
  Serial.println("\n--- I2S SD Card Recorder ---");

  // 1. Initialize I2S
  Serial.println("Setting up I2S...");
  if (i2s_install() != ESP_OK) {
    Serial.println("Error installing I2S driver. Halting.");
    while (1)
      ;
  }
  if (i2s_setpin() != ESP_OK) {
    Serial.println("Error setting I2S pins. Halting.");
    while (1)
      ;
  }
  if (i2s_zero_dma_buffer(I2S_PORT) != ESP_OK) {
    Serial.println("Warning: Failed to zero I2S DMA buffer.");
  }
  if (i2s_start(I2S_PORT) != ESP_OK) {
    Serial.println("Error starting I2S driver. Halting.");
    while (1)
      ;
  }
  Serial.println("I2S setup complete.");

  // 2. Initialize SD Card
  if (!initSDCard()) {
    Serial.println("SD Card initialization failed. Halting.");
    while (1)
      ;
  }

  // 3. Open File for Writing
  Serial.print("Opening file for writing: ");
  Serial.println(audioFileName);

#ifdef USE_SD_MMC
  // Check if file exists and delete it (optional, uncomment to overwrite)
  // if (SD_MMC.exists(audioFileName)) {
  //   Serial.println("File exists. Deleting old file...");
  //   SD_MMC.remove(audioFileName);
  // }
  audioFile = SD_MMC.open(audioFileName, FILE_WRITE);  // Open for writing
#else
  // Check if file exists and delete it (optional, uncomment to overwrite)
  // if (SD.exists(audioFileName)) {
  //   Serial.println("File exists. Deleting old file...");
  //   SD.remove(audioFileName);
  // }
  audioFile = SD.open(audioFileName, FILE_WRITE);  // Open for writing
#endif

  if (!audioFile) {
    Serial.print("Failed to open file ");
    Serial.print(audioFileName);
    Serial.println(" for writing. Halting.");
    while (1)
      ;
  }

  Serial.println("File opened successfully. Starting recording...");
  isRecording = true;  // Set the flag to start recording in loop()
}
// --- End Setup Function ---


// --- Loop Function ---
void loop() {
  if (!isRecording) {
    // If not recording (e.g., stopped manually or due to error), do nothing.
    delay(100);  // Prevent busy-waiting
    return;
  }

  // --- Read I2S Data ---
  size_t bytesRead = 0;
  esp_err_t result = i2s_read(I2S_PORT,
                              &StreamBuffer,         // Pointer to buffer
                              StreamBufferNumBytes,  // Max bytes to read in one go
                              &bytesRead,            // Actual bytes read
                              portMAX_DELAY);        // Wait indefinitely for data

  // --- Process and Write Data ---
  if (result == ESP_OK) {
    if (bytesRead > 0) {
      // Successfully read some data from I2S
      // Write the raw bytes directly to the SD card file
      size_t bytesWritten = audioFile.write((const uint8_t*)StreamBuffer, bytesRead);

      if (bytesWritten != bytesRead) {
        // Error handling: Did not write all the bytes read
        Serial.print("SD Write Error! Expected ");
        Serial.print(bytesRead);
        Serial.print(" bytes, but wrote ");
        Serial.println(bytesWritten);
        Serial.println("Stopping recording.");
        isRecording = false;  // Stop recording on error
        audioFile.close();    // Close the file
      } else {
        // Optional: Print status periodically
        // static unsigned long lastPrint = 0;
        // if (millis() - lastPrint > 1000) {
        //    Serial.printf("Read %d bytes, wrote %d bytes to SD.\n", bytesRead, bytesWritten);
        //    lastPrint = millis();
        // }
      }
    }
    // else: bytesRead is 0, means i2s_read timed out or returned no data (shouldn't happen with portMAX_DELAY unless stopped)
  } else {
    // Error reading from I2S
    Serial.print("I2S Read Error: ");
    Serial.println(esp_err_to_name(result));
    Serial.println("Stopping recording.");
    isRecording = false;  // Stop recording on error
    audioFile.close();    // Close the file
  }

  // --- Optional: Add a way to stop recording ---
  // For example, after a certain duration or file size, or via a button press.
  // Simple duration example (record for 30 seconds):
  /*
  const unsigned long maxRecordingMillis = 30000; // 30 seconds
  static unsigned long recordingStartTime = millis();
  if (isRecording && (millis() - recordingStartTime > maxRecordingMillis)) {
      Serial.println("Maximum recording time reached. Stopping recording.");
      isRecording = false;
      audioFile.close(); // IMPORTANT: Close the file to save data properly!
      Serial.println("File closed.");
      Serial.println("You can now remove the SD card.");
      // Optional: Enter a halt state
      // while(1) { delay(1000); }
  }
  */
}
// --- End Loop Function ---
