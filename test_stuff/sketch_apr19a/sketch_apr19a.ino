#include <driver/i2s.h>
#include "FS.h"       // Filesystem support
#include "SD.h"       // SD card library (SPI)
#include "SPI.h"      // SPI library

// --- Target Check ---
#ifndef ESP32
#error "This code is intended for ESP32 only."
#endif

// --- SD Card Configuration ---
#define SD_CS_PIN    5 // Chip Select pin for SD card
const char* wavFilename = "/

.wav"; // File path on SD card

// --- I2S Configuration ---
// Replace with your actual I2S GPIO pin numbers
#define I2S_SCK_PIN  14 // Serial Clock (SCK/BCLK)
#define I2S_WS_PIN   15 // Word Select (WS/LRCLK/FS)
#define I2S_SD_PIN   32 // Serial Data (SD/DOUT)

#define I2S_PORT     I2S_NUM_0

// I2S Settings - Read 32 bits per sample (common for I2S mics on ESP32)
const int             SAMPLE_RATE = 16000;
const i2s_bits_per_sample_t BITS_PER_SAMPLE_I2S = I2S_BITS_PER_SAMPLE_32BIT;
// We will write 16-bit data to the WAV file
const int             BITS_PER_SAMPLE_WAV = 16;
// Read stereo, even if mic is mono (driver often requires this)
const i2s_channel_fmt_t CHANNEL_FORMAT = I2S_CHANNEL_FMT_RIGHT_LEFT;
// We will write mono data to the WAV file
const int             NUM_CHANNELS_WAV = 1;

// --- Recording Configuration ---
const int RECORD_DURATION_SECONDS = 10; // How long to record

// --- Buffers ---
// Size of buffer for reading I2S data (samples * bytes/sample * channels)
// Must be multiple of 4 bytes for 32-bit samples
#define I2S_READ_BUFFER_SAMPLES 512
#define I2S_READ_BUFFER_BYTES (I2S_READ_BUFFER_SAMPLES * sizeof(int32_t) * 2) // *2 for stereo
int32_t i2s_read_buf[I2S_READ_BUFFER_SAMPLES * 2]; // Buffer for raw 32-bit stereo data

// Buffer for WAV data (16-bit mono)
#define WAV_WRITE_BUFFER_SAMPLES I2S_READ_BUFFER_SAMPLES
#define WAV_WRITE_BUFFER_BYTES (WAV_WRITE_BUFFER_SAMPLES * sizeof(int16_t) * NUM_CHANNELS_WAV)
int16_t wav_write_buf[WAV_WRITE_BUFFER_SAMPLES * NUM_CHANNELS_WAV];

// --- Global Variables ---
File file;
uint32_t totalDataSize = 0; // Total bytes of raw audio data written

// --- Function Prototypes ---
void setupI2S();
void setupSDCard();
void writeWavHeader(File f, uint32_t dataSize);
void finishRecording();

// ==========================================================
// Setup
// ==========================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n--- I2S Recorder ---");

  setupSDCard();
  setupI2S();

  // --- Create WAV File ---
  Serial.printf("Creating WAV file: %s\n", wavFilename);
  file = SD.open(wavFilename, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    while (true);
  }

  // Write initial WAV header (with placeholder sizes)
  writeWavHeader(file, 0); // Size 0 for now

  Serial.printf("Recording for %d seconds...\n", RECORD_DURATION_SECONDS);
}

// ==========================================================
// Main Loop
// ==========================================================
void loop() {
  static unsigned long recordingStartTime = millis();
  size_t bytes_read = 0;

  // --- Check Recording Duration ---
  if (millis() - recordingStartTime >= RECORD_DURATION_SECONDS * 1000) {
    finishRecording();
    // Loop indefinitely after finishing
    while (true) {
      delay(1000);
    }
  }

  // --- Read I2S Data ---
  esp_err_t result = i2s_read(I2S_PORT, i2s_read_buf, I2S_READ_BUFFER_BYTES,
                              &bytes_read, portMAX_DELAY); // Wait for data

  if (result == ESP_OK && bytes_read > 0) {
    int samples_read = bytes_read / (sizeof(int32_t) * 2); // Samples per channel

    // --- Process and Write Data ---
    int wav_buf_idx = 0;
    for (int i = 0; i < samples_read; i++) {
      // Extract the left channel's data (assuming L/R pin selects left)
      // I2S data is typically Left, Right, Left, Right...
      int32_t left_sample_32bit = i2s_read_buf[i * 2];

      // Convert 32-bit sample to 16-bit (take upper 16 bits)
      // This assumes the useful audio data is in the MSBs.
      int16_t sample_16bit = (int16_t)(left_sample_32bit >> 16);

      // Add to WAV buffer
      wav_write_buf[wav_buf_idx++] = sample_16bit;
    }

    // Write the processed 16-bit mono data to the SD card
    size_t bytes_written = file.write((const uint8_t*)wav_write_buf, wav_buf_idx * sizeof(int16_t));
    if (bytes_written == wav_buf_idx * sizeof(int16_t)) {
      totalDataSize += bytes_written;
    } else {
      Serial.println("Error writing to SD card!");
      finishRecording(); // Stop on error
      while (true);
    }

  } else if (result != ESP_OK) {
    Serial.printf("I2S Read Error: %d\n", result);
  }
}

// ==========================================================
// Finish Recording
// ==========================================================
void finishRecording() {
  Serial.println("\nRecording finished.");

  // --- Update WAV Header ---
  Serial.println("Updating WAV header...");
  writeWavHeader(file, totalDataSize);

  // --- Close File ---
  file.close();
  Serial.println("File closed.");
  Serial.printf("Total audio data written: %u bytes\n", totalDataSize);

  // --- Stop I2S ---
  i2s_driver_uninstall(I2S_PORT);
  Serial.println("I2S driver uninstalled.");
}


// ==========================================================
// Setup I2S
// ==========================================================
void setupI2S() {
  Serial.println("Configuring I2S...");

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = BITS_PER_SAMPLE_I2S,
    .channel_format = CHANNEL_FORMAT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = I2S_READ_BUFFER_SAMPLES, // Samples per DMA buffer
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
  };

  esp_err_t err;
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed installing I2S driver: %d\n", err);
    while (true);
  }
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed setting I2S pin: %d\n", err);
    while (true);
  }
  Serial.println("I2S driver installed.");
}

// ==========================================================
// Setup SD Card
// ==========================================================
void setupSDCard() {
  Serial.println("Initializing SD card...");
  // SPI.begin(SCK, MISO, MOSI, CS); // Optional: Define custom SPI pins if needed
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card Mount Failed!");
    Serial.println("Check connections, card formatting (FAT32), and CS pin.");
    while (true);
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    while (true);
  }
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) Serial.println("MMC");
  else if (cardType == CARD_SD) Serial.println("SDSC");
  else if (cardType == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("UNKNOWN");

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

// ==========================================================
// Write WAV Header
// ==========================================================
void writeWavHeader(File f, uint32_t dataSize) {
  // Calculate header values
  uint32_t fileSize = dataSize + 36; // dataSize + size of header (excluding RIFF ID and file size)
  uint32_t byteRate = SAMPLE_RATE * NUM_CHANNELS_WAV * BITS_PER_SAMPLE_WAV / 8;
  uint16_t blockAlign = NUM_CHANNELS_WAV * BITS_PER_SAMPLE_WAV / 8;
  uint16_t bitsPerSample = BITS_PER_SAMPLE_WAV;
  uint16_t numChannels = NUM_CHANNELS_WAV;
  uint16_t audioFormat = 1; // 1 = PCM (uncompressed)

  // RIFF chunk descriptor
  byte header[44];
  header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
  header[4] = (byte)(fileSize & 0xFF);          // File size - 8 (lower byte)
  header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF);
  header[7] = (byte)((fileSize >> 24) & 0xFF);  // File size - 8 (upper byte)
  header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';

  // fmt sub-chunk
  header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' '; // Subchunk1ID
  header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0; // Subchunk1Size (16 for PCM)
  header[20] = (byte)(audioFormat & 0xFF);      // AudioFormat (lower byte)
  header[21] = (byte)((audioFormat >> 8) & 0xFF); // AudioFormat (upper byte)
  header[22] = (byte)(numChannels & 0xFF);      // NumChannels (lower byte)
  header[23] = (byte)((numChannels >> 8) & 0xFF); // NumChannels (upper byte)
  header[24] = (byte)(SAMPLE_RATE & 0xFF);      // SampleRate (lower byte)
  header[25] = (byte)((SAMPLE_RATE >> 8) & 0xFF);
  header[26] = (byte)((SAMPLE_RATE >> 16) & 0xFF);
  header[27] = (byte)((SAMPLE_RATE >> 24) & 0xFF); // SampleRate (upper byte)
  header[28] = (byte)(byteRate & 0xFF);         // ByteRate (lower byte)
  header[29] = (byte)((byteRate >> 8) & 0xFF);
  header[30] = (byte)((byteRate >> 16) & 0xFF);
  header[31] = (byte)((byteRate >> 24) & 0xFF); // ByteRate (upper byte)
  header[32] = (byte)(blockAlign & 0xFF);       // BlockAlign (lower byte)
  header[33] = (byte)((blockAlign >> 8) & 0xFF);  // BlockAlign (upper byte)
  header[34] = (byte)(bitsPerSample & 0xFF);    // BitsPerSample (lower byte)
  header[35] = (byte)((bitsPerSample >> 8) & 0xFF); // BitsPerSample (upper byte)

  // data sub-chunk
  header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a'; // Subchunk2ID
  header[40] = (byte)(dataSize & 0xFF);         // Subchunk2Size (lower byte)
  header[41] = (byte)((dataSize >> 8) & 0xFF);
  header[42] = (byte)((dataSize >> 16) & 0xFF);
  header[43] = (byte)((dataSize >> 24) & 0xFF); // Subchunk2Size (upper byte)

  // Write header to file
  f.seek(0); // Go to the beginning of the file
  f.write(header, 44);
}
