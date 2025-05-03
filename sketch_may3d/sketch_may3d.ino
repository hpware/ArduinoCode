/*
  * Origin: https://www.instructables.com/ESP32-Mic-Testing-With-INMP441-and-DumbDisplay/
  * AI Chat: https://t3.chat/chat/8b05176e-5139-4266-93ca-87f3d6a477da
  * This document is AI generated. But it works, Diagram: https://content.instructables.com/FG0/7OW9/LDA8Z9XU/FG07OW9LDA8Z9XU.png
*/

#include <driver/i2s.h>
  #define I2S_WS               25
  #define I2S_SD               33
  #define I2S_SCK              32
  #define I2S_SAMPLE_BIT_COUNT 16
  #define SOUND_SAMPLE_RATE    8000
  #define SOUND_CHANNEL_COUNT  1
  #define I2S_PORT             I2S_NUM_0

// --- I2S Configuration Constants ---
const int I2S_DMA_BUF_COUNT = 8;
const int I2S_DMA_BUF_LEN = 1024;

#if I2S_SAMPLE_BIT_COUNT == 32
  const int StreamBufferNumBytes = 512; // Bytes to read at a time
  const int StreamBufferLen = StreamBufferNumBytes / 4; // Number of 32-bit samples
  int32_t StreamBuffer[StreamBufferLen]; // Buffer for 32-bit samples
#else // Assuming 16-bit
  #if SOUND_SAMPLE_RATE == 16000
    // 16 bits, 16000 Hz: 512 bytes = 16 ms per read
    const int StreamBufferNumBytes = 512;
  #else // Assuming 8000 Hz
    // 16 bits, 8000 Hz: 256 bytes = 16 ms per read
    const int StreamBufferNumBytes = 256;
  #endif
  const int StreamBufferLen = StreamBufferNumBytes / 2; // Number of 16-bit samples
  int16_t StreamBuffer[StreamBufferLen]; // Buffer for 16-bit samples
#endif
// --- End I2S Configuration Constants ---


// --- I2S Setup Functions ---
esp_err_t i2s_install() {
  uint32_t mode = I2S_MODE_MASTER | I2S_MODE_RX;
#if I2S_SCK == I2S_PIN_NO_CHANGE
    // Enable PDM mode if SCK is not used (typical for PDM microphones)
    mode |= I2S_MODE_PDM;
#endif
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(mode),
    .sample_rate = SOUND_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BIT_COUNT),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Adjust if using stereo
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
#if defined(FOR_ESP_SPARKBOT)  // Specific flags possibly for older IDF versions
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM,
    .dma_buf_count = 3, // Different DMA buffer count for this board
    .dma_buf_len = 1024,
    .use_apll = true // Use APLL for higher clock accuracy if needed
#else
    .intr_alloc_flags = 0, // Default interrupt flags
    .dma_buf_count = I2S_DMA_BUF_COUNT,
    .dma_buf_len = I2S_DMA_BUF_LEN,
    .use_apll = false // Default: don't use APLL
#endif
  };
  return i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

esp_err_t i2s_setpin() {
  const i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_PIN_NO_CHANGE, // Master Clock not usually needed for MEMS mics
    .bck_io_num = I2S_SCK,           // Bit Clock (Serial Clock)
    .ws_io_num = I2S_WS,            // Word Select (Left/Right Clock)
    .data_out_num = I2S_PIN_NO_CHANGE, // No data output needed for microphone
    .data_in_num = I2S_SD             // Serial Data input
  };
  return i2s_set_pin(I2S_PORT, &pin_config);
}
// --- End I2S Setup Functions ---


void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial connection (optional)

  Serial.println("Setting up I2S...");

  // Install and start I2S driver
  if (i2s_install() != ESP_OK) {
    Serial.println("Error installing I2S driver.");
    while (1); // Halt execution
  }

  // Set I2S pin configuration
  if (i2s_setpin() != ESP_OK) {
    Serial.println("Error setting I2S pins.");
    while (1); // Halt execution
  }

  // Zero the DMA buffer (good practice)
  if (i2s_zero_dma_buffer(I2S_PORT) != ESP_OK) {
    Serial.println("Error zeroing I2S DMA buffer.");
    // Continue execution, might not be critical
  }

   // Start I2S driver
  if (i2s_start(I2S_PORT) != ESP_OK) {
    Serial.println("Error starting I2S driver.");
     while (1); // Halt execution
  }

  Serial.println("I2S setup complete.");
}


void loop() {
  // Buffer to hold read data
  size_t bytesRead = 0;

  // Read data from I2S bus
  esp_err_t result = i2s_read(I2S_PORT,
                              &StreamBuffer,        // Buffer to read into
                              StreamBufferNumBytes, // Max bytes to read
                              &bytesRead,           // Actual bytes read
                              portMAX_DELAY);       // Wait indefinitely for data

  if (result == ESP_OK) {
    if (bytesRead > 0) {
      int samplesRead = 0;
#if I2S_SAMPLE_BIT_COUNT == 32
      samplesRead = bytesRead / 4; // 4 bytes per 32-bit sample
      // Optional: Process 32-bit samples if needed (e.g., scale down)
      // Example: Find average magnitude
      int64_t sum_abs = 0;
      for (int i = 0; i < samplesRead; i++) {
          // Extract upper 16 bits as an approximation for magnitude
          sum_abs += abs((int16_t)(StreamBuffer[i] >> 16));
      }
      float avg_magnitude = (float)sum_abs / samplesRead;
      Serial.print("Read ");
      Serial.print(samplesRead);
      Serial.print(" (32-bit) samples. Avg Mag: ");
      Serial.println(avg_magnitude);

#else // Assuming 16-bit
      samplesRead = bytesRead / 2; // 2 bytes per 16-bit sample
      // Optional: Process 16-bit samples
      // Example: Find average value
      int32_t sum = 0;
      for (int i = 0; i < samplesRead; i++) {
        sum += StreamBuffer[i];
      }
      float meanVal = (float)sum / samplesRead;
      Serial.print("Read ");
      Serial.print(samplesRead);
      Serial.print(" (16-bit) samples. Mean value: ");
      Serial.println(meanVal);
#endif
    } else {
      // Serial.println("Read 0 bytes."); // Can be noisy
    }
  } else {
    Serial.print("Error reading from I2S: ");
    Serial.println(esp_err_to_name(result));
  }

  // Optional small delay to prevent overwhelming Serial output
  delay(10);
}
