// DEVICE: HUB8735 ULTRA USING REALTEK RTL8735B
#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"
#include "NNObjectDetection.h"
#include "VideoStreamOverlay.h"
#include "Base64.h"
#include <CameraLED.h>

#include <algorithm>  // Required for std::min

#define CHANNEL 0        // H264 RTSP Stream
#define CHANNEL_STILL 1  // NEW: Dedicated channel for still JPEG capture

#define LED_PWM 13


// VideoSetting for H264 RTSP stream
VideoSetting config(VIDEO_FHD, 30, VIDEO_H264, 0);
// NEW: VideoSetting for still JPEG capture
// Using a common still image resolution like 640x480 or 1280x720, adjusted for your sensor
// The '1' in the last parameter for VIDEO_JPEG often indicates a dedicated still capture mode.
VideoSetting configStill(1280, 720, 1, VIDEO_JPEG, 1);  // Example: 640x480 JPEG at 1 FPS for capture

RTSP rtsp;
CameraLED flashLight;
StreamIO videoStreamer(1, 1);

// 設定
char ssid[] = "hel";         // SSID
char pass[] = "1234567890";  // 密碼
int status = WL_IDLE_STATUS;

IPAddress ip;
int rtsp_portnum;

// Function to encode a captured image to Base64
String EncodeBase64ImageFile(uint32_t addr, uint32_t len) {
  Serial.println("Encoding image to Base64...");
  Serial.print("Image address: ");
  Serial.println(addr, HEX);
  Serial.print("Image length: ");
  Serial.println(len);

  if (addr == 0 || len == 0) {
    Serial.println("Error: No valid image data provided for Base64 encoding.");
    return "";
  }

  uint8_t *fbBuf = (uint8_t *)addr;
  size_t fbLen = len;

  // The base64_encode function in Base64.h often encodes 3 bytes into 4 chars.
  // The original loop (if (i%3==0) imageFile += String(output);) was problematic.
  // This revised loop correctly processes chunks of 3 bytes.
  String imgFileInBase64 = "data:image/jpeg;base64,";  // Prepend for web display (adjust mime type if not JPEG)
  char chunkOutput[5];                                 // 4 base64 chars + null terminator
  for (size_t i = 0; i < fbLen; i += 3) {
    int bytesToEncode = std::min((size_t)3, fbLen - i);
    if (bytesToEncode <= 0) break;  // Should not happen with correct loop conditions

    base64_encode(chunkOutput, (char *)(fbBuf + i), bytesToEncode);
    imgFileInBase64 += String(chunkOutput);
  }

  return imgFileInBase64;
}


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);

    // wait 2 seconds for connection:
    delay(2000);
  }
  ip = WiFi.localIP();

  // Configure camera video channels with video format information
  // Adjust the bitrate based on your WiFi network quality
  config.setBitrate(2 * 1024 * 1024);                     // Recommend to use 2Mbps for RTSP streaming to prevent network congestion
  Camera.configVideoChannel(CHANNEL, config);             // H264 stream for RTSP
  Camera.configVideoChannel(CHANNEL_STILL, configStill);  // NEW: Dedicated channel for still capture

  Camera.printInfo();
  Camera.videoInit();

  // Configure RTSP with corresponding video format information
  rtsp.configVideo(config);
  rtsp.begin();
  rtsp_portnum = rtsp.getPort();

  flashLight.attach(LED_PWM, 1, 2000);
  flashLight.writeMicroseconds(0);

  // Configure StreamIO object to stream data from video channel to RTSP
  videoStreamer.registerInput(Camera.getStream(CHANNEL));
  videoStreamer.registerOutput(rtsp);
  if (videoStreamer.begin() != 0) {
    Serial.println("StreamIO link start failed");
  }
  Camera.channelBegin(CHANNEL);  // Start H264 data stream

  // Start OSD drawing on RTSP video channel
  OSD.configVideo(CHANNEL, config);
  OSD.begin();

  // NEW: Start the dedicated still capture channel
  // Depending on SDK, you might need to call channelBegin() and channelStop() around getImage()
  // for a still capture channel if it's resource-intensive.
  Camera.channelBegin(CHANNEL_STILL);
}

void loop() {
  
  // Check if ESP32 sent a capture command
  if (Serial2.available()) {
    String command = Serial2.readStringUntil('\n');
    command.trim();
    
    // If the command is <CAPTURE />, trigger image capture
    if (command == "<!CAPTURE /!>") {
      Serial.println("Capture command received from ESP32");
      
      // Capture a still image from the dedicated channel
      uint32_t still_img_addr = 0;
      uint32_t still_img_len = 0;

      // Capture the image from the dedicated still channel
      Camera.getImage(CHANNEL_STILL, &still_img_addr, &still_img_len);

      // Pass the captured image data to the Base64 encoding function
      const String encodingProcess = EncodeBase64ImageFile(still_img_addr, still_img_len);
      Serial.println("<!START BLOCK!>");  // Start block for base64 data in case of esp32 just cutting off half of the base64 data.
      Serial.println(encodingProcess);
      Serial.println("<!END BLOCK!>");

      Serial2.print("<!START BLOCK!>");  // Start block for base64 data in case of esp32 just cutting off half of the base64 data.
      Serial2.print(encodingProcess);
      Serial2.println("</!END BLOCK!>");
    } else if (command.startsWith("<!FLASHLIGHT!>") && command.endsWith("</!FLASHLIGHT!>")) {
      int duration = command.substring(14, command.length() - 15).toInt();
      Serial.print("Flashlight command received for duration: ");
      Serial.println(duration);
      flashLight.writeMicroseconds(200 * duration);
    } else {
      Serial.println("Unknown command received: " + command);
    }
  }
  
  // Continue with regular OSD updates
  OSD.createBitmap(CHANNEL);  // Create bitmap for OSD on CHANNEL 0
  OSD.update(CHANNEL);  // Update the OSD on CHANNEL 0
  delay(10);
}
