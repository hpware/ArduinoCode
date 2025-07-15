// DEVICE: HUB8735 ULTRA USING REALTEK RTL8735B
#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"
#include "NNObjectDetection.h"
#include "VideoStreamOverlay.h"
#include "ObjectClassList.h"
#include "Base64.h" // Make sure this library provides a suitable base64_encode function for buffers.

#include <algorithm> // Required for std::min

#define CHANNEL 0
#define CHANNELNN 3
#define NNWIDTH 576
#define NNHEIGHT 320

VideoSetting config(VIDEO_FHD, 30, VIDEO_H264, 0);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0);
NNObjectDetection ObjDet;
RTSP rtsp;
StreamIO videoStreamer(1, 1);
StreamIO videoStreamerNN(1, 1);

// 設定
char ssid[] = "hel";         // SSID
char pass[] = "1234567890";  // 密碼
int status = WL_IDLE_STATUS;

IPAddress ip;
int rtsp_portnum;

String EncodeBase64ImageFile() {
  uint32_t addr = 0;
  uint32_t len = 0;
  // IMPORTANT FIX: Get image from CHANNELNN (RGB stream for NN)
  Camera.getImage(CHANNELNN, &addr, &len);
  Serial.println("Encoding image to Base64...");
  Serial.print("Image address: ");
  Serial.println(addr, HEX); // Print in hexadecimal for clarity
  Serial.print("Image length: ");
  Serial.println(len);

  if (addr == 0 || len == 0) {
    Serial.println("Error: Could not retrieve image data from CHANNELNN. Returning empty string.");
    return ""; // Return an empty string or handle error appropriately
  }

  uint8_t *fbBuf = (uint8_t *)addr;
  size_t fbLen = len;

  // Assuming base64_enc_len(input_length) calculates required output buffer size
  // and base64_encode(output_buffer, input_buffer, input_length) encodes
  // the entire buffer. If not, you need to implement a correct chunking loop.

  // The base64_enc_len(3) in your original code only gives size for 3 bytes.
  // We need space for the entire encoded image.
  size_t outputBufferLen = base64_enc_len(fbLen);
  // Allocate buffer on heap to handle potentially large images
  char *outputBuffer = new (std::nothrow) char[outputBufferLen + 1]; // +1 for null terminator

  if (outputBuffer == nullptr) {
    Serial.println("Error: Failed to allocate memory for Base64 output.");
    return "";
  }

  // Use the library's function to encode the whole buffer if available
  // Replace this with the correct function signature from your Base64.h
  // This is a placeholder for a typical full-buffer encode function.
  // If your library only provides a 3-byte chunk encoder, the loop below is needed.
  size_t actualEncodedLen = 0; // This might be returned by the base64_encode function

  // If your Base64.h only provides base64_encode(output, input, 3), use this loop:
  String imgFileInBase64 = "";
  char chunkOutput[5]; // 4 base64 chars + null terminator
  for (size_t i = 0; i < fbLen; i += 3) {
      int bytesToEncode = std::min((size_t)3, fbLen - i); // Encode 3 bytes or less if at end
      if (bytesToEncode <= 0) break; // Should not happen with correct loop conditions

      base64_encode(chunkOutput, (char*)(fbBuf + i), bytesToEncode);
      imgFileInBase64 += String(chunkOutput);
  }

  // If a full-buffer base64_encode function is available, use this instead of the loop above:
  /*
  base64_encode(outputBuffer, (char*)fbBuf, fbLen); // Assuming this function exists
  String imgFileInBase64 = String(outputBuffer);
  */

  delete[] outputBuffer; // Free the allocated memory
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
  config.setBitrate(2 * 1024 * 1024); // Recommend to use 2Mbps for RTSP streaming to prevent network congestion
  Camera.configVideoChannel(CHANNEL, config);
  Camera.configVideoChannel(CHANNELNN, configNN);
  Camera.printInfo();
  Camera.videoInit();

  // Configure RTSP with corresponding video format information
  rtsp.configVideo(config);
  rtsp.begin();
  rtsp_portnum = rtsp.getPort();

  // Configure object detection with corresponding video format information
  // Select Neural Network(NN) task and models
  ObjDet.configVideo(configNN);
  ObjDet.modelSelect(OBJECT_DETECTION, DEFAULT_YOLOV4TINY, NA_MODEL, NA_MODEL);
  ObjDet.begin();

  // Configure StreamIO object to stream data from video channel to RTSP
  videoStreamer.registerInput(Camera.getStream(CHANNEL));
  videoStreamer.registerOutput(rtsp);
  if (videoStreamer.begin() != 0) {
    Serial.println("StreamIO link start failed");
  }

  // Start data stream from video channel
  Camera.channelBegin(CHANNEL);

  // Configure StreamIO object to stream data from RGB video channel to object detection
  videoStreamerNN.registerInput(Camera.getStream(CHANNELNN));
  videoStreamerNN.setStackSize();
  videoStreamerNN.setTaskPriority();
  videoStreamerNN.registerOutput(ObjDet);
  if (videoStreamerNN.begin() != 0) {
    Serial.println("StreamIO link start failed");
  }

  // Start video channel for NN
  Camera.channelBegin(CHANNELNN);

  // Start OSD drawing on RTSP video channel
  OSD.configVideo(CHANNEL, config);
  OSD.begin();
}

void loop() {
  std::vector<ObjectDetectionResult> results = ObjDet.getResult();

  uint16_t im_h = config.height();
  uint16_t im_w = config.width();

  OSD.createBitmap(CHANNEL);

  if (ObjDet.getResultCount() > 0) {
    Serial.println(EncodeBase64ImageFile());
    /**
    for (int i = 0; i < ObjDet.getResultCount(); i++) {
      int obj_type = results[i].type();
      if (itemList[obj_type].filter) {  // check if item should be ignored

        ObjectDetectionResult item = results[i];
        // Result coordinates are floats ranging from 0.00 to 1.00
        // Multiply with RTSP resolution to get coordinates in pixels
        int xmin = (int)(item.xMin() * im_w);
        int xmax = (int)(item.xMax() * im_w);
        int ymin = (int)(item.yMin() * im_h);
        int ymax = (int)(item.yMax() * im_h);

        // Draw boundary box
        OSD.drawRect(CHANNEL, xmin, ymin, xmax, ymax, 3, OSD_COLOR_WHITE);

        // Print identification text
        char text_str[20];
        //OSD.drawText(CHANNEL, xmin, ymin - OSD.getTextHeight(CHANNEL), text_str, OSD_COLOR_CYAN);
      }
    } */
  }
  OSD.update(CHANNEL);

  // delay to wait for new results
  delay(100);
}