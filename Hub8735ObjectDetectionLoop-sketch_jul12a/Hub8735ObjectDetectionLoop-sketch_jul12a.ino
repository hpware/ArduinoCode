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
#define CHANNELNN 3 // Channel used for NN input
#define NNWIDTH 576
#define NNHEIGHT 320

VideoSetting config(VIDEO_FHD, 30, VIDEO_H264, 0);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0); // RGB channel for NN input
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

// This function attempts to get the image frame that was just processed by the NN.
// This is a common pattern in SDKs where the NN module holds a copy of its input.
// You MUST verify the actual function name and return types in your Realtek SDK.
String EncodeBase64ImageFile() {
  uint8_t *frameBuffer = nullptr;
  size_t frameLen = 0;

  // HYPOTHETICAL: Replace with actual SDK call if it exists.
  // This is the most likely way to get the frame that the NN is working with.
  // Example: ObjDet.getProcessedInputFrame(&frameBuffer, &frameLen);
  // Example: frameBuffer = ObjDet.getLatestInputFrame(); frameLen = ObjDet.getInputFrameSize();
  // Example: NNObjectDetection provides an internal buffer accessible like: ObjDet.input_buffer_addr, ObjDet.input_buffer_len
  
  // As a fallback for the example, we'll keep the Camera.getImage call, but it's noted as problematic.
  // If no ObjDet method exists, you'd need to consider a separate video channel or deeper SDK integration.
  Camera.getImage(CHANNELNN, (uint32_t*)&frameBuffer, (uint32_t*)&frameLen); // Cast for compatibility with uint32_t*

  Serial.println("Encoding image to Base64...");
  Serial.print("Image address: ");
  Serial.println((uint32_t)frameBuffer, HEX); // Cast for printing
  Serial.print("Image length: ");
  Serial.println(frameLen);

  if (frameBuffer == nullptr || frameLen == 0) {
    Serial.println("Error: Could not retrieve image data from CHANNELNN or NN module. Returning empty string.");
    return ""; // Return an empty string or handle error appropriately
  }

  // --- Base64 Encoding Logic ---
  // Assuming base64_enc_len(input_length) calculates required output buffer size
  // and base64_encode(output_buffer, input_buffer, input_length) encodes
  // the entire buffer. If not, you need to implement a correct chunking loop.

  // Determine the output buffer size needed for Base64
  // The base64_enc_len(3) in your original code only gives size for 3 bytes.
  // We need space for the entire encoded image.
  size_t outputBufferLen = base64_enc_len(frameLen);
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
  
  // Option 1: If your Base64.h provides a full buffer encoding function:
  // base64_encode(outputBuffer, (char*)frameBuffer, frameLen); // Example usage
  // String imgFileInBase64 = String(outputBuffer);

  // Option 2: If your Base64.h only provides base64_encode(output, input, 3) (as implied by previous code):
  String imgFileInBase64 = "";
  char chunkOutput[5]; // 4 base64 chars + null terminator for a 3-byte input chunk
  for (size_t i = 0; i < frameLen; i += 3) {
      int bytesToEncode = std::min((size_t)3, frameLen - i); // Encode 3 bytes or less if at end
      if (bytesToEncode <= 0) break; // Should not happen with correct loop conditions

      // Ensure base64_encode can handle fewer than 3 bytes for the last chunk
      base64_encode(chunkOutput, (char*)(frameBuffer + i), bytesToEncode);
      imgFileInBase64 += String(chunkOutput);
  }

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
  Camera.configVideoChannel(CHANNELNN, configNN); // Configure the RGB channel for NN
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

  // These dimensions should ideally match the OSD channel config,
  // which is CHANNEL (FHD), not NNWIDTH/NNHEIGHT (RGB for NN).
  // If you intend to draw on the NN frame before encoding, you'd use NNWIDTH/NNHEIGHT.
  // For OSD on RTSP (CHANNEL), use config.width()/height().
  uint16_t im_h = config.height(); // OSD is configured for CHANNEL 0 (FHD)
  uint16_t im_w = config.width();   // OSD is configured for CHANNEL 0 (FHD)

  OSD.createBitmap(CHANNEL); // Create bitmap for OSD on CHANNEL 0

  if (ObjDet.getResultCount() > 0) {
    // Call the Base64 encoding function only when objects are detected
    // This will try to get the image that the NN processed.
    Serial.println(EncodeBase64ImageFile());

    // --- Uncomment and adapt this section to draw bounding boxes on the RTSP stream ---
    // Make sure the OSD coordinates are scaled correctly if results are from a different resolution NN
    /*
    for (int i = 0; i < ObjDet.getResultCount(); i++) {
      int obj_type = results[i].type();
      // Assume itemList[obj_type].filter exists and is correct from ObjectClassList.h
      // Check if item should be ignored or if you want to draw all detections
      // if (itemList[obj_type].filter) {

        ObjectDetectionResult item = results[i];
        // Result coordinates are floats ranging from 0.00 to 1.00 relative to NNWIDTH/NNHEIGHT
        // Scale them to the OSD resolution (im_w, im_h from config - FHD)
        int xmin = (int)(item.xMin() * im_w);
        int xmax = (int)(item.xMax() * im_w);
        int ymin = (int)(item.yMin() * im_h);
        int ymax = (int)(item.yMax() * im_h);

        // Draw boundary box on CHANNEL 0 (RTSP stream)
        OSD.drawRect(CHANNEL, xmin, ymin, xmax, ymax, 3, OSD_COLOR_WHITE);

        // Print identification text (ensure text_str is populated with actual class name)
        // char text_str[20];
        // snprintf(text_str, sizeof(text_str), "%s (%.2f)", itemList[obj_type].name, item.score());
        // OSD.drawText(CHANNEL, xmin, ymin - OSD.getTextHeight(CHANNEL), text_str, OSD_COLOR_CYAN);
      // }
    }
    */
  }
  OSD.update(CHANNEL); // Update the OSD on CHANNEL 0

  // delay to wait for new results
  delay(100);
}