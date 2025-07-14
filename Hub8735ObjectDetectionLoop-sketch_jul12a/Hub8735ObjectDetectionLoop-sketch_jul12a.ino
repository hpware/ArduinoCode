// DEVICE: HUB8735 ULTRA USING REALTEK RTL8735B
#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"
#include "NNObjectDetection.h"
#include "VideoStreamOverlay.h"
#include "ObjectClassList.h"

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

class Base64Encoder {
private:
  const char* base64_chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

public:
  int encodedLength(size_t length) {
    return 4 * ((length + 2) / 3);
  }

  void encode(char* output, const char* input, size_t length) {
    int i = 0, j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    while (length--) {
      char_array_3[i++] = *(input++);
      if (i == 3) {
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (i = 0; i < 4; i++)
          output[j++] = base64_chars[char_array_4[i]];
        i = 0;
      }
    }

    if (i) {
      for (int k = i; k < 3; k++)
        char_array_3[k] = '\0';

      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);

      for (int k = 0; k < i + 1; k++)
        output[j++] = base64_chars[char_array_4[k]];

      while (i++ < 3)
        output[j++] = '=';
    }
    output[j] = '\0';
  }
};

// Replace Base64 instance with Base64Encoder
Base64Encoder base64;

// Modify capImageBase64() function to use new encoder
String capImageBase64() {
  static unsigned long lastCapture = 0;
  const unsigned long CAPTURE_INTERVAL = 1000;  // Increase interval to 1 second

  if (millis() - lastCapture < CAPTURE_INTERVAL) {
    return "";
  }

  uint32_t addr = 0;
  uint32_t len = 0;

  // Try to capture without stopping channel
  Serial.println("Attempting direct capture...");
  Camera.getImage(CHANNEL, &addr, &len);

  if (!addr || len == 0) {
    Serial.println("Direct capture failed, trying with channel stop...");

    // Stop streaming
    Camera.channelEnd(CHANNEL);
    delay(200);  // Increase delay

    // Clear any pending commands
    Camera.videoInit();
    delay(100);

    // Try capture again
    Camera.getImage(CHANNEL, &addr, &len);

    // Restart streaming regardless of result
    Camera.channelBegin(CHANNEL);

    if (!addr || len == 0) {
      Serial.println("Both capture attempts failed");
      return "";
    }
  }

  Serial.print("Capture success - length: ");
  Serial.println(len);

  String base64Image = "";
  int encodedLen = base64.encodedLength(len);
  char* encodedBuffer = new char[encodedLen + 1];

  if (encodedBuffer) {
    base64.encode(encodedBuffer, (char*)addr, len);
    base64Image = String(encodedBuffer);
    delete[] encodedBuffer;
    Serial.print("Encoded length: ");
    Serial.println(base64Image.length());
  }

  lastCapture = millis();
  return base64Image;
}

void loop() {
  std::vector<ObjectDetectionResult> results = ObjDet.getResult();
  int objCount = ObjDet.getResultCount();

  if (objCount > 0) {
    static unsigned long lastDetection = 0;
    const unsigned long DETECTION_INTERVAL = 2000;  // Increase to 2 seconds

    Serial.print("Objects detected: ");
    Serial.println(objCount);

    if (millis() - lastDetection >= DETECTION_INTERVAL) {
      String base64Image = capImageBase64();
      if (base64Image.length() > 0) {
        Serial.println("Sending image data...");
        Serial2.print("FILE_");
        Serial2.println(base64Image);
        lastDetection = millis();
      }
    }

    drawDetectionBoxes(results);
  }

  OSD.update(CHANNEL);
  delay(200);  // Increase main loop delay
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial.println("Starting setup...");

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);

    // wait 2 seconds for connection:
    delay(2000);
  }
  ip = WiFi.localIP();
  // Camera setup with more explicit configuration
  config.setBitrate(2 * 1024 * 1024);
  config.setFPS(15);  // Reduce FPS
  config.setQuality(95);

  Camera.configVideoChannel(CHANNEL, config);
  Camera.configVideoChannel(CHANNELNN, configNN);

  if (Camera.videoInit()) {
    Serial.println("Camera initialized");
    delay(1000);  // Warm-up delay

    // Set additional camera parameters
    Camera.setImageQuality(CHANNEL, 95);
    Camera.setFrameRate(CHANNEL, 15);
    delay(500);

    Camera.channelBegin(CHANNEL);
    delay(500);
  } else {
    Serial.println("Camera init failed!");
    while (1) delay(1000);
  }

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
  Serial.println("--- Setup Complete ---");
  Serial.print("Resolution: ");
  Serial.print(NNWIDTH);
  Serial.print("x");
  Serial.println(NNHEIGHT);
  Serial.print("Video Format: RGB (");
  Serial.print(VIDEO_RGB);
  Serial.println(")");
  Serial.println("----------------------");

  Serial.println("Setup complete!");
}

void loop() {
  std::vector<ObjectDetectionResult> results = ObjDet.getResult();

  if (ObjDet.getResultCount() > 0) {
    static unsigned long lastDetection = 0;
    const unsigned long DETECTION_INTERVAL = 1000;

    Serial.print("Objects detected: ");
    Serial.println(ObjDet.getResultCount());

    if (millis() - lastDetection >= DETECTION_INTERVAL) {
      String base64Image = capImageBase64();
      if (base64Image.length() > 0) {
        Serial.println("Sending image data...");
        Serial2.print("FILE_");
        Serial2.println(base64Image);
        lastDetection = millis();
      }
    }

    drawDetectionBoxes(results);
  }

  OSD.update(CHANNEL);
  delay(100);
}

void drawDetectionBoxes(const std::vector<ObjectDetectionResult>& results) {
  uint16_t im_h = config.height();
  uint16_t im_w = config.width();

  OSD.createBitmap(CHANNEL);

  for (size_t i = 0; i < results.size(); i++) {
    // Create a non-const copy to work with the non-const methods
    ObjectDetectionResult item = results[i];
    int obj_type = item.type();

    if (itemList[obj_type].filter) {
      int xmin = (int)(item.xMin() * im_w);
      int xmax = (int)(item.xMax() * im_w);
      int ymin = (int)(item.yMin() * im_h);
      int ymax = (int)(item.yMax() * im_h);

      Serial.print("Object ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(itemList[obj_type].objectName);

      OSD.drawRect(CHANNEL, xmin, ymin, xmax, ymax, 3, OSD_COLOR_WHITE);

      char text_str[20];
      snprintf(text_str, sizeof(text_str), "%s %d",
               itemList[obj_type].objectName,
               item.score());
      OSD.drawText(CHANNEL, xmin,
                   ymin - OSD.getTextHeight(CHANNEL),
                   text_str, OSD_COLOR_CYAN);
    }
  }
}
