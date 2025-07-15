// DEVICE: HUB8735 ULTRA USING REALTEK RTL8735B
#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"
#include "NNObjectDetection.h"
#include "VideoStreamOverlay.h"
#include "ObjectClassList.h"
#include "Base64.h"


#define CHANNEL 0
#define CHANNELIMG 1
#define CHANNELNN 3
#define NNWIDTH 576
#define NNHEIGHT 320
#define CAMERA_INIT_RETRY 3
#define CAMERA_INIT_DELAY 1000
VideoSetting config(VIDEO_FHD, 30, VIDEO_H264, 0);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0);
NNObjectDetection ObjDet;
RTSP rtsp;
StreamIO videoStreamer(1, 1);
StreamIO videoStreamerNN(1, 1);
uint32_t img_addr = 0;
uint32_t img_len = 0;
// 設定
char ssid[] = "hel";         // SSID
char pass[] = "1234567890";  // 密碼
int status = WL_IDLE_STATUS;

IPAddress ip;
int rtsp_portnum;

String EncodeBase64ImageFile() {
    if (Camera.getImage(CHANNELIMG, &img_addr, &img_len) != 0) {
        return "";  // Return empty if getImage fails
    }

    if (img_addr == 0 || img_len == 0) {
        return "";  // Return empty if invalid image data
    }

    uint8_t *fbBuf = (uint8_t *)img_addr;
    size_t fbLen = img_len;
    String imgFileInBase64 = "data:image/jpeg;base64,";

    // Process in proper 3-byte chunks
    const size_t chunkSize = 3;
    char output[base64_enc_len(chunkSize) + 1];  // +1 for null terminator
    
    for (size_t i = 0; i < fbLen; i += chunkSize) {
        size_t remaining = ((fbLen - i) < chunkSize) ? (fbLen - i) : chunkSize;
        base64_encode(output, (char *)(fbBuf + i), remaining);
        imgFileInBase64 += output;
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

  // Configure camera with retry mechanism
  int retry = 0;
  bool camera_initialized = false;
  
  while (!camera_initialized && retry < CAMERA_INIT_RETRY) {
      Camera.videoInit();
      delay(CAMERA_INIT_DELAY);
      
      // Configure channels
      if (Camera.configVideoChannel(CHANNEL, config) == 0 &&
          Camera.configVideoChannel(CHANNELNN, configNN) == 0) {
          
          Camera.channelBegin(CHANNELIMG);
          Camera.printInfo();
          
          camera_initialized = true;
      } else {
          retry++;
          Serial.print("Camera init retry: ");
          Serial.println(retry);
      }
  }

  if (!camera_initialized) {
      Serial.println("Camera initialization failed!");
      while(1) { delay(1000); } // Halt execution
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
