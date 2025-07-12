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
  
                  for(i = 0; i < 4; i++)
                      output[j++] = base64_chars[char_array_4[i]];
                  i = 0;
              }
          }
  
          if (i) {
              for(int k = i; k < 3; k++)
                  char_array_3[k] = '\0';
  
              char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
              char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
              char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
  
              for (int k = 0; k < i + 1; k++)
                  output[j++] = base64_chars[char_array_4[k]];
  
              while(i++ < 3)
                  output[j++] = '=';
          }
          output[j] = '\0';
      }
  };
  
  // Replace Base64 instance with Base64Encoder
  Base64Encoder base64;
  
  // Modify capImageBase64() function to use new encoder
  String capImageBase64() {
      String base64Image = "";
      uint32_t addr = 0;
      uint32_t len = 0;
      Camera.getImage(CHANNEL, &addr, &len);
  
      if (addr && len > 0) {
          uint8_t* imageData = (uint8_t*)addr;
          int encodedLen = base64.encodedLength(len);
          char* encodedBuffer = new char[encodedLen + 1];
          
          base64.encode(encodedBuffer, (char*)imageData, len);
          base64Image = String(encodedBuffer);
          
          delete[] encodedBuffer;
      } else {
          Serial.println("Failed to capture image");
      }
  
      return base64Image;
  }

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial2.println("OK");

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
  config.setBitrate(2 * 1024 * 1024);  // Recommend to use 2Mbps for RTSP streaming to prevent network congestion
  Camera.configVideoChannel(CHANNEL, config);
  Camera.configVideoChannel(CHANNELNN, configNN);
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

  Serial.print("rtsp://");
  Serial.print(ip);
  Serial.print(":");
  Serial.println(rtsp_portnum);
  Serial.println(" ");

  printf("Total number of objects detected = %d\r\n", ObjDet.getResultCount());
  OSD.createBitmap(CHANNEL);

  if (ObjDet.getResultCount() > 0) {
    String base64Image = capImageBase64();
    Serial2.print("FILE_");
    Serial2.println(base64Image);

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
        printf("Item %d %s:\t%d %d %d %d\n\r", i, itemList[obj_type].objectName, xmin, xmax, ymin, ymax);
        OSD.drawRect(CHANNEL, xmin, ymin, xmax, ymax, 3, OSD_COLOR_WHITE);

        // Print identification text
        char text_str[20];
        snprintf(text_str, sizeof(text_str), "%s %d", itemList[obj_type].objectName, item.score());
        OSD.drawText(CHANNEL, xmin, ymin - OSD.getTextHeight(CHANNEL), text_str, OSD_COLOR_CYAN);
      }
    }
  }
  OSD.update(CHANNEL);
  delay(100);
}
