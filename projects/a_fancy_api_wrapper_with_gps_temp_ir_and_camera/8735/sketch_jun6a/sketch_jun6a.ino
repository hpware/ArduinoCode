#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"
#include "NNObjectDetection.h"
#include "VideoStreamOverlay.h"
#include "obj.h"

#define CHANNEL   0
#define CHANNELNN 3

// Lower resolution for NN processing
#define NNWIDTH  576
#define NNHEIGHT 320

// 定義要偵測的特定物件和對應的傳送碼
#define OBJ_PSILOPOGON_NUCHALIS     1  // Psilopogon nuchalis
#define OBJ_PASSER_MONTANUS         2  // Passer montanus
#define OBJ_GORSACHIUS_MELANOLOPHUS 3  // Gorsachius melanolophus
#define OBJ_CHEIROTONUS_FORMOSANUS  4  // Cheirotonus formosanus
#define OBJ_TRYPOXYLUS_DICHOTOMUS   5  // Trypoxylus dichotomus

VideoSetting config(VIDEO_FHD, 30, VIDEO_H264, 0);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0);
NNObjectDetection ObjDet;
RTSP rtsp;
StreamIO videoStreamer(1, 1);
StreamIO videoStreamerNN(1, 1);

char ssid[] = "hel";    // your network SSID (name)
char pass[] = "1234567890";        // your network password
int status = WL_IDLE_STATUS;

IPAddress ip;
int rtsp_portnum;

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200);  // 僅使用波特率初始化UART2
    
    Serial.println("Starting object detection with UART2 notification");
    Serial2.println("UART2 initialized for object detection notification");

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
    config.setBitrate(2 * 1024 * 1024);    // Recommend to use 2Mbps for RTSP streaming to prevent network congestion
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
    ObjDet.setResultCallback(ODPostProcess);
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

void loop()
{
    // Do nothing
}

// 檢查物件名稱並傳送對應代碼到UART2
void checkAndSendObjectCode(const char* objectName) {
    if (strcmp(objectName, "Psilopogon nuchalis") == 0) {
        Serial2.write(OBJ_PSILOPOGON_NUCHALIS);
        Serial.println("Detected: Psilopogon nuchalis, sent code 1");
    }
    else if (strcmp(objectName, "Passer montanus") == 0) {
        Serial2.write(OBJ_PASSER_MONTANUS);
        Serial.println("Detected: Passer montanus, sent code 2");
    }
    else if (strcmp(objectName, "Gorsachius melanolophus") == 0) {
        Serial2.write(OBJ_GORSACHIUS_MELANOLOPHUS);
        Serial.println("Detected: Gorsachius melanolophus, sent code 3");
    }
    else if (strcmp(objectName, "Cheirotonus formosanus") == 0) {
        Serial2.write(OBJ_CHEIROTONUS_FORMOSANUS);
        Serial.println("Detected: Cheirotonus formosanus, sent code 4");
    }
    else if (strcmp(objectName, "Trypoxylus dichotomus") == 0) {
        Serial2.write(OBJ_TRYPOXYLUS_DICHOTOMUS);
        Serial.println("Detected: Trypoxylus dichotomus, sent code 5");
    }
}

// User callback function for post processing of object detection results
void ODPostProcess(std::vector<ObjectDetectionResult> results)
{
    uint16_t im_h = config.height();
    uint16_t im_w = config.width();

    Serial.print("Network URL for RTSP Streaming: ");
    Serial.print("rtsp://");
    Serial.print(ip);
    Serial.print(":");
    Serial.println(rtsp_portnum);
    Serial.println(" ");

    printf("Total number of objects detected = %d\r\n", ObjDet.getResultCount());
    OSD.createBitmap(CHANNEL);

    if (ObjDet.getResultCount() > 0) {
        for (int i = 0; i < ObjDet.getResultCount(); i++) {
            int obj_type = results[i].type();
            if (itemList[obj_type].filter) {    // check if item should be ignored

                ObjectDetectionResult item = results[i];
                // Result coordinates are floats ranging from 0.00 to 1.00
                // Multiply with RTSP resolution to get coordinates in pixels
                int xmin = (int)(item.xMin() * im_w);
                int xmax = (int)(item.xMax() * im_w);
                int ymin = (int)(item.yMin() * im_h);
                int ymax = (int)(item.yMax() * im_h);

                // Check if detected object is one of our target species and send UART2 code
                checkAndSendObjectCode(itemList[obj_type].objectName);

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
}