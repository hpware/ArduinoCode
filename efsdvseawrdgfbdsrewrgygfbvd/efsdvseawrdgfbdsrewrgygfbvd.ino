#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"
#include "NNObjectDetection.h"
#include "VideoStreamOverlay.h"
#include "ObjectClassList.h"
#include "AmebaFatFS.h" // Needed for FA_WRITE, FA_CREATE_ALWAYS
#include "UUID.h"

#define CHANNEL   0 // RTSP Stream Channel
#define CHANNEL_2 1 // JPEG Capture Channel
#define CHANNELNN 3 // NN Input Channel

// Lower resolution for NN processing
#define NNWIDTH  576
#define NNHEIGHT 320

uint32_t img_addr = 0;
uint32_t img_len = 0;

AmebaFatFS fs; // Filesystem object

UUID uuid; // UUID generator object
// String uuidstr; // This global variable seems unused, can be removed if not needed elsewhere

// Define object codes for UART transmission (Chinese comments kept)
#define OBJ_PSILOPOGON_NUCHALIS     1  // Psilopogon nuchalis
#define OBJ_PASSER_MONTANUS         2  // Passer montanus
#define OBJ_GORSACHIUS_MELANOLOPHUS 3  // Gorsachius melanolophus
#define OBJ_CHEIROTONUS_FORMOSANUS  4  // Cheirotonus formosanus
#define OBJ_TRYPOXYLUS_DICHOTOMUS   5  // Trypoxylus dichotomus

// Video configurations
VideoSetting config(VIDEO_FHD, 30, VIDEO_H264, 0); // RTSP H.264 stream
VideoSetting config2(VIDEO_FHD, 10, VIDEO_JPEG, 1); // JPEG capture
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0); // NN Input

// Objects for different functionalities
NNObjectDetection ObjDet;
RTSP rtsp;
StreamIO videoStreamer(1, 1);  // RTSP stream pipeline
// StreamIO videoStreamer2(1, 1); // This seems unused, can be removed if not needed
StreamIO videoStreamerNN(1, 1); // NN stream pipeline

// WiFi Credentials
char ssid[] = "a";       // your network SSID (name)
char pass[] = "pi!=7.188"; // your network password
int status = WL_IDLE_STATUS;

IPAddress ip;
int rtsp_portnum;

// Forward declarations
void ODPostProcess(std::vector<ObjectDetectionResult> results);
void checkAndSendObjectCode(const char* objectName);


void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200);  // UART for sending detection codes (Chinese comment kept)

    Serial.println("Starting object detection with UART2 notification");
    Serial2.println("UART2 initialized for object detection notification");

    // --- WiFi Connection ---
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    while (status != WL_CONNECTED) {
        status = WiFi.begin(ssid, pass);
        Serial.print(".");
        delay(2000); // wait 2 seconds for connection
    }
    ip = WiFi.localIP();
    Serial.println("\nWiFi Connected.");
    Serial.print("IP Address: ");
    Serial.println(ip);

    // --- Filesystem Initialization --- ADDED fs.begin() HERE ---
    Serial.println("Initializing Filesystem...");
    if (fs.begin()) {
        Serial.println("Filesystem initialized successfully.");
        Serial.print("SD Card Root Path: ");
        Serial.println(fs.getRootPath());
    } else {
        Serial.println("ERROR: Failed to initialize filesystem!");
        Serial.println("Halting execution. Check SD card.");
        while (1) { delay(1000); } // Halt
    }

    // --- Camera Configuration ---
    Serial.println("Configuring Camera Channels...");
    config.setBitrate(2 * 1024 * 1024); // Recommend 2Mbps for RTSP
    Camera.configVideoChannel(CHANNEL, config);
    Camera.configVideoChannel(CHANNELNN, configNN);
    Camera.configVideoChannel(CHANNEL_2, config2); // Config for JPEG channel
    Camera.videoInit();
    Serial.println("Camera Initialized.");

    // --- Start Camera Channels --- ADDED Camera.channelBegin(CHANNEL_2) ---
    Serial.println("Starting Camera Channels...");
    Camera.channelBegin(CHANNEL);   // Start RTSP H.264 stream channel
    Camera.channelBegin(CHANNELNN); // Start NN Input RGB channel
    Camera.channelBegin(CHANNEL_2); // *** Start JPEG Capture channel ***
    Serial.println("Camera Channels Started.");


    // --- RTSP Setup ---
    Serial.println("Configuring RTSP Server...");
    rtsp.configVideo(config);
    rtsp.begin();
    rtsp_portnum = rtsp.getPort();
    Serial.print("RTSP Server Started on port: ");
    Serial.println(rtsp_portnum);
    Serial.print("Network URL for RTSP Streaming: rtsp://");
    Serial.print(ip);
    Serial.print(":");
    Serial.println(rtsp_portnum);

    // --- Object Detection Setup ---
    Serial.println("Configuring Object Detection...");
    ObjDet.configVideo(configNN);
    ObjDet.setResultCallback(ODPostProcess); // Set the callback function
    ObjDet.modelSelect(OBJECT_DETECTION, DEFAULT_YOLOV4TINY, NA_MODEL, NA_MODEL);
    ObjDet.begin(); // Initialize NN engine
    Serial.println("Object Detection Initialized."); // Check Serial for "vipnn not applied" error here

    // --- StreamIO Link: Camera(H264) -> RTSP ---
    Serial.println("Setting up StreamIO: Camera(H264) -> RTSP...");
    videoStreamer.registerInput(Camera.getStream(CHANNEL));
    videoStreamer.registerOutput(rtsp);
    if (videoStreamer.begin() != 0) {
        Serial.println("ERROR: StreamIO link (RTSP) start failed!");
    } else {
        Serial.println("StreamIO link (RTSP) started.");
    }

    // --- StreamIO Link: Camera(RGB) -> NN ---
    Serial.println("Setting up StreamIO: Camera(RGB) -> NN...");
    videoStreamerNN.registerInput(Camera.getStream(CHANNELNN));
    videoStreamerNN.setStackSize(); // Use default stack size
    videoStreamerNN.setTaskPriority(); // Use default priority
    videoStreamerNN.registerOutput(ObjDet);
    if (videoStreamerNN.begin() != 0) {
        Serial.println("ERROR: StreamIO link (NN) start failed!");
    } else {
        Serial.println("StreamIO link (NN) started.");
    }

    // --- OSD Setup ---
    Serial.println("Configuring OSD...");
    OSD.configVideo(CHANNEL, config); // Configure OSD for the RTSP channel
    OSD.begin();
    Serial.println("OSD Initialized.");

    Serial.println("\nSetup Complete. Starting main loop...");
}

void loop()
{
    // --- Image Saving Logic --- (REPLACED SECTION)
    // Generate UUID and construct filename correctly
    uuid.generate();
    String filename = String(uuid.toCharArray()) + ".jpg"; // Corrected concatenation
    String fullPath = String(fs.getRootPath()) + filename;

    Serial.print("Attempting to save image: ");
    Serial.println(fullPath);

    // Reset image variables before capture
    img_addr = 0;
    img_len = 0;

    // Get image from JPEG channel
    Camera.getImage(CHANNEL_2, &img_addr, &img_len);

    // Check if image capture was successful
    if (img_len == 0 || img_addr == 0) {
        Serial.println("ERROR: Failed to get image from camera channel 2!");
    } else {
        Serial.print("Got image: Address=0x");
        Serial.print(img_addr, HEX);
        Serial.print(", Length=");
        Serial.println(img_len);

        // Open file for writing (with check) using FatFS flags
        // FA_WRITE: Enable writing
        // FA_CREATE_ALWAYS: Create file if it doesn't exist, overwrite if it does
        File file = fs.open(fullPath, FA_WRITE | FA_CREATE_ALWAYS); // Use correct flags
        if (!file) {
            Serial.println("ERROR: Failed to open file for writing!");
            Serial.print("Check SD card space and permissions. Path: ");
            Serial.println(fullPath);
        } else {
            Serial.println("File opened successfully for writing.");

            // Write image data to file (with check)
            size_t bytesWritten = file.write((uint8_t *)img_addr, img_len);

            if (bytesWritten != img_len) {
                Serial.print("ERROR: Failed to write complete file! Wrote ");
                Serial.print(bytesWritten);
                Serial.print(" of ");
                Serial.print(img_len);
                Serial.println(" bytes.");
                // Consider deleting the partial file if write fails: fs.remove(fullPath);
            } else {
                Serial.print("Successfully wrote ");
                Serial.print(bytesWritten);
                Serial.print(" bytes to ");
                Serial.println(filename);
            }
            // Close the file to save changes
            file.close();
            Serial.println("File closed.");
        }
    }
    // --- End of Replaced Section ---


    // --- Optional Serial2 Trigger Logic (kept commented out as in original) ---
    /*
    if (Serial2.available()) {
        String receivedData = Serial2.readStringUntil('\n');
        receivedData.trim(); // Remove potential leading/trailing whitespace

        Serial.print("Received on Serial2: ");
        Serial.println(receivedData);

        // Use .equals() for reliable String comparison
        if (receivedData.equals("true")) {
            Serial.println("Trigger condition met via Serial2.");
            // If you only want to save on trigger, move the
            // image saving logic from above into this block.
        } else {
             Serial.println("Trigger condition not met via Serial2.");
        }
    }
    */

    // Delay before next loop iteration / image capture attempt
    delay(1000); // Delay 1 second between captures
}


// 檢查物件名稱並傳送對應代碼到UART2 (Function kept as provided)
void checkAndSendObjectCode(const char* objectName) {
    int codeToSend = 0; // Default to 0 (no specific code) - Added for clarity

    if (strcmp(objectName, "Psilopogon nuchalis") == 0) {
        codeToSend = OBJ_PSILOPOGON_NUCHALIS;
        Serial.println("Detected: Psilopogon nuchalis, sent code 1");
    }
    else if (strcmp(objectName, "Passer montanus") == 0) {
        codeToSend = OBJ_PASSER_MONTANUS;
        Serial.println("Detected: Passer montanus, sent code 2");
    }
    else if (strcmp(objectName, "Gorsachius melanolophus") == 0) {
        codeToSend = OBJ_GORSACHIUS_MELANOLOPHUS;
        Serial.println("Detected: Gorsachius melanolophus, sent code 3");
    }
    else if (strcmp(objectName, "Cheirotonus formosanus") == 0) {
        codeToSend = OBJ_CHEIROTONUS_FORMOSANUS;
        Serial.println("Detected: Cheirotonus formosanus, sent code 4");
    }
    else if (strcmp(objectName, "Trypoxylus dichotomus") == 0) {
        codeToSend = OBJ_TRYPOXYLUS_DICHOTOMUS;
        Serial.println("Detected: Trypoxylus dichotomus, sent code 5");
    }
    // Add more else if blocks here for other target objects

    // Send the code if it's one of the target objects
    if (codeToSend != 0) {
        Serial2.println(codeToSend);
    }
}

// User callback function for post processing of object detection results (Function kept as provided)
void ODPostProcess(std::vector<ObjectDetectionResult> results)
{
    uint16_t im_h = config.height(); // RTSP stream height
    uint16_t im_w = config.width();  // RTSP stream width

    // This print statement might be excessive if called frequently
    // Serial.print("Network URL for RTSP Streaming: rtsp://");
    // Serial.print(ip);
    // Serial.print(":");
    // Serial.println(rtsp_portnum);
    // Serial.println(" ");

    int detection_count = ObjDet.getResultCount(); // Use a variable
    // printf("Total number of objects detected = %d\r\n", detection_count); // Can be verbose

    OSD.createBitmap(CHANNEL); // Clear previous OSD drawings

    if (detection_count > 0) {
        // Serial.println("--- Detected Objects ---"); // Optional: Add header for clarity
        for (int i = 0; i < detection_count; i++) {
            int obj_type = results[i].type();

            // Check itemList filter flag
            if (itemList[obj_type].filter) {
                ObjectDetectionResult item = results[i];
                const char* objectName = itemList[obj_type].objectName; // Get name once
                int score = item.score(); // Get score once

                // Convert normalized coordinates (0.0-1.0) to pixel coordinates
                int xmin = (int)(item.xMin() * im_w);
                int xmax = (int)(item.xMax() * im_w);
                int ymin = (int)(item.yMin() * im_h);
                int ymax = (int)(item.yMax() * im_h);

                // Check if detected object is one of our target species and send UART2 code
                checkAndSendObjectCode(objectName);

                // Draw boundary box on RTSP stream
                // printf("Item %d %s:\t%d %d %d %d\n\r", i, objectName, xmin, xmax, ymin, ymax); // Print detection info
                OSD.drawRect(CHANNEL, xmin, ymin, xmax, ymax, 3, OSD_COLOR_WHITE);

                // Prepare and draw identification text above the box
                char text_str[40]; // Increased size slightly
                snprintf(text_str, sizeof(text_str), "%s %d%%", objectName, score); // Add % sign
                // Adjust text position slightly if needed
                OSD.drawText(CHANNEL, xmin, ymin - OSD.getTextHeight(CHANNEL) - 2, text_str, OSD_COLOR_CYAN);
            }
        }
         // Serial.println("------------------------"); // Optional: Add footer
    }
    OSD.update(CHANNEL); // Update the OSD overlay
}
