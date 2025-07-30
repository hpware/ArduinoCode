
#include <WiFi.h>
#include "VideoStream.h"
#include <CameraLED.h>
//#include "Base64.h"
//#include <algorithm>


#define CHANNEL 0
#define CHANNEL_STILL 1
#define LED_PWM 13

// Use a pre-defined resolution, or choose to configure your own resolution
// Depending on your WiFi network quality, using HD resolution may lead to an inconsistent frame rate
// VideoSetting config(VIDEO_HD, CAM_FPS, VIDEO_JPEG, 1);
// VideoSetting config(VIDEO_VGA, CAM_FPS, VIDEO_JPEG, 1);
VideoSetting config(1024, 576, CAM_FPS, VIDEO_JPEG, 1);
VideoSetting configStill(1280, 720, 1, VIDEO_JPEG, 1);
CameraLED flashLight;


char ssid[] = "thisishell";       // your network SSID (name)
char pass[] = "keQV6a&6E*xx20!";  // your network password
int status = WL_IDLE_STATUS;
WiFiServer server(80);

uint32_t img_addr = 0;
uint32_t img_len = 0;

#define PART_BOUNDARY "123456789000000000000987654321"
char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
char* IMG_HEADER = "Content-Type: image/jpeg\r\nContent-Length: %lu\r\n\r\n";


String EncodeBase64ImageFile(uint32_t addr, uint32_t len) {
  /**  Serial.println("Encoding image to Base64...");
  Serial.print("Image address: ");
  Serial.println(addr, HEX);
  Serial.print("Image length: ");
  Serial.println(len);

  if (addr == 0 || len == 0) {
    Serial.println("Error: No valid image data provided for Base64 encoding.");
    return "";
  }

  uint8_t* fbBuf = (uint8_t*)addr;
  size_t fbLen = len;

  // The base64_encode function in Base64.h often encodes 3 bytes into 4 chars.
  // The original loop (if (i%3==0) imageFile += String(output);) was problematic.
  // This revised loop correctly processes chunks of 3 bytes.
  String imgFileInBase64 = "data:image/jpeg;base64,";  // Prepend for web display (adjust mime type if not JPEG)
  char chunkOutput[5];                                 // 4 base64 chars + null terminator
  for (size_t i = 0; i < fbLen; i += 3) {
    int bytesToEncode = std::min((size_t)3, fbLen - i);
    if (bytesToEncode <= 0) break;  // Should not happen with correct loop conditions

    base64_encode(chunkOutput, (char*)(fbBuf + i), bytesToEncode);
    imgFileInBase64 += String(chunkOutput);
  }
*/
  String imgFileInBase64 = "HELLO WORLD";
  return imgFileInBase64;
}

void sendHeader(WiFiClient& client) {
  client.print("HTTP/1.1 200 OK\r\nContent-type: multipart/x-mixed-replace; boundary=");
  client.println(PART_BOUNDARY);
  client.print("Access-Control-Allow-Origin: *\r\n");
  client.print("Transfer-Encoding: chunked\r\n");
  client.print("\r\n");
}

void sendChunk(WiFiClient& client, uint8_t* buf, uint32_t len) {
  uint8_t chunk_buf[64] = { 0 };
  uint8_t chunk_len = snprintf((char*)chunk_buf, 64, "%lX\r\n", len);
  client.write(chunk_buf, chunk_len);
  client.write(buf, len);
  client.print("\r\n");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  flashLight.attach(LED_PWM, 1, 2000);
  flashLight.writeMicroseconds(0);
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    delay(5000);
  }
  Camera.configVideoChannel(CHANNEL, config);
  Camera.videoInit();
  Camera.channelBegin(CHANNEL);

  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("new client connected");
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        if (c == '\n') {
          if (currentLine.length() == 0) {
            sendHeader(client);
            while (client.connected()) {
              Camera.getImage(CHANNEL, &img_addr, &img_len);
              uint8_t chunk_buf[64] = { 0 };
              uint8_t chunk_len = snprintf((char*)chunk_buf, 64, IMG_HEADER, img_len);
              sendChunk(client, chunk_buf, chunk_len);
              sendChunk(client, (uint8_t*)img_addr, img_len);
              sendChunk(client, (uint8_t*)STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
              delay(5);  // Increase this delay for higher resolutions to get a more consistent, but lower frame rate
            }
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
    Serial.println("client disconnected");
  } else {
    Serial.println("waiting for client connection");
    delay(1000);
  }
}

void base64ImageRequest() {
  uint32_t still_img_addr = 0;
  uint32_t still_img_len = 0;

  // Capture the image from the dedicated still channel
  Camera.getImage(CHANNEL_STILL, &still_img_addr, &still_img_len);

  // Pass the captured image data to the Base64 encoding function
  const String encodingProcess = EncodeBase64ImageFile(still_img_addr, still_img_len);
  // Debugging
  Serial.print("<!START BLOCK!>");  // Start block for base64 data in case of esp32 just cutting off half of the base64 data.
  Serial.println(encodingProcess);
  Serial.println("<!END BLOCK!>");

  Serial2.print("<!START BLOCK!>");  // Start block for base64 data in case of esp32 just cutting off half of the base64 data.
  Serial2.println(encodingProcess);
  Serial2.println("<!END BLOCK!>");
}

void SetLedBrightness(int nummt) {
  // 1 -> 8
  flashLight.writeMicroseconds(200 * nummt);
}