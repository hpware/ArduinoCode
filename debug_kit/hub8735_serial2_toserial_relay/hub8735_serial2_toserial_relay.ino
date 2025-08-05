// DEVICE: HUB8735 ULTRA USING REALTEK RTL8735B
#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"
#include "NNObjectDetection.h"
#include "VideoStreamOverlay.h"
#include "Base64.h"
#include <CameraLED.h>

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
}

void loop() {
    if (Serial2.available()) {
        String data = Serial2.readStringUntil('\n');
        Serial.println(data);
    }

    if (Serial.available()) {
        String data = Serial.readStringUntil('\n');
        Serial2.println(data);
    }
}