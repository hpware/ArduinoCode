#include "recipes/WiFi.h"
#include "Fetch.h"
#include <ArduinoJson.h>
#include <SD.h>
#include <SPI.h>
#include <AudioZero.h>

const char *ssid = "hel";
const char *password = "1234567890";
const char *serverUrl = "http://192.168.1.27:3000/";
// SD CARD 引腳
#define SD_CS 5
#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCK 18

void setup() {
    Serial.begin(9600);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

  // 初始化 SD 卡
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  SPI.setFrequency(1000000);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD卡初始化失敗！");
    return;
  }
    Serial.println("Initialization done.");

    // Open file for reading
    File myFile = SD.open("audio.mp3");
    if (!myFile) {
        Serial.println("Failed to open file for reading");
        return;
    }

    // Send file to server
    sendFileToServer(myFile);
    myFile.close();
}

void loop() {
    // Your loop code here
}

void sendFileToServer(File &file) {
    RequestOptions options;
    options.method = "POST";
    options.headers["Content-Type"] = "audio/mpeg";

    // Read file content
    size_t fileSize = file.size();
    uint8_t *fileContent = new uint8_t[fileSize];
    file.read(fileContent, fileSize);

    // Set request body
    options.body = (const char *)fileContent;
    options.bodyLength = fileSize;

    // Send POST request
    Response response = fetch(serverUrl, options);

    // Check response
    if (response.status == 200) {
        Serial.println("File uploaded successfully");
    } else {
        Serial.println("Failed to upload file");
        Serial.println("Response status: " + String(response.status));
        Serial.println("Response body: " + response.text());
    }

    // Free allocated memory
    delete[] fileContent;
}


    // Fetch the response
    /*Response response = fetch("https://edu.yhw.tw/weather/%E5%A4%A7%E5%B1%AF%E5%B1%B1", options);

    // Check if the response is valid
        // Printing response body as plain text.
        Serial.println();
        Serial.println(response.text());

        // Parse JSON response
        DynamicJsonDocument doc(200);
        deserializeJson(doc, response.text());

        // Access JSON fields
        const char* weather = doc["weather"];
        float temp = doc["temp"];
        int humidity = doc["humidity"];
        float highestTemp = doc["highestTemp"];
        float lowestTemp = doc["lowestTemp"];
        const char* dateTime = doc["DateTime"];

        // Print JSON fields
        Serial.println(weather);
        Serial.println(temp);
        Serial.println(humidity);
        Serial.println(highestTemp);
        Serial.println(lowestTemp);
        Serial.println(dateTime);*/
}
