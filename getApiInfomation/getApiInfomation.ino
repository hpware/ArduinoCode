#include "recipes/WiFi.h"
#include "Fetch.h"
#include <ArduinoJson.h>
#include <SD.h>
#include <SPI.h>

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

  // Initialize SD card
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  SPI.setFrequency(1000000);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD卡初始化失敗！");
    return;
  }
  Serial.println("SD card initialization done.");

  // Open file for reading
  File myFile = SD.open("audio.mp3");
  if (!myFile) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.println("File opened successfully.");

  RequestOptions options;
  Response base64 =
      fetch("https://edu.yhw.tw/yhwdata/filebase64.txt", options);

  Serial.print("Base64 Response Status Code: ");
  Serial.println(base64.status);
  Serial.print("Base64 Response Text: ");
  Serial.println(base64.text());

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

  // Convert file content to String
  String fileContentString = "";
  for (size_t i = 0; i < fileSize; i++) {
    fileContentString += (char)fileContent[i];
  }

  // Set request body
  options.body = fileContentString;

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

