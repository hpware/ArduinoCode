#include <TinyGPS++.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
HardwareSerial GPS_Serial(1); // 使用 UART1

void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); // RX = 16, TX = 17
  Serial.println("GPS 模組啟動中...");
}

void loop() {
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());

    if (gps.location.isUpdated()) {
      Serial.print("經度: ");
      Serial.println(gps.location.lng(), 6);
      Serial.print("緯度: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("速度: ");
      Serial.println(gps.speed.kmph());
      Serial.print("高度: ");
      Serial.println(gps.altitude.meters());
      Serial.print("時間: ");
      Serial.print(gps.time.hour());
      Serial.print(":");
      Serial.print(gps.time.minute());
      Serial.print(":");
      Serial.println(gps.time.second());
      Serial.println("-----------------------------");
    }
  }
}