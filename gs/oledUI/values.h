#ifndef VALUES_H
#define VALUES_H

// State of those buttons.
int b1s = 0;
int b2s = 0;
int b3s = 0;
// Button Pins?
int b1p = 33;
int b2p = 34;
int b3p = 36;
// Screen SDA SCL pins
int sda = 21;
int scl = 22;

// 馬達控制腳位
#define INA1 2 // 馬達方向 1
#define INA2 4 // 馬達方向 2

// 硬體設定
#define RXD2 26         // Modbus RX
#define TXD2 25         // Modbus TX
#define ONE_WIRE_BUS 19 // 溫度感測器資料線
#define LED_32 32       // 指示燈 1
#define LED_27 27       // 指示燈 2

// Modbus 設定
#define MODBUS_SLAVE_ID 8      // Modbus 裝置 ID
#define MODBUS_REGISTER 0x0002 // 讀取 pH 值的暫存器地址

// WiFi & MQTT 設定
const char ssid = "hel";
const charpassword = "1234567890";
const char *mqtt_server = "120.102.36.38";
const int mqtt_port = 5007;

// 計時器
uint32_t modbusTimer = 0;
uint32_t tempTimer = 0;
uint32_t mqttTimer = 0;
const uint32_t interval = 500;

// 30 秒定時發送 Feeder 狀態
bool feederActive = false;
uint32_t feederTimer = 0;
const uint32_t feederInterval = 30000;

float lastPH = 0.0;
float lastTemp = 0.0;
float feedResidue = 50.0; // 初始餘餵食量 50%

// Modbus 錯誤計數
int modbusErrorCount = 0;
const int modbusErrorThreshold = 3;

// 餵食參數
int feedMode = 0;     // 0: 無, 1: 定時, 2: 週期
int feedTime = 0;     // 定時模式時間（格式 HHMM）
int feedInterval = 0; // 週期模式間隔（小時）
int feedDuration = 2; // 投餵時長（秒）
unsigned long lastFeedTime = 0;

#endif