#include <WiFi.h>
#include <PubSubClient.h>

char ssid[] = "hel";          // your network SSID (name)
char pass[] = "1234567890";   // your network password
int status = WL_IDLE_STATUS;  // Indicator of Wifi status

const char* mqttServer = "39312.20090526.xyz";
const char* clientId = "camera_client_7884a866-4ae1-4945-9fba-b2b8d2b7c5a9";
const uint16_t mqttPort = 1883;
const char* clientUser = "";
const char* clientPass = "";
const char* mainPublishDir = "imagedata/6e92ff0d-adbe-43d8-b228-e4bc6f948506/";
char publishPayload[] = "";
int numberadd = 0;

void callback(char* topic, byte* payload, unsigned int length) {}

WiFiClient wifiClient;
PubSubClient client(mqttServer, mqttPort, callback, wifiClient);

void reconnect() {
  // Loop until we're reconnected
  while (!(client.connected())) {
    //Serial.print("\r\nAttempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId)) {
      Serial.println("Connected");
    } else {
      Serial.println("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(2000);
    }
  }
}

void setup() {
  // Initialize serial and wait for port to open:

  // Attempt to connect to WiFi network
  while (status != WL_CONNECTED) {
    //Serial.print("\r\nAttempting to connect to SSID: ");
    //Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  wifiClient.setNonBlockingMode();
  while (!client.connect(clientId, clientUser, clientPass)) {
  }
  // Allow Hardware to sort itself out (yeah, the dev doesn't know what he's doing.)
  delay(1500);
}

// why tf is it sending random data to the server?????
void loop() {
  if (!(client.connected())) {
    reconnect();
  }
  client.publish(mainPublishDir, publishPayload);
  client.loop();
  delay(10);
}
