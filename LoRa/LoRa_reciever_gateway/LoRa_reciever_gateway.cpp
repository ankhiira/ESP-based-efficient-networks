/*
  Name: Topology based on LoRa communication - Reciever
  Year: 2018/2019
  Author: Gabriela Chmelarova
  Purpose: LoRa Communication between reciever and multiple senders
  Credits: this work was inspired on following projects
   1) LoRa communication - https://github.com/dragino/Arduino-Profile-Examples/tree/master/libraries/Dragino/examples/LoRa/multi-nodes-with-temperature-sensor
   2) Deep-sleep usage - https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
*/

#include <SPI.h>
#include <RH_RF95.h>

#include <vector>
#include <iostream>

#include "Adafruit_Sensor.h"
#include "DHT.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_wifi.h>

using std::vector;

//When use LG01, uncomment this line, so print the result in Console. 
//When use LoRa Mini Dev, Comment this link
//#define  LG01_GATEWAY  

#ifdef LG01_GATEWAY
#include <Console.h>
#include <Process.h>
#define BAUDRATE 115200 
#define SerialPrint Console
#else
#define SerialPrint Serial
#endif

float frequency = 868.0;// Change the frequency here.

#define RFM95_CS 18
#define RFM95_RST 9
#define RFM95_INTERR 26

#define SX1278_SCK 5
#define SX1278_MISO 19
#define SX1278_MOSI 27
#define SX1278_CS   18

/*Initialization of rf95  */
RH_RF95 rf95(RFM95_CS, RFM95_INTERR);

//-------- Variables for WiFI and MQTT connection -----------
const char* ssid = "SSID";
const char* password = "password";

const char* mqttServer = "mqttServer";
const int   mqttPort = 1883;
const char *mqttUser = "mqttUser";
const char *mqttPassword = "mqttPassword";

#define DEVICE_TYPE "AP"
#define DEVICE_ID "Devkit"
//-------- Customise the above values --------

char clientId[] = "device:" DEVICE_TYPE "-" DEVICE_ID;

// the X's get replaced with the remote sensor device mac address
const char deviceTopic[] = "my-home/LoRaNetwork/ZZZZZ/XXXXX/YYYYY";

WiFiClient wifiClient;
PubSubClient client(mqttServer, 1883, wifiClient);

int timer;

// function prototypes
void sendData();
void wifiConnect();
void mqttConnect();
void renameTopic();
void readSensor();

/* Struct for storing measured values */
// keep in sync with ESP_NOW sensor struct
struct __attribute__((packed)) SENSOR_DATA {
  char  deviceType;
  char  type;
  float value;
  char  devMac[6];
} sensorData;

struct VECTOR_DATA {
  String  devMac;
  char  deviceType;
  char  type;
  float value;
} vectorData;

vector<VECTOR_DATA> vectorStruc;

void setup() {
  SerialPrint.begin(9600);
  while (!Serial); // Wait for serial port to be available
  Serial.println("LoRa_Simple_Client_DHT11");
  //  SPI.begin(SX1278_SCK, SX1278_MISO, SX1278_MOSI, SX1278_CS);
  if (!rf95.init())
    Serial.println("init failed");
  Serial.println("Humidity and temperature\n\n"); 
  // Setup ISM frequency
  rf95.setFrequency(frequency);
  // Setup Power,dBm
  rf95.setTxPower(13);
  
  // Defaults BW Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  SerialPrint.print("Listening on frequency: ");
  SerialPrint.println(frequency);
}

void loop() {
  /* we wait 20 seconds for messages delivery 
     - after that we put ESP into deep sleep for 1 minute */
  if (millis() - timer > 20000) {
    Serial.println("***Ready to send***");
    for (const auto& elem : vectorStruc) {
      Serial.printf("DeviceType=%c Type=%c, Value=%.2f\n", 
      elem.deviceType, elem.type, elem.value);
    }
    wifiConnect();
    mqttConnect();
    renameTopic();
    timer = millis();

  } else {
    // Serial.println("Waiting for LoRa messages");
    if (rf95.available()) {
      Serial.println("Message recieved.");
      // Should be a message for us now   
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(sensorData);

      if(rf95.recv((uint8_t *)&sensorData, &len)) {
        Serial.print("Mac=");
        for (size_t i = 0; i < 6; i++) {
          Serial.printf("%c", sensorData.devMac[i]);
        }
        Serial.printf(", Device type=%c, ValueType=%c, value=%.2f\n", sensorData.deviceType, sensorData.type, sensorData.value);

        vectorData.deviceType = sensorData.deviceType;
        vectorData.type = sensorData.type;
        vectorData.value = sensorData.value;
        vectorData.devMac = sensorData.devMac;
        vectorStruc.push_back(vectorData);

        sensorData.type = {};
        sensorData.deviceType = {};
        sensorData.value = 0;
      } else {
        SerialPrint.println("recv failed");
      }
    }
  }
}

void wifiConnect() {
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to "); Serial.print(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
     delay(250);
     Serial.print(".");
  }  
  Serial.print("\nWiFi connected, IP address: "); Serial.println(WiFi.localIP());
}

void mqttConnect() {
  if (!client.connected()) {
    Serial.print("Reconnecting MQTT client to "); Serial.println(mqttServer);
    if (!client.connect(clientId, mqttUser, mqttPassword)) {
      Serial.println("Enable to connect");
      return;
    }
    while (!client.connect(clientId, mqttUser, mqttPassword)) {
      Serial.print(".");
      delay(250);
    }
    Serial.println("Ready to publish");
    Serial.println();
  }
}

void publishTo(const char* topic, const char* payload) {
  Serial.print("publish "); 
  if (client.publish(topic, payload)) {
    Serial.print(" OK ");
  } else {
    Serial.print(" FAILED ");
  }
  Serial.print(" topic: "); Serial.print(topic);
  Serial.print(" payload: "); Serial.println(payload);
}

void renameTopic() {
  for (const auto& elem : vectorStruc) {
    String payload;
    String topic = String(deviceTopic);
    topic.replace("XXXXX", elem.devMac);
    switch (elem.deviceType)
    {
    case 'w':
      topic.replace("ZZZZZ", "WeMos");
      break;
    case 'd':
      topic.replace("ZZZZZ", "DevKit");
      break;
    case 'b':
      topic.replace("ZZZZZ", "WeMosBattery");
      break;
    case 'l':
      topic.replace("ZZZZZ", "LoRa");
      break;
    case 'n':
      topic.replace("ZZZZZ", "Lolin32");
      break;
    default:
      break;
    }

    switch (elem.type) {
    case 't':
      payload = String(elem.value, 1);
      topic.replace("YYYYY", "temperature");
      break;
    case 'h':
      payload = String(elem.value, 1);
      topic.replace("YYYYY", "humidity");
      break;
    case 'l':
      payload = String(elem.value, 0);
      topic.replace("YYYYY", "lightLevel");
      break;
    case 'o':
      payload = String(elem.value, 1);
      topic.replace("YYYYY", "objectTemperature");
      break;
    case 'r':
      if (elem.value < 0.5) {
        payload = "Raining";
      } else if (elem.value > 0.5 && elem.value < 1.5) {
        payload = "Rain Warning";
      } else {
        payload = "Not Raining";
      }
      topic.replace("YYYYY", "rain");
      break;
    default:
      break;
    }
    Serial.println(topic.substring(0) == "ZZZZZ");
    if ((topic.substring(0) == "XXXXX") || (topic.substring(0) == "YYYYY") || (topic.substring(0) == "ZZZZZ")) {
      return;
    }
    publishTo(topic.c_str(), payload.c_str());
  }
}
