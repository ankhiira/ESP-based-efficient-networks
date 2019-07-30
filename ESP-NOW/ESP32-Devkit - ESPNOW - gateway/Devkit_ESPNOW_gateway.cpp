/*
  Name: Topology based on ESPNOW communication - Master
  Year: 2018/2019
  Author: Gabriela Chmelarova
  Purpose: ESPNOW Communication between a Master ESP32 and multiple ESP Slaves
  Description: This Master sensor is waiting for messages from Slave nodes and 
  after certain amount of time Master processes them and publishes them via WiFi to 
  MQTT broker. Because ESPNOW can't work with WiFi together, after publishing 
  messages to MQTT will ESP restart itself. The restart is necessary as 
  ESP-NOW doesn't seem to work again even after WiFi is disabled.
  Credits: this work was inspired on following projects
   1a) ESP-NOW communication - https://github.com/HarringayMakerSpace/ESP-Now
   1b) - https://navody.arduino-shop.cz/navody-k-produktum/bezdratova-komunikace-esp-now-s-esp32.html
   1c) - https://github.com/SensorsIot/ESP-Now-Tests
   3)  Deep-sleep usage - https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
   4)  DHT sensor - https://github.com/adafruit/DHT-sensor-library/blob/master/examples/DHTtester/DHTtester.ino
*/

#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_wifi.h>

#include <vector>
#include <iostream>

#include "Adafruit_Sensor.h"
#include "DHT.h"

using namespace std;

/* Definiton of DHT pins */
#define DHTTYPE DHT22 // DHT 22
const int dhtpin = 17;
DHT dht(dhtpin, DHTTYPE); // Initialize DHT sensor.

/* Global copy of slave */
#define NUMSLAVES 20
esp_now_peer_info_t slaves[NUMSLAVES] = {};
int SlaveCnt = 0;

/* Definiton of deep sleep variables 
  (this part was inspired from project mentioned under number 3) in header) */
#define uS_TO_S_FACTOR 1000000  /* Conversion factor from micro seconds to seconds */
#define TIME_TO_SLEEP  60       /* Time after ESP32 will go to sleep (in seconds) */

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

/* Set static MAC address */
uint8_t masterCustomMac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};
void initVariant() {
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(ESP_IF_WIFI_STA, &masterCustomMac[0]);
}

/* Definiton of topic and clientId for MQTT */
const char deviceTopic[] = "my-home/ESPNOW/ZZZZZ/XXXXX/YYYYY";
char clientId[] = "device:" DEVICE_TYPE ":" DEVICE_ID;

WiFiClient wifiClient;
PubSubClient client(mqttServer, 1883, wifiClient);

String deviceMac;

/* Struct for storing recieved ESPNOW message 
 (this part was inspired from project mentioned under number 1a) in header) */
struct __attribute__((packed)) SENSOR_DATA {
  char  deviceType;
  char  type;
  float value;
} sensorData;

/* Struct for storing recieved ESPNOW messages in vector */
struct VECTOR_DATA {
  String  devMac;
  char    deviceType;
  char    type;
  float   value;
} vectorData;

/* Vector for storing all recieved ESPNOW messages */
vector<VECTOR_DATA> vectorStruc;

/* Function prototypes */
void InitESPNow();
void sendData();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecieved(const uint8_t *mac, const uint8_t *data, int len);
void wifiConnect();
void mqttConnect();
void renameTopic();
void readSensor();

void setup() {
  Serial.begin(9600); Serial.println();
  Serial.println("**ESP-NOW/Multi-Slave/Master**");
  // This is the mac address of the Master in Station Mode
  Serial.print("This node STA mac: "); Serial.println(WiFi.macAddress());

  // Init ESPNow with a fallback logic
  InitESPNow();

  /* We can also measure sensor values of gateway, if we attached one 
    - unkomment lines below to start reading sensor values */
  dht.begin();
  readSensor(); 

  /* Once ESPNow is successfully initialised, we will register for send callback to
    get the status of trasnmitted packet */
  esp_now_register_send_cb(OnDataSent);

  /* Here we configure the wake up source,
    then we set our ESP32 to wake up every TIME_TO_SLEEP seconds 
    (this part was inspired from project mentioned under number 3) in header) */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
}

int timer;

void loop() {
  /* we wait 20 seconds for messages delivery 
  after that we publish recieved messages to MQTT broker */
  if (millis() - timer > 20000) {
    Serial.println("***Ready to send***");
    // Print values to check what we will try to publish to MQTT
    for (const auto& elem : vectorStruc) {
      Serial.printf("DeviceType=%c Type=%c, Value=%0.0f\n", 
      elem.deviceType, elem.type, elem.value);
    }

    /* We try to connect to WiFi and MQTT broker */
    wifiConnect();
    mqttConnect();
    renameTopic();
    Serial.println("***Ready to restart***");

    /* After publishing values to MQTT gateway restarts itself */
    client.disconnect();
    delay(200);
    ESP.restart(); // <----- Reboots to re-enable ESP-NOW
  } else {
    Serial.println("Waiting for ESP-NOW messages...");
    delay(5000);
  }
}

/* Function to establish WiFi connection */
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

/* Function to establish MQTT connection */
void mqttConnect() {
  if (!client.connected()) {
    Serial.print("Reconnecting MQTT client to "); Serial.println(mqttServer);
    while (!client.connect(clientId, mqttUser, mqttPassword)) {
      Serial.print(".");
      delay(250);
    }
    Serial.println("Ready to publish");
    Serial.println();
  }
}

/* Function to publish recieved message to MQTT topic 
(this part was inspired from project mentioned under number 1a) in header) */
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

/* Function to rename MQTT topic to publish recieved message 
  (this part was inspired from project mentioned under number 1a) in header) */
void renameTopic() {
  for (const auto& elem : vectorStruc) {
    String payload;
    String topic = String(deviceTopic);
    // here we replace XXXXX with MAC address of sending device
    topic.replace("XXXXX", elem.devMac);
    // then we replace ZZZZZ with name of the platform, which sent us message
    switch (elem.deviceType) {
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

    // here we replace YYYYY with value type 
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
    
    Serial.println();
    // if any of fields wasn't replace, we discard whole message
    if ((topic.substring(0) == "XXXXX") || (topic.substring(0) == "YYYYY") || (topic.substring(0) == "ZZZZZ")) {
      return;
    }
    // then we publish message on created topic
    publishTo(topic.c_str(), payload.c_str());
  }
}

/* Function to establish ESPNOW communication 
  (this part was inspired from project mentioned under number 1a) in header) */
void InitESPNow() {
  if (esp_now_init()!=0) {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  } else {
    Serial.println("ESPNow Init Success");
  }
  /* register callback for recieved messages via ESPNOW */
  esp_now_register_recv_cb(OnDataRecieved);
}

/* Callback when data is sent from Master to Slave */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

/* Callback for processing recieved data 
  (this part was inspired from project mentioned under number 1a) in header) */
void OnDataRecieved(const uint8_t *mac, const uint8_t *data, int len) {
 for (int ii = 0; ii < 6; ++ii ) {
    slaves[0].peer_addr[ii] = (uint8_t) mac[ii];
  }

  const esp_now_peer_info_t *peer = &slaves[0];
  const uint8_t *peer_addr = slaves[0].peer_addr;
  Serial.print("Processing: ");
  for (int ii = 0; ii < 6; ++ii ) {
    Serial.print((uint8_t) slaves[0].peer_addr[ii], HEX);
    if (ii != 5) Serial.print(":");
  }
  Serial.print(" Status: ");
  // check if the peer exists
  bool exists = esp_now_is_peer_exist(peer_addr);
  if (exists) {
    // Slave already paired.
    Serial.println("Already Paired");
  } else {
    // Slave not paired, attempt pair
    esp_err_t addStatus = esp_now_add_peer(peer);
    if (addStatus == ESP_OK) {
      // Pair success
      Serial.println("Pair success");
    } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
      // How did we get so far!!
      Serial.println("ESPNOW Not Init");
    } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Add Peer - Invalid Argument");
    } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
      Serial.println("Peer list full");
    } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
      Serial.println("Out of memory");
    } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
      Serial.println("Peer Exists");
    } else {
      Serial.println("Not sure what happened");
    }
    delay(100);
  }

  /* Chosen number is sent to slave as acknowledgement of received message */
  uint8_t ack = 1;
  esp_err_t result = esp_now_send(peer_addr, &ack, sizeof(ack));

  Serial.print("Send Status: ");
    if (result == ESP_OK) {
      Serial.println("Success");
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
      // How did we get so far!!
      Serial.println("ESPNOW not Init.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Invalid Argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
      Serial.println("Internal Error");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
      Serial.println("ESP_ERR_ESPNOW_NO_MEM");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      Serial.println("Peer not found.");
    } else {
      Serial.println("Not sure what happened");
    }
    delay(100); 

  Serial.println("Acknowledgement sent");

  /* Then we parse recieved message */
  deviceMac = "";
  deviceMac += String(mac[0], HEX);     
  deviceMac += String(mac[1], HEX);
  deviceMac += String(mac[2], HEX);
  deviceMac += String(mac[3], HEX);
  deviceMac += String(mac[4], HEX);
  deviceMac += String(mac[5], HEX);

  /* Recieved message is parsed and stored into struct vector */
  memcpy(&sensorData, data, sizeof(sensorData));
  vectorData.deviceType = sensorData.deviceType;
  vectorData.type = sensorData.type;
  vectorData.value = sensorData.value;
  vectorData.devMac = deviceMac;

  Serial.print("recv_cb, msg from device: "); Serial.print(vectorData.devMac);
  Serial.printf(" DeviceType=%c Type=%c, Value=%.2f\n", 
  vectorData.deviceType, vectorData.type, vectorData.value);

  vectorStruc.push_back(vectorData);
}

/* Function for reading sensor values */
void readSensor() {
  /* Parse MAC address of this device */
  String mac = WiFi.macAddress();
  deviceMac = "";
  deviceMac += String(mac[0], HEX);     
  deviceMac += String(mac[1], HEX);
  deviceMac += String(mac[2], HEX);
  deviceMac += String(mac[3], HEX);
  deviceMac += String(mac[4], HEX);
  deviceMac += String(mac[5], HEX);

  Serial.printf("***Own measured data***\n");

  /* Read temperature data and store them into struct vector */
  vectorData.deviceType = 'h';
  vectorData.type = 't';
  vectorData.value = dht.readTemperature();
  vectorData.devMac = deviceMac;
  vectorStruc.push_back(vectorData);

  Serial.printf("ValueType=%c, Status=%.2f\n", 
  vectorData.type, vectorData.value);

  /* Read humidity data and store them into struct vector */
  vectorData.deviceType = 'h';
  vectorData.type = 'h';
  vectorData.value = dht.readHumidity();
  vectorData.devMac = deviceMac;
  vectorStruc.push_back(vectorData);

  Serial.printf("ValueType=%c, Status=%.2f\n", 
  vectorData.type, vectorData.value);
}