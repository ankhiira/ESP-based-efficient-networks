/*
  Name: Topology based on BLE communication - Client
  Year: 2018/2019
  Author: Gabriela Chmelarova
  Purpose: BLE Communication between Client ESP32 and multiple ESP Servers
  Credits: this work was inspired on following projects
   1) BLE communication - https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE
   2) Deep-sleep usage - https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
   3) DHT sensor - https://github.com/adafruit/DHT-sensor-library/blob/master/examples/DHTtester/DHTtester.ino
*/

#include <BLEDevice.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_wifi.h>

#include <vector>
#include <iostream>

#include "Adafruit_Sensor.h"
#include "DHT.h"

using std::vector;

/* define DHT pins */
#define DHTTYPE DHT11 // DHT 11
const int dhtpin = 14;
DHT dht(dhtpin, DHTTYPE); // Initialize DHT sensor.

/* BLE variables */
BLEScan* pBLEScan;
static BLEAdvertisedDevice* myDevice;
static BLERemoteCharacteristic* pRemoteCharacteristic;
BLEClient* pClient = NULL;

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
bool onoff = true;

// The remote service we wish to connect to.
static BLEUUID serviceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

bool connectToServer();

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("disconnected from BLE server");
  }
};

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

/* Struct for storing measured values */
// keep in sync with ESP_NOW sensor struct
struct __attribute__((packed)) SENSOR_DATA {
  char  deviceType;
  char  type;
  float value;
} sensorData;

struct VECTOR_DATA {
  String  devMac;
  char  deviceType;
  char  type;
  float value;
} vectorData;

vector<VECTOR_DATA> vectorStruc;

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  Serial.print("Notify callback for characteristic ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
  Serial.print("data: ");
  Serial.printf("%u",*pData);
  Serial.println();

  // After we store values to local variables we disconnect from Server
  pClient->disconnect();
  
  vectorData.type = 't';
  vectorData.value = *pData;
  vectorData.devMac = myDevice->getAddress().toString().c_str();
  vectorStruc.push_back(vectorData);

  /* Check if Client is still connected */
  Serial.print("Client is still connected:");
  Serial.println(pClient->isConnected());
}

/* Scan for BLE servers and find the first one that advertises the service we are looking for. */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  /* Called for each advertising BLE server. */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Let us now see if this device contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      Serial.printf("Adress of wanted: ");
      BLEAddress addr = advertisedDevice.getAddress();
      Serial.println(addr.toString().c_str());
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
      if (advertisedDevice.getName() == "r") {
        vectorData.deviceType = 'r';
      }

      if (advertisedDevice.getName() == "l") {
        vectorData.deviceType = 'l';
      }
    } // Found our server
  }
};

/* function prototypes */
bool connectToServer();
void BLEInit();
void wifiConnect();
void mqttConnect();
void renameTopic();
void readSensor();

void setup() {
  Serial.begin(9600); Serial.println();
  Serial.println("Starting BLE");
  // This is the mac address of the Master in Station Mode
  Serial.print("This node STA mac: "); Serial.println(WiFi.macAddress());
  dht.begin();
  BLEInit();
}

int timeCounter;

void loop() {
  /* we wait 20 seconds for messages delivery 
    after that we publish recieved messages to MQTT broker */
  if (millis() - timeCounter > 20000) {
    Serial.println("***Ready to send***");
    // Print values to check what we will try to publish to MQTT
    for (const auto& elem : vectorStruc) {
      Serial.printf("DeviceType=%c Type=%c, Value=%0.0f\n", 
      elem.deviceType, elem.type, elem.value);
    }

    // at first we read own sensor in gateway and add it to the delivered sensor readings
    readSensor();
    /* We try to connect to WiFi and MQTT broker */
    wifiConnect();
    mqttConnect();
    renameTopic();
    timeCounter = millis();

  /* we wait 20 seconds for BLE Server connection
  - after that we connect to WiFi and publish messages to MQTT broker */
  } else {
    Serial.println("Scanning for BLE devices...");
    // Device will start scanning the network and search for devices with desired UUID
    pBLEScan->start(5, false);
    // If the flag "doConnect" is true then we have scanned for and found the desired
    // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
    // connected we set the connected flag to be true.
    if (doConnect == true) {
      if (connectToServer()) {
        Serial.println("We are now connected to the BLE Server.");
        connected = true;
      } else {
        Serial.println("We have failed to connect to the server; there is nothin more we will do.");
      }
      doConnect = false;
    }
    delay(2000);
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
  }
}

/* Function to publish recieved message to MQTT topic */
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
    if ((topic.substring(0) == "XXXXX") || (topic.substring(0) == "YYYYY") || (topic.substring(0) == "ZZZZZ")) {
      return;
    }
    publishTo(topic.c_str(), payload.c_str());
  }
}

uint8_t data = 0;

void readSensor() {
  String mac = WiFi.macAddress();
  deviceMac = "";
  deviceMac += String(mac[0], HEX);     
  deviceMac += String(mac[1], HEX);
  deviceMac += String(mac[2], HEX);
  deviceMac += String(mac[3], HEX);
  deviceMac += String(mac[4], HEX);
  deviceMac += String(mac[5], HEX);

  /* Read temperature data and store them into struct vector */
  vectorData.deviceType = 'r';
  vectorData.type = 't';
  vectorData.value = dht.readTemperature();
  vectorData.devMac = deviceMac;
  vectorStruc.push_back(vectorData);
}

void BLEInit() {
  BLEDevice::init("Devkit-client");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449); // less or equal setInterval value
}

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  pClient->connect(myDevice);  
  Serial.println(" - Connected to server");

  if(pClient->isConnected()) {

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);
  }
}