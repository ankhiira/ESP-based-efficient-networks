/*
  Name: Topology based on BLE communication - Server
  Year: 2018/2019
  Author: Gabriela Chmelarova
  Purpose: BLE Communication between Client ESP32 and multiple ESP Servers
  Credits: this work was inspired on following projects
   1) BLE communication - https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE
   2) Deep-sleep usage - https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
   3) DHT sensor - https://github.com/adafruit/DHT-sensor-library/blob/master/examples/DHTtester/DHTtester.ino
*/

#include "Adafruit_Sensor.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "DHT.h"

#include <WiFi.h>
#include <esp_wifi.h>


/* define DHT pins */
#define DHTTYPE DHT11 // DHT 11
const int dhtpin = 17;
DHT dht(dhtpin, DHTTYPE); // Initialize DHT sensor.


/* Deep sleep variables */
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10       /* Time ESP32 will go to sleep (in seconds) */

#define LED_BUILTIN 2

/* Variables for measuring time */
unsigned long bootMs, setupMs, sendMs, sentMs, delMs;

/* BLE variables */
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;

bool deviceConnected = false;
bool oldDeviceConnected = false;
bool canDisconnect = false;

uint8_t txValue = 0;
uint8_t data = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    delMs = millis();
    deviceConnected = false;

    // On disconnect we start deep sleep
    Serial.println("Going to sleep now");
    Serial.printf("Boot: %lu ms, setup: %lu ms, send: %lu ms, delivery: %lu ms, now %lu ms, going to sleep for %i secs...\n", bootMs, setupMs, sendMs, delMs, millis(), TIME_TO_SLEEP);
    Serial.flush(); 
    esp_deep_sleep_start();
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i]);

      Serial.println();
      Serial.println("*********");
    }
  }
};

/* function prototypes */
void goToSleep();
void BLEInit();

void setup() {
  setupMs = millis();
  Serial.begin(9600); Serial.println();

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup BLE to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

  dht.begin();
  BLEInit();
  data = dht.readTemperature();
}

int timer;

void loop() {

  if (millis() - timer > 5000) {

    pTxCharacteristic->setValue(&data, 1);
    pTxCharacteristic->notify();
    sendMs = millis();
    Serial.println(data);
  
    if (deviceConnected) {
      pTxCharacteristic->setValue(&data, 1);
      pTxCharacteristic->notify();
      sendMs = millis();
      Serial.println(data);
    }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
    }

    // connecting
    if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
    }
  } else {
    goToSleep();
  }

  // bluetooth stack will go into congestion, if too many packets are sent
  delay(2000);
}

void goToSleep() {
    /* Prepare platform to sleep mode and start deep sleep */
  Serial.println("Going to sleep now");
  Serial.printf("Boot: %lu ms, setup: %lu ms, send: %lu ms, sent: %lu, delivered: %lu, now %lu ms, going to sleep for %i secs...\n", bootMs, setupMs, sendMs, sentMs, delMs, millis(), TIME_TO_SLEEP);
  Serial.flush();
  BLEDevice::deinit();
  esp_deep_sleep_start();
}

void BLEInit() {
  // Create the BLE Device
  BLEDevice::init("r");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE send Characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  // Create a BLE recieve Characteristic
  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->addServiceUUID(BLEUUID(SERVICE_UUID));
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}