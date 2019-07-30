/*
  Name: Topology based on ESPNOW communication - Slave LoRa - raindrop sensor
  Year: 2018/2019
  Author: Gabriela Chmelarova
  Purpose: ESPNOW Communication between a Master ESP32 and multiple ESP Slaves
  Description: This slave node periodically measure value of analog pin, from
  which we get current rain identification value, then send data via ESPNOW
  protocol to gateway and then it will put itself to deep sleep. After certain 
  Credits: this work was inspired on following projects
   1a) ESP-NOW communication - https://github.com/HarringayMakerSpace/ESP-Now
   1b) - https://navody.arduino-shop.cz/navody-k-produktum/bezdratova-komunikace-esp-now-s-esp32.html
   1c) - https://github.com/SensorsIot/ESP-Now-Tests
   1d) - https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/ESPNow
   3)  Deep-sleep usage - https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
   4)  DHT sensor - https://github.com/adafruit/DHT-sensor-library/blob/master/examples/DHTtester/DHTtester.ino
*/

#include <Adafruit_Sensor.h>
#include <esp_now.h>
#include <WiFi.h>

#include <esp_bt_main.h>  
#include <esp_bt.h>
#include <esp_wifi.h>

#include <DHT.h>

/* Definiton of deep sleep variables 
  (this part was inspired from project mentioned under number 3) in header) */
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10       /* Time ESP32 will go to sleep (in seconds) */

/* Variable for storing old measured value */
RTC_DATA_ATTR float oldTemp = 0;

/* DHT variables */
#define DHTTYPE DHT11 // DHT 11
const int dhtpin = 27;
// Initialize DHT sensor.
DHT dht(dhtpin, DHTTYPE);

/* Variables for comparing difference */
int sensorReading;
int range;

/* this is the MAC Address of the remote ESP server which receives these sensor readings */
const esp_now_peer_info_t remoteMac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};

#define CHANNEL 1
bool recieved = false;

/* Function prototypes */
void goToSleep();
void InitESPNow();
void configDeviceAP();
void readSensor(const char i);
void sendData(const char i);
void manageMaster();
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

/* Variables for measuring time */
unsigned long setupMs, sendMs, delMs;

/* Struct for storing recieved ESPNOW message 
 (this part was inspired from project mentioned under number 1a) in header) */
struct __attribute__((packed)) SENSOR_DATA {
  char  deviceType;
  char  type;
  float value;
} sensorData;

unsigned long bootMs = millis();

void setup() {
  setupMs = millis();
  // initialize serial communication at 9600 baud:
  Serial.begin(9600); Serial.println();
  Serial.println("**ESPNow/LoRa/DHT11/Slave**");

  /* First we configure the wake up source
  We set our ESP32 to wake up every TIME_TO_SLEEP seconds */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

  /* Then we read sensor values */
  dht.begin();
  readSensor('t');

  float diff = fabs(sensorData.value - oldTemp);

  Serial.printf("new=%.2f ", sensorData.value);
  Serial.printf("old=%.2f ", oldTemp);
  Serial.printf("diff=%.2f \n\n",diff);
    
  /* If the difference between old and new value is smaller than
    set threshold, put sensor back to sleep mode */
  if (diff < 0.2) {
    Serial.println("Difference is smaller than 0.2");
    oldTemp = sensorData.value;
    Serial.printf("old:%.2f new:%.2f\n", oldTemp, sensorData.value);
    goToSleep();
  }
  // store new measured value into RTC memory
  oldTemp = sensorData.value;

  /* If the value changed, we initialize ESP-NOW communication */
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_STA);
  // configure device AP mode
  // configDeviceAP();
  // Init ESPNow with a fallback logic
  InitESPNow();
  /* Once ESPNow is successfully Init, we will register for recv CB to
   get recv packet info. */
  esp_now_register_recv_cb(OnDataRecv);
  /* Here we register for send CB */
  esp_now_register_send_cb(OnDataSent);

  // here we add master to known peers
  manageMaster();
  // here we set device type indikator
  sensorData.deviceType = 'l';

  // than we send temperature and humidity data
  sendData('t');
  sendData('h');
  //after that we start deep-sleep
  goToSleep();
}

void loop() {
  // do nothing
}

void goToSleep() {
  /* Prepare platform to sleep mode and start deep sleep */
  Serial.println("Going to sleep now");
  Serial.printf("Setup: %lu ms, send: %lu ms, delivered: %lu, now %lu ms, going to sleep for %i secs...\n", setupMs, sendMs, delMs, millis(), TIME_TO_SLEEP);
  esp_deep_sleep_start();
}

/* Function to establish ESPNOW communication */
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  String Prefix = "Slave:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. AP: " + String(SSID));
  }
}

/* Function to read sensor value */
void readSensor(const char i) {
  sensorData.value = 0;
  if (i == 't') {
    sensorData.type = 't';
    sensorData.value = dht.readTemperature();
  } else if (i == 'h') {
    sensorData.type = 'h';
    sensorData.value = dht.readHumidity();
  }
}

/* Function to send sensor data to Master via ESPNOW */
void sendData(const char i) {
  readSensor(i);
  uint8_t bs[sizeof(sensorData)];
  memcpy(bs, &sensorData, sizeof(sensorData));
  sendMs = millis();
  esp_now_send(NULL, bs, sizeof(sensorData)); // NULL means send to all peers
  Serial.printf(" DeviceType=%c, ValueType=%c, Status=%.2f\n", 
  sensorData.deviceType, sensorData.type, sensorData.value);
}

/* Function to add master to known peers 
  (this part was inspired from project mentioned under number 1a) in header) */
void manageMaster() {
  // Master not paired, attempt pair
  esp_err_t addStatus = esp_now_add_peer(remoteMac);
  if (addStatus == ESP_OK) {
    // Pair success
    Serial.println("remote MAC added");
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
}

/* Callback for processing recieved data from Master 
  (this part was inspired from project mentioned under number 1d) in header) */
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  delMs = millis();
  recieved = true;
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); Serial.println(*data);
  Serial.println("");
}

/* Callback when data is sent from Slave to Master 
  (this part was inspired from project mentioned under number 1d) in header) */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}