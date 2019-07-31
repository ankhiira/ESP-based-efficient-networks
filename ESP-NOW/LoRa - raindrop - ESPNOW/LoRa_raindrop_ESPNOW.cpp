/*
  Name: Topology based on ESPNOW communication - Slave LoRa - raindrop sensor
  Year: 2018/2019
  Author: Gabriela Chmelarova
  Purpose: ESPNOW Communication between a Master ESP32 and multiple ESP Slaves
  Description: This slave node periodically measure value of analog pin, from
  which we get current rain identification value, then send data via ESPNOW
  protocol to gateway and then it will put itself to deep sleep. After certain 
  amount of time will ESP wake itself and start new cycle.
  Credits: this work was inspired on following projects
   1a) ESP-NOW communication - https://github.com/HarringayMakerSpace/ESP-Now
   1c) - https://github.com/SensorsIot/ESP-Now-Tests
   1d) - https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/ESPNow
   3)  Deep-sleep usage - https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
   4)  Raindrop sensor - https://www.c-sharpcorner.com/article/finding-weather-conditions-using-rain-sensor-with-arduino-me/
*/

#include <Adafruit_Sensor.h>
#include <esp_now.h>
#include <WiFi.h>

/* Definiton of deep sleep variables 
  (this part was inspired from project mentioned under number 3) in header) */
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10       /* Time ESP32 will go to sleep (in seconds) */

/* Variable for storing old measured value */
RTC_DATA_ATTR int oldVal;

/* lowest and highest sensor readings:
  (this part was inspired from project mentioned under number 4) in header) */
const int sensorMin = 600;   // sensor minimum
const int sensorMax = 5000;  // sensor maximum

// select the input pin for raindrop sensor
int analogPin = 27; 
String rainValue;

/* Variables for comparing difference in measured values */
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
void readSensor();
void sendData();
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

void setup() {
  setupMs = millis();
  // initialize serial communication at 9600 baud:
  Serial.begin(9600); Serial.println(); 
  Serial.println("**ESPNow/LoRa/raindrop/Slave**");

  /* First we configure the wake up source
    We set our ESP32 to wake up every TIME_TO_SLEEP seconds 
    (this part was inspired from project mentioned under number 3) in header) */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

  /* Then we read sensor value 
   - read the sensor on analog A0: */
  sensorReading = analogRead(analogPin);
  /* map the sensor range (four options):
    ex: 'long int map(long int, long int, long int, long int, long int)' */
	range = map(sensorReading, sensorMin, sensorMax, 0, 3);

  Serial.printf("new=%i ", range);
  Serial.printf("old=%i ", oldVal);
 
  /* If there is a difference between old and new 
    value, put sensor back to sleep mode */
  if (range == oldVal) {
    Serial.println("- Old value didn't change");
    goToSleep();
  }
  // store new measured value into RTC memory
  oldVal = range;

  /* If the value changed, we initialize ESP-NOW communication */
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_STA);
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

  // than we send rain value
  sendData();
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

/* Function to establish ESPNOW communication 
  (this part was inspired from project mentioned under number 1a) in header) */
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

/* Function to read sensor value 
  (this part was inspired from project mentioned under number 4) in header) */
void readSensor() {
  // range value:
  switch (range) {
    case 0:    // Sensor getting wet
      Serial.println("Flood");
      break;
    case 1:    // Sensor getting wet
      Serial.println("Rain Warning");
      break;
    case 2:    // Sensor dry - To shut this up delete the " Serial.println("Not Raining"); " below.
      Serial.println("Not Raining");
      break;
  }
  sensorData.value = range;
  sensorData.type = 'r';
}

/* Function to send sensor data to Master via ESPNOW */
void sendData() {
  readSensor();
  uint8_t bs[sizeof(sensorData)];
  memcpy(bs, &sensorData, sizeof(sensorData));
  esp_err_t result = esp_now_send(NULL, bs, sizeof(sensorData)); // NULL means send to all peers
  sendMs = millis();
  Serial.printf(" DeviceType=%c, ValueType=%c, Status=%0.2f\n", 
  sensorData.deviceType, sensorData.type, sensorData.value);
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
}

/* Function to add master to known peers */
void manageMaster() {
  // Master not paired, attempt pair
  esp_err_t addStatus = esp_now_add_peer(remoteMac);
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
}

/* Callback for processing recieved data from Master */
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

/* Callback when data is sent from Slave to Master */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}