/*
  Name: Topology based on ESPNOW communication - Slave WeMos - DHT11
  Year: 2018/2019
  Author: Gabriela Chmelarova
  Purpose: ESPNOW Communication between a Master ESP32 and multiple ESP Slaves
  Description: This slave node periodically measure value of DHT sensor, from
  which we get current temperature and humidity, then send data via ESPNOW
  protocol to gateway and then it will put itself to deep sleep. After certain 
  amount of time will ESP wake itself and start new cycle.
  Credits: this work was inspired on following projects
   1a) ESP-NOW communication - https://github.com/HarringayMakerSpace/ESP-Now
   1c) - https://github.com/SensorsIot/ESP-Now-Tests
   1d) - https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/ESPNow
   4)  DHT sensor - https://github.com/adafruit/DHT-sensor-library/blob/master/examples/DHTtester/DHTtester.ino
*/

#include <Adafruit_Sensor.h>
#include <ESP8266WiFi.h>
#include <DHT.h>

extern "C" {
  #include <espnow.h>
}

// this is the MAC Address of the remote ESP server which receives these sensor readings
uint8_t remoteMac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};

#define WIFI_CHANNEL 1

/* Definiton of deep sleep variables */
#define SLEEP_SECS 10L  // 10 seconds

/* DHT variables */
#define DHTTYPE DHT11 // DHT 11
const int dhtpin = D2;
// Initialize DHT sensor.
DHT dht(dhtpin, DHTTYPE);

/* Variables for measuring time */
unsigned long bootMs, setupMs, sendMs, delMs;

bool callbackCalled = false;

uint32_t newVal = 0;

/* Struct for storing recieved ESPNOW message 
   (this part was inspired from project mentioned under number 1a) in header) */
struct __attribute__((packed)) SENSOR_DATA {
  char  deviceType;
  char  type;
  float value;
} sensorData;

/* Function prototypes */
void gotoSleep();
void readSensor(const char i);
void sendData(const char i);
void manageMaster();

/* measurement of boot time */
RF_PRE_INIT() {
  bootMs = millis();
}

void setup() {
  setupMs = millis();
  Serial.begin(115200); Serial.println();
  Serial.println("**ESPNow/WeMos/DHT11/Slave**");

  /* First we read sensor values */
  dht.begin();
  readSensor('t');

  /* oldValue is loaded from RTC memory 
    and compared with new value */
  uint32_t oldVal = 0;
  ESP.rtcUserMemoryRead(0, &oldVal, sizeof(oldVal));

  /* If the difference between old and new value is smaller than
    set threshold, put sensor back to sleep mode */
  float diff = fabs(sensorData.value - oldVal);

  Serial.printf("new=%.2f ", sensorData.value);
  Serial.printf("old=%.2f ", oldVal);
  Serial.printf("diff=%.2f \n\n",diff);

  if (diff < 0.2) {
    Serial.println("Difference is smaller than 0.2");
    Serial.printf("old:%.2f new:%.2f\n", oldVal, sensorData.value);
    gotoSleep();
  } else {
    // store new measured value into RTC memory
    newVal = sensorData.value;
    ESP.rtcUserMemoryWrite(0, &newVal, sizeof(newVal));
  }
  
  /* If the value changed, we initialize ESP-NOW communication */
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_STA);
  /* initialization of ESPNOW */
  if (esp_now_init() != 0) {
    Serial.println("ESPNow Init Failed");
    gotoSleep();
  } else {
    Serial.println("ESPNow Init Success");
  }

  manageMaster();

  // registration of send callback
  esp_now_register_send_cb([](uint8_t* mac, uint8_t sendStatus) {
    Serial.printf("send_cb, send done, status = %i\n", sendStatus);
    Serial.printf("Boot: %lu ms, setup: %lu ms, send: %lu ms, delivered: %lu, now %lu ms\n", bootMs, setupMs, sendMs, delMs, millis());
  });

  esp_now_register_recv_cb([](uint8_t* mac, uint8_t* data, uint8_t len) {
    delMs = millis();
    callbackCalled = true;
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.print("Last Packet Recv from: "); Serial.println(macStr);
    Serial.print("Last Packet Recv Data: "); Serial.println(*data);
    Serial.println("");
  });

  // here we set device type indikator
  sensorData.deviceType = 'w';

  // than we send temperature and humidity data
  sendData('t');
  sendData('h');
  
  //after that we start deep-sleep
  /* Prepare platform to sleep mode and start deep sleep */
  Serial.println("Going to sleep now");
  gotoSleep(); 
}

void loop() {
 // do nothing
}

void gotoSleep() { 
  int sleepSecs = SLEEP_SECS; 
  Serial.printf("Boot: %lu ms, setup: %lu ms, send: %lu ms, delivered: %lu, now %lu ms, going to sleep for %i secs...\n", bootMs, setupMs, sendMs, delMs, millis(), sleepSecs);
  ESP.deepSleep(sleepSecs * 1000000); //on wake up DISABLE the modem. So for example I can't connect the esp to wifi.
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

/* Function to add master to known peers */
void manageMaster() {
  // Master not paired, attempt pair
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(remoteMac, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);
}