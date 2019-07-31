/*
  Name: Topology based on ESPNOW communication - Slave WeMos - Light sensor
  Year: 2018/2019
  Author: Gabriela Chmelarova
  Purpose: ESPNOW Communication between a Master ESP32 and multiple ESP Slaves
  Description: This slave node periodically measure value of analog pin, from
  which we convert value to light specification, then send data via ESPNOW
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

extern "C" {
  #include <espnow.h>
}

// this is the MAC Address of the remote ESP server which receives these sensor readings
uint8_t remoteMac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};

#define WIFI_CHANNEL 1

/* Definiton of deep sleep variables */
#define SLEEP_SECS 10L  // 10 seconds

/* light sensor variables */
int sensorPin = A0; // select the input pin for LDR
int sensorValue = 0; // variable to store the value coming from the sensor

/* Variables for measuring time */
unsigned long bootMs, setupMs, sendMs, delMs;

bool callbackCalled = false;

uint32_t newVal = 0;

/* Struct for storing recieved ESPNOW message */
struct __attribute__((packed)) SENSOR_DATA {
  char  deviceType;
  char  type;
  float value;
} sensorData;

/* Function prototypes */
void gotoSleep();
void readSensor();
void sendData();
void manageMaster();

/* measurement of boot time */
RF_PRE_INIT() {
  bootMs = millis();
}

void setup() {
  setupMs = millis();
  Serial.begin(115200); Serial.println();
  Serial.println("**ESPNow/WeMos/light/Slave**");

  /* First we read sensor values */
  readSensor();

  /* oldValue is loaded from RTC memory 
    and compared with new value */
  uint32_t oldVal = 0;
  ESP.rtcUserMemoryRead(0, &oldVal, sizeof(oldVal));

  float diff = fabs(sensorData.value - oldVal);

  Serial.printf("new=%.2f ", sensorData.value);
  Serial.printf("old=%.2f ", oldVal);
  Serial.printf("diff=%.2f \n\n",diff);

  if (diff < 50) {
    Serial.println("Difference is smaller than 50");
    gotoSleep();
  } else {
    newVal = sensorData.value;
    ESP.rtcUserMemoryWrite(0, &newVal, sizeof(newVal));
  }

  /* If the value changed, we initialize ESP-NOW communication */
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_STA);
  /* initialization of ESPNOW 
   (this part was inspired from project mentioned under number 1a) in header) */*/
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

  sensorData.deviceType = 'w';
  sendData();

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
void readSensor() {
  sensorData.value = 0;
  sensorValue = analogRead(sensorPin);
  sensorData.value = sensorValue;
  sensorData.type = 'l';
}

/* Function to send sensor data to Master via ESPNOW */
void sendData() {
  uint8_t bs[sizeof(sensorData)];
  memcpy(bs, &sensorData, sizeof(sensorData));
  esp_now_send(NULL, bs, sizeof(sensorData)); // NULL means send to all peers
  sendMs = millis();
  Serial.print("Light value: "); Serial.println(sensorData.value);
}

/* Function to add master to known peers 
  (this part was inspired from project mentioned under number 1a) in header) */
void manageMaster() {
  // Master not paired, attempt pair
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(remoteMac, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);
}