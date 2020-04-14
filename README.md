# ESP-based-efficient-networks
Three network communication examples of ESP-NOW, BLE and LoRa technology using optimized energy-saving ESP32 and ESP8266

This repository deals with construction of three star network topologies based on different ESP based platforms. Each topology communicates via different protocol mentioned below. 

# Communication protocols
* Topology 1: ESP-NOW
* Topology 2: BLE
* Topology 3: LoRa

# Topology 1 sensors (ESP-NOW)
* ESP32-DevKit - gateway
* LILYGO TTGO LoRa - DHT11
* LILYGO TTGO LoRa - MH-RD rain module with YL-83
* WeMos D1 mini - DHT11
* WeMos D1 mini - light module with LDR 

# Topology 2 sensors (BLE)
* ESP32-DevKit - gateway
* LILYGO TTGO LoRa - DHT11
* LILYGO TTGO LoRa - DHT22
* Lolin32 - CJMCU-906 object temperature module
* WeMos WiFi and Bluetooth Battery - DHT11

# Topology 2 sensors (LoRa)
* LILYGO TTGO LoRa - gateway
* LILYGO TTGO LoRa - DHT11
* LILYGO TTGO LoRa - DHT22
* LILYGO TTGO LoRa - MH-RD rain module with YL-83
* LILYGO TTGO LoRa - light module with LDR 


# Result on MQTT
![alt text](https://github.com/4Gabby4/ESP-based-efficient-networks/blob/master/fig/topic_shiftr.png)

# Graph of theoretical battery life of sensors
The intervals in this graph indicate the frequency of sensor measurements (their wake-up frequency).
![alt text](https://github.com/4Gabby4/ESP-based-efficient-networks/blob/master/fig/graph_consumtion.png)

As you can see from the graph, the best results got devices based on WeMos platform with ESP8266 chip, communicating via ESP-NOW. But I need to add, that those results highly depends on the platform, you choose. I also measured devices communicating via ESP-NOW based on LoRa platform (which includes ESP32 microchip) and the results were comparable with the devices communicating via BLE protocol.
\You can also see that there isn't that big difference between measurement interval of one hour or one minute. This is caused by very long sleep intervals in proportion to really short wake up periods (in matter of milliseconds).
