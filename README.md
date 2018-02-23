# ESP MQTT Multisensor

This project shows a super easy way to get started with your own DIY Multisensor to use with [Home Assistant](https://home-assistant.io/), a sick, open-source Home Automation platform that can do just about anything. 

Bonus, this project requires **no soldering** and **no breadboards** - just header wires and the development board! 

Video Tutorial - https://youtu.be/jpjfVc-9IrQ

The code covered in this repository utilizies Home Assistant's [MQTT JSON Light Component](https://home-assistant.io/components/light.mqtt_json/), [MQTT Sensor Component](https://home-assistant.io/components/sensor.mqtt/), and a [NodeMCU ESP8266](http://geni.us/cpmi) development board. 

[Original Code by BruhAutomation](https://github.com/bruhautomation/ESP-MQTT-JSON-Multisensor)

[Modified by Shawn Corey](https://github.com/ShawnCorey/ESP-MQTT-Multisensor-Authdiscovery)


### Supported Features Include
- **DHT22** temperature sensor
- **DHT 22** humdidity sensor
- **AM312** PIR motion sensor 
- **photoresistor** or **TEMT600** light sensor
- **RGB led** with support for color, flash, fade, and transition
- **Over-the-Air (OTA)** upload from the ArduinoIDE


#### OTA Uploading
This code also supports remote uploading to the ESP8266 using Arduino's OTA library. To utilize this, you'll need to first upload the sketch using the traditional USB method. However, if you need to update your code after that, your WIFI-connected ESP chip should show up as an option under Tools -> Port -> Porch at your.ip.address.xxx. More information on OTA uploading can be found [here](http://esp8266.github.io/Arduino/versions/2.0.0/doc/ota_updates/ota_updates.html). Note: You cannot access the serial monitor over WIFI at this point.  


### Parts List

**Amazon Prime (fast shipping)**
- [NodeMCU 1.0](http://geni.us/cpmi)
- [DHT22 Module](http://geni.us/vAJWMXo)
- [LDR Photoresistor Module](http://geni.us/O0AO0)
    OR
- [TEMT6000](http://geni.us/aRYe)
- [Power Supply](http://geni.us/ZZ1r)
- [Common Cathode RGB Led](http://geni.us/nFcB)
- [Header Wires](http://geni.us/pvFNG)
- [AM312 Mini PIR Sensor](http://geni.us/dbGQ)

**Aliexpress (long shipping = cheap prices)**
- [NodeMCU 1.0](http://geni.us/EfYA)
- [DHT22 Module](http://geni.us/35Np8H)
- [LDR Photoresistor Module](http://geni.us/O5iv)
    OR
- [TEMT6000](http://geni.us/xAuLoy)
- [Power Supply](http://geni.us/NSYjvb)
- [Common Cathode RGB Led](http://geni.us/OfHbhZb)
- [Header Wires](http://geni.us/Iv6p9)
- [AM312 Mini PIR Sensor](http://geni.us/WBKyxhx)


### Wiring Diagram
![alt text](https://github.com/bruhautomation/ESP-MQTT-JSON-Multisensor/blob/master/wiring_diagram_v2.png?raw=true "Wiring Diagram")


### 3D Printed Enclosure
In an effort to make the sensor less ugly, I designed an enclosure in 123D Design and uploaded the [STL file](https://github.com/bruhautomation/ESP-MQTT-JSON-Multisensor/blob/master/BRUH%20Multisensor%20V1.stl) in case you want to print your own. It's also availible on [Thingiverse](http://www.thingiverse.com/thing:2239142). I printed mine on a [Prusa I3 clone](https://www.youtube.com/watch?v=PLRdMtZVQfQ) with a layer height of 0.2 mm, 40% infill, and no supports in [ESUN PLA](http://geni.us/GS3U) and it turned out great. 

Alternatively, you can also make your own enclosure by hand using something like [Instamorph](http://geni.us/BtidLG3). It's themoplastic that melts in hot water and then solidifies to hard plastic at room temperature. You can even get [pigment packs](http://geni.us/dNTi) and take it next level. I, personally, suck at using it, but it's cheap and functional. 

Of course, you can use a project box, tupperware, a card board box, or skip the enclosure all together. 

![alt text](https://github.com/bruhautomation/ESP-MQTT-JSON-Multisensor/blob/master/BRUH%20Multisensor%20V1.PNG?raw=true "Enclosure")


