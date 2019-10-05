# esp8266_sml_s0_mqtt
ESP-Script for<br>
-Reading SML data from a digital energy counter (incl. CRC-Check)<br>
-Counting S0 (impulses)<br>
-Sending the data via MQTT.<br>
-OTA Update<br>
<br>
This is a basic INO script. It's not beauty and contains lot of debugging code.<br>
It's ment for your reference if you plan to implement something like me.<br>
<br>
Pull requests greatly appreciated. :)<br>
<br>
I used<br>
-An ESP8266-12E (Wemos D1 Mini).<br>
-PCF8575 (GPIO expander with 16 ports), connected to D1 (SCL), D2 (SDA) and D5 (Interrupt)<br>
-Infrared-diode OSRAM SFH309FA (880-1120nm/35V) connected to D4<br>
