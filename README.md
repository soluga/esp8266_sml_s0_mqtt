# esp8266_sml_s0_mqtt
ESP-Script for 
-Reading SML data from a digital energy counter (incl. CRC-Check)
-Counting S0 (impulses)
-Sending the data via MQTT.
-OTA Update

This is a basic INO script. It's not beauty and contains lot of debugging code.
It's ment for your reference if you plan to implement something like me.

Pull requests greatly appreciated. :)

I used 
-An ESP8266-12E (Wemos D1 Mini).
-PCF8575 (GPIO expander with 16 ports), connected to D1 (SCL), D2 (SDA) and D5 (Interrupt)
-Infrared-diode connected to D4
