# IR_Hub
IR proxy for controlling remote control functions on home devices from a mobile app using hqtt protocol.
This firmware is compatible with ESP8266 microcontrollers, and may be ported to other platforms changing the libraries used.

Main  Features:
1) Allows the control of multiple devices through IR commands (needs external hardware to the microcontroller to be able to drive IR leds)
2) All configuration is made using an internal web server using http
3) Commands are received via wifi using the MQTT protocol wich has low overhead and allows integration con external servers and voice activated devices like google home
4) Firmware can be updated over the air (OTA) which allows easy updates on installed devices
5) A function to contorl an RGB led strip is available but power transistors have to be used to allow th uController to drive it
6) Configuration and web pages are stored in non-volatile memory in json format
