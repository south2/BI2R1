# BI2: bodyinteraction IOT sex toy (revision 1)

This is the code for a ESP8266 module for controlling a vibration motor and reading movement data. 
The hardware uses Wemos mini modules. The motor is controlled by movements. 

In addition a local web server is started to allow control of the toy over a web based user interface.   

The movement data will be send to a arbitrary MQTT server. Data and commands from the MQTT server send to the toy 
will be reveived and processed. 

Additional a node-RED interface for displaying the toy's data and for controlling the toy is available. 
The node-RED interface connects to the toy via MQTT.

How-to: https://bodyinteraction.com/2017/03/23/basic-node-for-the-internet-of-sex-toys-part-1/
More information: www.bodyinteraction.com
