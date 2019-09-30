# NMEA2000 Fuel Level and Exhaust Temp Sender
This repository shows how to measure the Fuel Level and the Exhaust Temeperature and send it as NNMEA2000 meassage.

![Prototype](https://github.com/AK-Homberger/NMEA-Fuel-and-Exhaust-Temperature-Sender/blob/master/Bild.png)

Version 0.1, 30.09.2019: Initial version.

The code is based on the NMEA 2000 library from Timo Lappalainen (https://github.com/ttlappalainen/NMEA2000).

The ESP32 in this project is an ESP32 NODE MCU from AzDelivery. Pin layout for other ESP32 devices might differ.

For the ESP32 CAN bus, I used the "Waveshare SN65HVD230 Can Board" as transceiver. It works well with the ESP32.
You have to define the correct GPIO ports in the header files for the NMEA2000 library (see documentation). For the Gateway, I use the pins GPIO4 for CAN RX and GPIO2 for CAN TX. 

- The Fuel Level is measured with TGT 200 device from manufacturer Philippi (https://www.philippi-online.de/en/products/supervision/tank-sensors.html). The resistor value from 0 tp 180 Ohm is measured with 1K resistor in row and translated to percent. The ADC value has to be calibrated in the code.

- The Exhaust Temperature is measured with a DS18B20 Sensor.


Further updates will follow.

