# NMEA2000 Engine RPM, Fuel Level and Exhaust Temp Sender
This repository shows how to measure the Engine RPM, Fuel Level and the Exhaust Temeperature and send it as NNMEA2000 meassage.

![Prototype](https://github.com/AK-Homberger/NMEA2000-Fuel-and-Exhaust-Temperature-Sender/blob/master/NMEA2000%20Interface2.png)
# Version 0.2, 06.10.2019: Added Engine RPM.

# Version 0.1, 30.09.2019: Initial version.

The code is based on the NMEA 2000 library from Timo Lappalainen (https://github.com/ttlappalainen/NMEA2000).

The ESP32 in this project is an ESP32 NODE MCU from AzDelivery. Pin layout for other ESP32 devices might differ.

For the ESP32 CAN bus, I used the "Waveshare SN65HVD230 Can Board" as transceiver. It works well with the ESP32.
You have to define the correct GPIO ports in the header files for the NMEA2000 library (see documentation). For this project, I use the pins GPIO4 for CAN RX and GPIO2 for CAN TX. 

- The 12 Volt is reduced with a DC Step-Down_Converter (D24V10F5, https://www.pololu.com/product/2831).


- The Exhaust Temperature is measured with a DS18B20 Sensor.


- The Fuel Level is measured with TGT 200 device from manufacturer Philippi (https://www.philippi-online.de/en/products/supervision/tank-sensors.html). The resistor value from 0 tp 180 Ohm is measured with 1K resistor in row and translated to percent. The ADC value has to be calibrated in the code.


- The Engine RPM is measured on connection "W" of the generator/alternator. The Engine RPM is detected with a PC900V optocoupler device (http://www.simandl.cz/stranky/elektro/fastsci/soubory/pc900v.pdf).
This device plus the 2K resistor and the 1N4007 diode) translates the signal from "W" connction of generator to ESP32 pin 23. The diode is not critical an can be replaced with nearly any another type (e.g. 1N4148).
There is a RPM difference between generator and diesel engine RPM. The calibration value has to be set in the program.


Further updates will follow.

