# NMEA2000 Battery Voltage, Engine RPM, Fuel Level and Exhaust Temp Sender
This repository shows how to measure the Battery Voltage, Engine RPM, Fuel Level and the Exhaust Temeperature and send it as NNMEA2000 meassage.

![Picture](https://github.com/AK-Homberger/NMEA2000-Data-Sender/blob/master/NMEA2000%20DataSender.png)


The project requires the NMEA2000 and the NMEA2000_esp32 libraries from Timo Lappalainen: https://github.com/ttlappalainen
Both libraries have to be downloaded and installed.

The ESP32 in this project is an ESP32 NODE MCU from AzDelivery. Pin layout for other ESP32 devices might differ.

For the ESP32 CAN bus, I used the "Waveshare SN65HVD230 Can Board" as transceiver. It works well with the ESP32.
The correct GPIO ports are defined in the main sketch. For this project, I use the pins GPIO4 for CAN RX and GPIO2 for CAN TX. 

- The 12 Volt is reduced to 5 Volt with a DC Step-Down_Converter (D24V10F5, https://www.pololu.com/product/2831).


- The Exhaust Temperature is measured with a DS18B20 Sensor (the DallasTemperature library has to be installed with the Arduiono IDE Library Manager).


- The Fuel Level is measured with TGT 200 device from manufacturer Philippi (https://www.philippi-online.de/en/products/supervision/tank-sensors.html). The resistor value from 5 tp 180 Ohm is measured with 1K resistor in row and translated to percent. The ADC value has to be calibrated in the code.

- Tha Battery Voltage is measuer at GPIO pin 35 (check kalobration value with regards to the real resitor values for (R4/R5).


- The Engine RPM is measured on connection "W" of the generator/alternator. The Engine RPM is detected with a H11L1 optocoupler device (or alternatively a PC900v). This device plus the 2K resistor and the 1N4148 diode) translates the signal from "W" connection of the generator to ESP32 pin 23. The diode is not critical an can be replaced with nearly any another type.
There is a RPM difference between generator and diesel engine RPM. The calibration value has to be set in the program.

Tho following PGNs are send to the NMEA 2000 Bus:
- 127505 Fluid Level
- 130312 Temperature
- 127488 Engine Rapid / RPM
- 127508 Battery Status

Change the PGNs isf your MFD can not show a certain PGN.


# Updates:

Version 0.5, 15.02.2020: Added Battery Voltage.

Version 0.4, 14.12.2019: Added CAN pin definition in sketch.

Version 0.3, 18.10.2019: Improved Chip ID calculation.

Version 0.2, 06.10.2019: Added Engine RPM.

Version 0.1, 30.09.2019: Initial version.
