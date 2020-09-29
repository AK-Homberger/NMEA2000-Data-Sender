# NMEA2000 Battery Voltage, Engine RPM, Fuel Level and Exhaust Temp Sender
This repository shows how to measure the Battery Voltage, Engine RPM, Fuel Level and the Exhaust Temeperature and send it as NNMEA2000 meassage.

![Picture](https://github.com/AK-Homberger/NMEA2000-Data-Sender/blob/master/NMEA2000%20DataSender.png)


The project requires the NMEA2000 and the NMEA2000_esp32 libraries from Timo Lappalainen: https://github.com/ttlappalainen.
Both libraries have to be downloaded and installed.

The ESP32 in this project is an ESP32 NODE MCU from AzDelivery. Pin layout for other ESP32 devices might differ.

For the ESP32 CAN bus, I used the "Waveshare SN65HVD230 Can Board" as transceiver. It works well with the ESP32.
The correct GPIO ports are defined in the main sketch. For this project, I use the pins GPIO4 for CAN RX and GPIO5 for CAN TX. 

The 12 Volt is reduced to 5 Volt with a DC Step-Down_Converter (D24V10F5, https://www.pololu.com/product/2831).

The following values are measured and transmitted to the NMEA2000 bus:

- The Exhaust Temperature is measured with a DS18B20 Sensor (the DallasTemperature library has to be installed with the Arduiono IDE Library Manager).


- The Fuel Level is measured with TGT 200 device from manufacturer Philippi (https://www.philippi-online.de/en/products/supervision/tank-sensors.html). The resistor value from 5 tp 180 Ohm is measured with 1K resistor in row and translated to percent. The ADC value has to be calibrated in the code.

- The Engine RPM is measured on connection "W" of the generator/alternator. The Engine RPM is detected with a H11L1 optocoupler device (or alternatively a PC900v). This device plus the 2K resistor and the 1N4148 diode) translates the signal from "W" connection of the generator to ESP32 GPIO pin 23. The diode is not critical an can be replaced with nearly any another type.
There is a RPM difference between generator and diesel engine RPM. The calibration value has to be set in the program.

- The Battery Voltage is measured at GPIO pin 35 (check calibration value with regards to the real resistor values of R4/R5).

The following PGNs are sent to the NMEA 2000 Bus:
- 127505 Fluid Level
- 130311 Temperature (or alternatively 130312, 130316)
- 127488 Engine Rapid / RPM
- 127508 Battery Status

Change the PGNs if your MFD can not show a certain PGN.
BTW: The full list of PGNs is defined in this header file of the NMEA 2000 library: https://github.com/ttlappalainen/NMEA2000/blob/master/src/N2kMessages.h

# Partlist:

- ESP32 [Link](https://www.amazon.de/AZDelivery-NodeMCU-Development-Nachfolgermodell-ESP8266/dp/B071P98VTG/ref=sxts_sxwds-bia-wc-drs3_0?__mk_de_DE=%C3%85M%C3%85%C5%BD%C3%95%C3%91&cv_ct_cx=ESP32&dchild=1&keywords=ESP32) 
- SN65HVD230 [Link](https://www.amazon.de/SN65HVD230-Board-Connecting-Communication-Development/dp/B00KM6XMXO/ref=sxts_sxwds-bia-wc-drs1_0?__mk_de_DE=%C3%85M%C3%85%C5%BD%C3%95%C3%91&cv_ct_cx=SN65HVD230&dchild=1&keywords=SN65HVD230&pd_rd_i=B00KM6XMXO&pd_rd_r=0000ea9b-16c8-4bfc-bb40-b71623633214&pd_rd_w=VecN7&pd_rd_wg=VRb2Q&pf_rd_p=578deb70-f9b7-4aa5-9f96-98765f2717c8&pf_rd_r=H8X4ND0GD8MN6WH9H17A&psc=1&qid=1601309172&s=industrial&sr=1-1-5a42e879-3844-4142-9c14-e77fe027c877)
- D24V10F5 [Link](https://eckstein-shop.de/Pololu-5V-1A-Step-Down-Spannungsregler-D24V10F5)
- Resistor 3,3KOhm [Link](https://www.reichelt.de/widerstand-kohleschicht-3-3-kohm-0207-250-mw-5--1-4w-3-3k-p1397.html?search=widerstand+250+mw+3k3) Other resistors are the same type!
- Transistor BC547 [Link](https://www.reichelt.de/bipolartransistor-npn-45v-0-1a-0-5w-to-92-bc-547b-dio-p219082.html?search=BC547)
- Diode 1N4148 [Link](https://www.reichelt.de/schalt-diode-100-v-150-ma-do-35-1n-4148-p1730.html?search=1n4148)
- Zenerdiode 3,3 V [Link](https://www.reichelt.de/zenerdiode-3-3-v-0-5-w-do-35-zf-3-3-p23126.html?&trstct=pos_6&nbc=1)
- H11-L1 [Link](https://www.reichelt.de/optokoppler-1-mbit-s-dil-6-h11l1m-p219351.html?search=H11-l1)
- PCB universal set [Link](https://www.amazon.de/70Stk-Doppelseitig-Lochrasterplatte-Kit-Lochrasterplatine/dp/B07BDKG68Q/ref=sr_1_6?adgrpid=70589021505&dchild=1&gclid=EAIaIQobChMI07qXtuaN7AIVjentCh3xPg80EAAYASAAEgK_-_D_BwE&hvadid=352809599274&hvdev=c&hvlocphy=9043858&hvnetw=g&hvqmt=e&hvrand=11402952735368332074&hvtargid=kwd-300896600841&hydadcr=26892_1772693&keywords=lochrasterplatine&qid=1601363175&sr=8-6&tag=googhydr08-21)


# Updates:

Version 0.6, 04.08.2020: Changed TX pin from 2 to 5. Store/restore NodeAddress. Lower task priority (GetTemperature()).

Version 0.5, 15.02.2020: Added Battery Voltage and optional temperature PGNs.

Version 0.4, 14.12.2019: Added CAN pin definition in sketch.

Version 0.3, 18.10.2019: Improved Chip ID calculation.

Version 0.2, 06.10.2019: Added Engine RPM.

Version 0.1, 30.09.2019: Initial version.
