/*  
  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// Version 0.1, 30.09.2019, AK-Homberger

#include <Arduino.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <memory>
#include <N2kMessages.h>
#include <DallasTemperature.h>

#define ENABLE_DEBUG_LOG 0 // Debug log

#define ADC_Calibration_Value 250.0 // For resitor measure 5 Volt and 180 Ohm 100% plus IK resitor.


// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM={127505L, // Fluid Level
                                                130312L, // Temperature
                                                0};
                                                
// Data wire for teperature (Dallas DS18B20) is plugged into GPIO 13 on the ESP32
#define ONE_WIRE_BUS 13
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// Send time offsets
#define TempSendOffset 0
#define TankSendOffset 40
#define SlowDataUpdatePeriod 1000  // Time between CAN Messages sent

// Tank fluid level measure is connected GPIO 34 (Analog ADC1_CH6) 
const int ADCpin = 34;
float FuelLevel=0;
float ExhaustTemp=0;

// Task handle for OneWire read (Core 0 on ESP32)
TaskHandle_t Task1;

// Serial port 2 config (GPIO 16)
const int baudrate = 38400;
const int rs_config = SERIAL_8N1;

  
void debug_log(char* str) {
#if ENABLE_DEBUG_LOG == 1
   Serial.println(str);
#endif
}

    
void setup() {

   uint8_t chipid[6];
   char ID[10]="01";
   
// Init USB serial port
   Serial.begin(115200);

// Start OneWire
   sensors.begin();

// Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
  
   NMEA2000.SetN2kCANMsgBufSize(8);
   NMEA2000.SetN2kCANReceiveFrameBufSize(250);
   NMEA2000.SetN2kCANSendFrameBufSize(250);

   esp_efuse_read_mac(chipid);
   snprintf(ID, sizeof(ID),"%X", chipid);

   
// Set product information
   NMEA2000.SetProductInformation(ID, // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "My Sensor Module",  // Manufacturer's Model ID
                                 "1.0.2.25 (2019-07-07)",  // Manufacturer's Software version code
                                 "1.0.2.0 (2019-07-07)" // Manufacturer's Model version
                                 );
// Set device information
   NMEA2000.SetDeviceInformation(1, // Unique number. Use e.g. Serial number.
                                132, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25, // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );
  
// If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below

   NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
   NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,32);

   NMEA2000.ExtendTransmitMessages(TransmitMessages);
        
   NMEA2000.Open();

// Create task for core 0, loop() runs on core 1
   xTaskCreatePinnedToCore(
      GetTemperature, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      2,  /* Priority of the task */
      &Task1,  /* Task handle. */
      0); /* Core where the task should run */
      
   delay(200);  
}

// This task runs isolated on core 0 because sensors.requestTemperatures() is slow and blocking for about 750 ms
  void GetTemperature( void * parameter) {
    float tmp=0;
    for(;;) {
    sensors.requestTemperatures(); // Send the command to get temperatures
    tmp = sensors.getTempCByIndex(0);
    if(tmp!=-127) ExhaustTemp=tmp;
    delay(200);
    }
  }


bool IsTimeToUpdate(unsigned long NextUpdate) {
  return (NextUpdate<millis());
}
unsigned long InitNextUpdate(unsigned long Period, unsigned long Offset=0) {
  return millis()+Period+Offset;
}

void SetNextUpdate(unsigned long &NextUpdate, unsigned long Period) {
  while ( NextUpdate<millis() ) NextUpdate+=Period;
}


void SendN2kTankLevel(double level, double capacity) {
  static unsigned long SlowDataUpdated=InitNextUpdate(SlowDataUpdatePeriod, TankSendOffset);
  tN2kMsg N2kMsg;
  
  if ( IsTimeToUpdate(SlowDataUpdated) ) {
    SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);

    Serial.printf("Fuel Level  : %3.0f ", level);
    Serial.println("%");
    
    SetN2kFluidLevel(N2kMsg,0,N2kft_Fuel, level, capacity );
    NMEA2000.SendMsg(N2kMsg);
  }
}


void SendN2kExhaustTemp(double temp) {
  static unsigned long SlowDataUpdated=InitNextUpdate(SlowDataUpdatePeriod, TempSendOffset);
  tN2kMsg N2kMsg;
  
  if ( IsTimeToUpdate(SlowDataUpdated) ) {
    SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);

    Serial.printf("Exhaust Temp: %3.0f °C \n", temp);
    
    SetN2kTemperature(N2kMsg, 0, 0, N2kts_ExhaustGasTemperature, CToKelvin(temp), N2kDoubleNA);
    NMEA2000.SendMsg(N2kMsg);
  }
}


// ReadVoltage is used to improve the linearity of the ESP32 ADC see: https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function

double ReadVoltage(byte pin){
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if(reading < 1 || reading > 4095) return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return (-0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089)*1000;
} // Added an improved polynomial, use either, comment out as required


  
void loop() {
unsigned int size;

  FuelLevel=((FuelLevel*15)+(ReadVoltage(ADCpin)*ADC_Calibration_Value/4096)) /16; // This implements a low pass filter to eliminate spike for ADC readings

  // if (FuelLevel>100) FuelLevel=100;
  
  SendN2kTankLevel(FuelLevel, 200);  // Adjust max tank capacity.  Is it 200 ???
  SendN2kExhaustTemp(ExhaustTemp);

  delay(50);
  
// Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
  if ( Serial.available() ) { Serial.read(); } 

}
