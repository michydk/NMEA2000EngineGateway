// Send DIY temp readings to NMEA 2000 bus.
// Based on the shoulders of my heros, Including but not limited to:
// DallasTemperature: Miles Burton https://github.com/milesburton
// NMEA2000: Timo Lappalainen https://github.com/ttlappalainen
// OneWire: Paul Stoffregen https://github.com/PaulStoffregen

#include <Arduino.h>
#define USE_N2K_CAN 1
#define N2k_SPI_CS_PIN 3
#define N2k_CAN_INT_PIN 7
#define USE_MCP_CAN_CLOCK_SET 16
#include <NMEA2000_CAN.h>  
#include <N2kMessages.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM={127489L,0};
#define ONE_WIRE_BUS 4 // Data wire is plugged into pin 4 on the Arduino
#define ALARMPIN 6 // Alarm pin to interface to buzzer, LED etc.
#define DEBUG
#define TempUpdatePeriod 2000
#define RPMUpdatePeriod 500
#define PERIOD_US 100L
#define OilAlarmTemp 85
#define CoolingAlarmTemp 85
#define CrankPulleyDiameter 127.86
#define AlternatorPulleyDiameter 58
#define PairsOfPoles 6
bool MotorAlarm = false;
const byte RPMPin=2;

//int intaketemp = 0;
int ratio = CrankPulleyDiameter / AlternatorPulleyDiameter;
int PulsePerRev = PairsOfPoles * ratio;
int coolingtemp = 0;int oiltemp = 0;
int pulses = 0, lastPulses = 0 ;
long eventTime = 0L, lastEventTime = 0L, prev_time =0L ; bool lastInput =0; 
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Assign the addresses of your 1-Wire temp sensors.
DeviceAddress cooling = {0x28, 0xFF, 0xCB, 0x30, 0x81, 0x15, 0x03, 0x6A };
DeviceAddress oil = {0x28, 0xFF, 0x9D, 0x67, 0x81, 0x15, 0x03, 0x3D };
//DeviceAddress intake = {0x28, 0xFF, 0xBB, 0x31, 0x81, 0x15, 0x03, 0x88 }; // Not really used, as there is no PNG for intake temp

void setup() {
  // Set Product information    
  NMEA2000.SetProductInformation("01", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "SillyDIY Engine Temp monitor",  // Manufacturer's Model ID
                                 "1.0.0.3 (2022-01-24)",  // Manufacturer's Software version code
                                 "1.0.0.3 (2022-01-24)" // Manufacturer's Model version                              
                                 );
  // Set device information
  NMEA2000.SetDeviceInformation(112233, // Unique number. Use e.g. Serial number.
                                160, // Device function=Temperature. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                50, // Device class=Sensor Communication Interface. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                174 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );                             
  #ifdef DEBUG
    Serial.begin(115200);
    //NMEA2000.SetForwardStream(&Serial);
    //NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); 
  #endif
  
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly,22); // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.EnableForward(false); 
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.Open();
  pinMode(RPMPin, INPUT);
  prev_time += PERIOD_US ;
}

void loop() {
   #ifdef DEBUG
       //Serial.print("----------------- ");Serial.print(millis()); Serial.println(" -----------------");
       //discoverOneWireDevices(); // Enable this to see your Temp sensor addresses 
   #endif
 
  //sensors.requestTemperatures();
  SendN2kData();
  NMEA2000.ParseMessages();
}

void SendN2kData() {
  static unsigned long TempUpdated=millis();
  static unsigned long RPMUpdated=millis();
  tN2kMsg N2kMsg;
// TEMP
  if ( TempUpdated+TempUpdatePeriod<millis() ) {
    TempUpdated=millis();
    ReadSensors();
    #ifdef DEBUG
      Serial.print("Cooling temp ");Serial.println(coolingtemp); 
      Serial.print("Oil temp ");Serial.println(oiltemp);
    #endif

    NMEA2000.SendMsg(N2kMsg);

    // Sending PGN 127489  
    if (coolingtemp > OilAlarmTemp || coolingtemp > CoolingAlarmTemp ) {
      SetN2kEngineDynamicParam(N2kMsg, 0, N2kDoubleNA, ReadoilTemp(), ReadEngineTemp(), N2kDoubleNA, N2kDoubleNA, N2kDoubleNA, N2kInt8NA,N2kInt8NA, N2kInt8NA, N2kInt8NA, false, true);
    } 
    else {
      SetN2kEngineDynamicParam(N2kMsg, 0, N2kDoubleNA, ReadoilTemp(), ReadEngineTemp(), N2kDoubleNA, N2kDoubleNA, N2kDoubleNA, N2kInt8NA,N2kInt8NA, N2kInt8NA, N2kInt8NA, false, false);
    }
    NMEA2000.SendMsg(N2kMsg);
  }
// RPM
  if ( RPMUpdated+RPMUpdatePeriod<millis() ) {
    RPMUpdated=millis();
    bool thisInput = digitalRead (RPMPin) ;
    if (lastInput != thisInput)   // account for every transition
      {
        account_pulses () ;
        lastInput = thisInput ;
      }
    if (micros() - prev_time >= PERIOD_US)  // regularly report
      {
      int puls1= ((pulses - lastPulses) * 6e7 / (eventTime - lastEventTime)) ;
      double rpm = puls1 / PulsePerRev; // totally wrong, but hay, am just testing..
      
      // PulsePerRev
      SetN2kPGN127488(N2kMsg, 0,  rpm, N2kDoubleNA, N2kInt8NA);
      #ifdef DEBUG
        Serial.print("Pulses ");
        Serial.println(pulses);
        Serial.print("Rpm ");
        Serial.println(rpm);
      #endif
      prev_time += PERIOD_US ;
      NMEA2000.SendMsg(N2kMsg);

/*
 * The crank pulley diameter is 127.86mm
The alternator pulley diameter is 58mm
Ration of 2.2044 to 1
12 pole alternator, 6 pairs of poles
Pulses per rev= number of pairs of poles x the pulley ratio.
6 x 2.2044 = 13.23 pulse per rev, for reference the standard number of pulses per rev for the original alternator is 10.29
This convert to frequencies of:
220 Hz at 1,000 rpm; 440 HZ at 2,000rpm and 660Hz at 3000 rpm
*/    
      }
  }
}
void account_pulses ()
{
  pulses ++ ;
  eventTime = micros() ;
}

void ReadSensors() {
 sensors.requestTemperatures();
 oiltemp = (sensorValue(oil)); 
 coolingtemp = (sensorValue(cooling));
 // intaketemp = (sensorValue(intake));
 // The value -127.00 is basicly unconnected.
 if (oiltemp == -127.00) { oiltemp = 0;} if (coolingtemp == -127.00) { coolingtemp = 0;} 
 //if (intaketemp == -127.00) { intaketemp = 0;}
}

float sensorValue (byte deviceAddress[])
{
  float tempC = sensors.getTempC (deviceAddress);
      #ifdef DEBUG
        //Serial.print("Sensorvalue: : "); 
        //Serial.println(tempC); 
    #endif
  return tempC;
}

double ReadoilTemp() {
  return CToKelvin(oiltemp); // Read here the true temperature e.g. from analog input
}

double ReadEngineTemp() {
  return CToKelvin(coolingtemp); // Read here the true temperature e.g. from analog input
}


void discoverOneWireDevices(void) {
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  
  Serial.print("Looking for 1-Wire devices...\n\r");
  while(oneWire.search(addr)) {
    Serial.print("\n\rFound \'1-Wire\' device with address:\n\r");
    for( i = 0; i < 8; i++) {
      Serial.print("0x");
      if (addr[i] < 16) {
        Serial.print('0');
      }
      Serial.print(addr[i], HEX);
      if (i < 7) {
        Serial.print(", ");
      }
    }
    if ( OneWire::crc8( addr, 7) != addr[7]) {
        Serial.print("CRC is not valid!\n");
        return;
    }
  }
  Serial.print("\n\r\n\rThat's it.\r\n");
  oneWire.reset_search();
  return;
}
