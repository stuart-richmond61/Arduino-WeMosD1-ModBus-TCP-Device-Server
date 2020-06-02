/*
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

  MbusDevServer for WeMosD1
  Created by Stuart Richmond   01-15-2020

  Uses WiFi Modbus TCP Server for multiple devices, using Adafruit libraries in most cases.  Device inclusion is set
  using defines below.

#define USE_DS18B20   - build with support for 1 or more DS18B20s on OneWire bus
#define USE_BMP280_1  - build with support for BMP280 at alternate address I2C
#define USE_BMP280_2  - build with support for BMP280 at primary address I2C
#define USE_HTU21_1   - build with support for HTU21 at primary address I2C
#define USE_HTU21_2   - build with support for HTU21 at alternate address I2C
#define USE_TSL2561   - build with support for TSL2561 on I2C

  Uilizes following libraries (some dependant on above defines) :

#include <SPI.h>
#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "Adafruit_HTU21DF.h"
#include <Modified_TSL2561.h>
#include <DS18B20.h>

Pin Usage:
D4     - OneWire bus for DS18B20s if included
D0     - input from motion sensor if connected

  Change log:
    02-01-2020  Modified to update DS18 device one per loop iteration
    05-27-2020  Working on RCWL-0516 radar motion detector
    06-01-2020  Add OTA (over the air) update build option
    01-01-2020  Add Scan timer modbus register (MS per scan)
  
*/
#define SRVR_DATE "06-01-2020  Author: Stu Richmond"
#define SRVR_MAJOR 1
#define SRVR_MINOR 9
//
// Unomment USE_OTA to have build include Over The Air programming code
//
#define USE_OTA
//
// Pin to associate motion detector input (on motion detected, off normal)
//
#define MOTION_PIN D0

//
//Caution: Keep UPDATERATEDS18 > UPDATERATEMS
//
#define UPDATERATEMS 1000
#define UPDATERATEDS18 10000
//
// Timers for managing poll and update timing in main loop
//
  uint32_t MainLoopLastRun;
  uint32_t DS18UpdateLastRun;
  uint32_t delta;
//
//  
//
//=======================================
//
//  ModBus Register assignments
//  floating point values
//
#define BMP1_FP_TEMP_BASE 0
//    40001  R(0) R(1)  BMP/E #1 Temperature
#define BMP1_FP_PRESS_BASE 2
//    40003  R(2) R(3)  BMP/E #1 Pressure
#define BME1_FP_HUMID_BASE 4
//    40005  R(4) R(5)  BME #1 Humidity (if build using BME280)
#define BMP2_FP_TEMP_BASE 6
//    40007  R(6) R(7)  BMP/E #2 Temperature
#define BMP2_FP_PRESS_BASE 8
//    40009  R(8) R(9)  BMP/E #2 Pressure
#define BME2_FP_HUMID_BASE 10
//    40011  R(10)R(11) BME #2 Humidity (if build using BME280)
#define HTU1_FP_HUMID_BASE 20
//    40021  R(20) R(21) HTU21 #1 Humidity
#define HTU1_FP_TEMP_BASE 22
//    40023  R(22) R(23) HTU21 #1 Temperature
#define HTU2_FP_HUMID_BASE 24
//    40025  R(24) R(25) HTU21 #2 Humidity
#define HTU2_FP_TEMP_BASE 26
//    40027  R(26) R(27) HTU21 #2 Temperature
//
#define DS18_FP_BASE 30
//    40031  R(30) R(31) DS18B20 #0
//    40033  R(32) R(33) DS18B20 #1
//    40035  R(34) R(35) DS18B20 #2
//    40037  R(36) R(37) DS18B20 #3
//    40039  R(38) R(39) DS18B20 #4
//    40041  R(40) R(41) DS18B20 #5
//    40043  R(42) R(43) DS18B20 #6
//    40045  R(44) R(45) DS18B20 #7
#define MAXDS 8

#define TSL_FP_LUMEN_BASE 50
//    40051  R(50) R(51)  TSL Luminosity reading
#define TSL_FP_BB_BASE 52
//    40053  R(52) R(53)  TSL broadband reading
#define TSL_FP_IR_BASE 54
//    40055  R(54) R(55)  TSL ir reading

#define SRVR_MAJOR_BASE 60
//    40061  R(60) Major version #
#define SRVR_MINOR_BASE 61
//    40062  R(61) Minor version #
#define WIFI_RSSI_BASE 62
//    40063  R(62) RSSI
#define DEV_BMPCNT_BASE 63
//    40064  R(63) BMP Count
#define DEV_HTUCNT_BASE 64
//    40065  R(64) HTU21 Count
#define DEV_DS18CNT_BASE 65
//    40066  R(65) DS18B20 Count
#define DEV_TSLCNT_BASE 66
//    40067  R(66) TSL 2561 Count
#define SRVR_CLIENT_CNT_BASE 67
//    40068 R(67) Client connection count
#define MOTION_PIN_BASE 68
//    40069 R(68) Motion detector Pin
#define MOTION_CNT_BASE 69
//    40070 R(69) Motion detector counter
#define SCAN_MS_BASE 70
//    40071 R(70) Motion detector counter
//
// Integer 16 values
#define BMP1_I16_TEMP_BASE 100
//    40101  R(100)  BMP/E #1 Temperature
#define BMP1_I16_PRESS_BASE 101
//    40102  R(101)  BMP/E #1 Pressure
#define BME1_I16_HUMID_BASE 102
//    40103  R(102)  BME #1 Humidity (if build using BME280)
#define BMP2_I16_TEMP_BASE 103
//    40104  R(103)  BMP/E #2 Temperature
#define BMP2_I16_PRESS_BASE 104
//    40105  R(104)  BMP/E #2 Pressure
#define BME2_I16_HUMID_BASE 105
//    40106  R(105)  BME #2 Humidity (if build using BME280)
#define HTU1_I16_HUMID_BASE 110
//    40111  R(110) HTU21 #1 Humidity
#define HTU1_I16_TEMP_BASE 111
//    40112  R(111) HTU21 #1 Temperature
#define HTU2_I16_HUMID_BASE 112
//    40113  R(112) HTU21 #2 Humidity
#define HTU2_I16_TEMP_BASE 113
//    40114  R(113) HTU21 #2 Temperature
#define TSL_I16_LUMEN_BASE 40
//    40115  R(114) TSL Luminosity reading
#define TSL_I16_BB_BASE 42
//    40116  R(115) TSL broadband reading
#define TSL_I16_IR_BASE 44
//    40117  R(116) TSL ir reading


#define DS18_I16_BASE 120
//    40121  R(120) DS18B20 #0
//    40122  R(71) DS18B20 #1
//    40123  R(72) DS18B20 #2
//    40124  R(73) DS18B20 #3
//    40125  R(74) DS18B20 #4
//    40126  R(75) DS18B20 #5
//    40127  R(76) DS18B20 #6
//    40128  R(77) DS18B20 #7
/*

*/
#define NUMREGISTERS 128+2
//
//=======================================
//
// Select which devices to include code for
// ONE_WIRE_BUS sets pin to use for onewire 
// devices
//
#define USE_DS18B20
#define ONE_WIRE_BUS D4

#define USE_BMP280_1
//#define USE_BMP280_2

#define USE_HTU21_1
//#define USE_HTU21_2

#define USE_TSL2561
//
//=======================================
// DEBUGI2C definition will include an I2C
// address scan in init with output to Serial
//
#define DEBUGI2C

#include "myprintf.h"

#include <SPI.h>
#include <ESP8266WiFi.h>
#ifdef USE_OTA
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#endif

//==== Onewire DS18B20 
#ifdef USE_DS18B20
#include <OneWire.h>
#endif

//==== I2C
#if defined(USE_BMP280_1) || defined(USE_BMP280_2) || defined(USE_HTU21) || defined(DEBUGI2C) || defined(USE_TSL2561)
#include <Wire.h>
#endif

//==== BMP280
#if defined(USE_BMP280_1) || defined(USE_BMP280_2)
#include <Adafruit_BMP280.h>
#endif

//==== HTU21
#if defined(USE_HTU21_1) || defined (USE_HTU21_2)
#include "Adafruit_HTU21DF.h"
#endif
#ifdef USE_HTU21_1
Adafruit_HTU21DF htu1 = Adafruit_HTU21DF();
#endif
#ifdef USE_HTU21_2
Adafruit_HTU21DF htu2 = Adafruit_HTU21DF();
#endif
//
//==== TSL2561
#ifdef USE_TSL2561
#include <Modified_TSL2561.h>
Adafruit_TSL2561 tsl = Adafruit_TSL2561(TSL2561_ADDR_FLOAT);
#endif

//==== Onewire DS18B20 
#ifdef USE_DS18B20
#include <DS18B20.h>
DS18B20 ds(ONE_WIRE_BUS);
#define DSADDRSIZE 8
uint8_t DS18AddressList[MAXDS][DSADDRSIZE];
int currentDS18;
#endif

#ifdef USE_BMP280_1
Adafruit_BMP280 bmp1;
#endif
#ifdef USE_BMP280_2
Adafruit_BMP280 bmp2; // use I2C interface
#endif
//
// Device status variables and counts
// Counts are made available via modbus
// registers
  bool BMP1State;
  bool BMP2State;
  int BMPCount;

  bool HTU1State;
  bool HTU2State;
  int HTUCount;

  bool TSLState;
  int TSLCount;

  int DS18B20Count;

  int ClientCount;
  
#include "ModbusTCPClient.h"
#include "ModbusTCPServer.h"
//
// Edit this file with your WIFI settings
//
#include "arduino_secrets.h"
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

WiFiServer wifiServer(502);

ModbusTCPServer modbusTCPServer;
//
//=========== Serial to RS485 ============================
// removed, unable to get to work on D1 mini
//#include <SoftwareSerial.h>
//SoftwareSerial  SSerial(D6,D7);  // RX, TX

//===============================================================
#ifdef DEBUG
void printascii( const char * str) {
  int i;
  for (i=0;i<strlen(str);i++) {
    if (isprint(str[i])!=0) {
      Serial.print(str[i]);
    } else {
      printf(Serial,"<%2.2X>",(unsigned char)str[i]);
    }
  }
}
#endif

//===============================================================
#ifdef DEBUGI2C
int WireScan()
{
  printf(Serial,"\n\rScanning for I2C devices...\n\r");
  byte count = 0;
  int FirstAddress = 0;
  Wire.begin();
  for (byte i = 1; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
      {
      printf(Serial," I2C device: %d (%0X)\n\r",i,i);
      if (FirstAddress == 0) FirstAddress = i;
      count++;
      delay (1);  // pause for settling
      } // found device
  } // for loop
  printf(Serial,"Found %d I2C device(s)\n\r",count);
  
  return FirstAddress;
}
#endif


//===============================================================
float LastMotion;
float MotionCount;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(74880);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
printf(Serial,"-- Init beginning --\n\r");
printf(Serial,"MBusDevServer v%d.%d  %s\n\r",SRVR_MAJOR,SRVR_MINOR,SRVR_DATE);
printf(Serial,"  Build Options:");
#ifdef USE_OTA
printf(Serial,"OTA ");
#endif
#ifdef USE_BMP280_1
printf(Serial,"BMP280_1 ");
#endif
#ifdef USE_BMP280_2
printf(Serial,"BMP280_2 ");
#endif
#ifdef USE_HTU21_1
printf(Serial,"HTU21_1 ");
#endif
#ifdef USE_HTU21_2
printf(Serial,"HTU21_2 ");
#endif
#ifdef USE_TSL2561
printf(Serial,"TSL2561 ");
#endif
#ifdef USE_DS18B20
printf(Serial,"DS18B20 ");
#endif
#ifdef DEBUGI2C
printf(Serial,"DEBUGI2C ");
#endif

printf(Serial," MOTION_PIN: %d ",MOTION_PIN);
printf(Serial,"\n\r");

pinMode(MOTION_PIN, INPUT);
LastMotion = 0;
MotionCount = 0;

#ifdef DEBUGI2C  
  WireScan();
#endif
#ifdef USE_BMP280_1  
  BMP1State = bmp1.begin(BMP280_ADDRESS_ALT);
#else
  BMP1State = false;
#endif
#ifdef USE_BMP280_2
  BMP2State = bmp2.begin(BMP280_ADDRESS );
#else
  BMP2State=false;
#endif
#ifdef USE_HTU21_1
  HTU1State = htu1.begin();
#else
  HTU1State=false;  
#endif   
#ifdef USE_HTU21_2
  HTU2State = htu2.begin();
#else
  HTU2State=false;  
#endif
#ifdef USE_TSL2561
  TSLState = tsl.begin();
#else
  TSLState = false;  
#endif
  BMPCount = (unsigned byte)BMP1State+(unsigned byte)BMP2State;
  HTUCount = (unsigned byte)HTU1State+(unsigned byte)HTU2State;
  TSLCount = (unsigned byte)TSLState;
#ifdef USE_DS18B20
// load address list of devices
  DS18B20Count = ds.getNumberOfDevices();
  ds.resetSearch();
  for (int i=0;i<DS18B20Count;i++) {
    ds.selectNext();
    ds.getAddress(DS18AddressList[i]);  
   }
#else
  DS18B20Count = 0;
#endif    

  printf(Serial,"\n\rI2C Devices:\n\r");
  printf(Serial,"        %d BMP280s\n\r",BMPCount);
  printf(Serial,"        %d HTU21s\n\r",HTUCount);
  printf(Serial,"        %d TSL2561s\n\r",TSLCount);
  printf(Serial,"\n\rOneWire Devices:\n\r");
  printf(Serial,"        %d DS18B20s\n\r",DS18B20Count);
#ifdef USE_DS18B20
  for (int i=0;i<DS18B20Count;i++) {
  printf(Serial,"            DS[%d]: %2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X\n\r",i,
     DS18AddressList[i][7],
     DS18AddressList[i][6],
     DS18AddressList[i][5],
     DS18AddressList[i][4],
     DS18AddressList[i][3],
     DS18AddressList[i][2],
     DS18AddressList[i][1],
     DS18AddressList[i][0]
     );
  }
  currentDS18 = 0;
#endif
//
// Run WIFI interface in station mode, connect to AP
//
  WiFi.mode(WIFI_STA);
  Serial.print("\n\rMAC: ");
  Serial.println(WiFi.macAddress());
  
// attempt to connect to Wifi AP, pause 10 seconds between attempts
//
  while (WiFi.status() != WL_CONNECTED) {
    printf(Serial,"Connecting to SSID: %s\n\r",ssid);
    // Logon to WPA/WPA2 network using credentials supplied in secrets file.
    status = WiFi.begin(ssid, pass);
    uint8_t i = 0;
    while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
    //    wait 10 seconds to retry connection if not connected
    if (WiFi.status() != WL_CONNECTED) delay(10000);
  }

  // Print connection status:
    printWifiStatus();

  // start OTA if included
#ifdef USE_OTA
 // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    printf(Serial,"Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    printf(Serial, "Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
#endif

  // Start server listenning
  wifiServer.begin();
  wifiServer.setNoDelay(true);  // send immediate, don't optimize
  
  // start the Modbus TCP server
  if (!modbusTCPServer.begin(1)) {
    Serial.println("Failed to start Modbus TCP Server!");
    while (1);
  }

  // configure input registers (two registers per value for floating point)
  // see defines at top of this file for register assignment usage
  modbusTCPServer.configureInputRegisters(0x00, NUMREGISTERS);

  // track number of client connections, and make available on
  // modbus register
  ClientCount = 0;
//
// initialize flag bad value for all registers (fix me - initialize with first device read if present
//
        float curvalue=-9999.0;
        UpdateModbusFloatRegister(HTU1_FP_TEMP_BASE,curvalue);
        UpdateModbusInt16Register(HTU1_I16_TEMP_BASE,curvalue);
        UpdateModbusFloatRegister(HTU1_FP_HUMID_BASE,curvalue);
        UpdateModbusInt16Register(HTU1_I16_HUMID_BASE,curvalue);           
        UpdateModbusFloatRegister(HTU2_FP_TEMP_BASE,curvalue);
        UpdateModbusInt16Register(HTU2_I16_TEMP_BASE,curvalue);
        UpdateModbusFloatRegister(HTU2_FP_HUMID_BASE,curvalue);
        UpdateModbusInt16Register(HTU2_I16_HUMID_BASE,curvalue);
        UpdateModbusFloatRegister(BMP1_FP_TEMP_BASE,curvalue);
        UpdateModbusInt16Register(BMP1_I16_TEMP_BASE,curvalue);
        UpdateModbusFloatRegister(BMP1_FP_PRESS_BASE,curvalue);
        UpdateModbusInt16Register(BMP1_I16_PRESS_BASE,curvalue);
        UpdateModbusFloatRegister(BMP2_FP_TEMP_BASE,curvalue);
        UpdateModbusInt16Register(BMP2_I16_TEMP_BASE,curvalue);
        UpdateModbusFloatRegister(BMP2_FP_PRESS_BASE,curvalue);
        UpdateModbusInt16Register(BMP2_I16_PRESS_BASE,curvalue);
        UpdateModbusFloatRegister(TSL_FP_LUMEN_BASE,curvalue);
        UpdateModbusInt16Register(TSL_I16_LUMEN_BASE,curvalue);
        UpdateModbusFloatRegister(TSL_FP_BB_BASE,curvalue);
        UpdateModbusInt16Register(TSL_I16_BB_BASE,curvalue);
        UpdateModbusFloatRegister(TSL_FP_IR_BASE,curvalue);
        UpdateModbusInt16Register(TSL_I16_IR_BASE,curvalue);
        for (int i=0;i<MAXDS;i++) { 
              UpdateModbusFloatRegister(DS18_FP_BASE+(i*2),curvalue);
              UpdateModbusInt16Register(DS18_I16_BASE+i,curvalue);
        }

// removed unable to get to work on D1 mini
//SSerial.begin(9600);
       
printf(Serial,"-- Init complete --\n\r");
}
//
//===========================================================================
//
// set up macro to place float in proper byte order for register storage
// only uncomment 1 of the macros
//
//#define USE_SET_FLOAT_ORDER modbus_set_float_abcd
#define USE_SET_FLOAT_ORDER modbus_set_float_dcba
//#define USE_SET_FLOAT_ORDER modbus_set_float_badc
//#define USE_SET_FLOAT_ORDER modbus_set_float_cdab
//#define USE_SET_FLOAT_ORDER modbus_set_float
//===========================================================================
//
// Store floating value into selected register pair
//
void UpdateModbusFloatRegister(uint16_t BaseRegister, float val)
{
    uint16_t intval[2];
    char fstr[11];

        USE_SET_FLOAT_ORDER(val,&(intval[0]));
        modbusTCPServer.inputRegisterWrite(BaseRegister, intval[0]);      
        modbusTCPServer.inputRegisterWrite(BaseRegister+1, intval[1]);      
//        dtostrf(val,8,2,fstr);
//        printf(Serial,"UpdateModbusFloatRegister: addr: %d (R%d) val - %s  %2.2x%2.2x%2.2x%2.2x\n",BaseRegister+1,BaseRegister,fstr,
//        (intval[0]>>8)& 0xFF,(intval[0])& 0xFF,
//        (intval[1]>>8)& 0xFF,(intval[1])& 0xFF);   
}
//
//============================================================================
//
// Store integer value into selected register
//
void UpdateModbusInt16Register(uint8_t BaseRegister, float val)
{
    union {
    uint16_t uival;
    int16_t ival;
    } x;
    char fstr[11];
       if (val == -9999.0) {
         x.ival = -9999;
       } else {
        if (val>100.0) {
          x.uival = int(val * 10.0);
        } else {
          x.uival = int(val * 100.0);
        }
       }
        modbusTCPServer.inputRegisterWrite(BaseRegister, x.uival);
//        dtostrf(val,8,2,fstr);
//        printf(Serial,"UpdateModbusInt16Register: addr: %d (R%d) val - %s  int - %d  uint - %d\n\r",BaseRegister+1,BaseRegister,fstr,x.ival, x.uival);
//  printf(Serial,".");
}  

//
//===========================================================================
//
// Called from main loop at UPDATERATEMS millisecond period
//
// Reads each device configured/available, and stores result into associated
// modbus register (floating and integer)
//

void UpdateModbusRegistersEveryScan()
{
  float curvalue;
  char fstr[11];  
  float CurMotion;
  
  int rssi = WiFi.RSSI();
//  printf(Serial," rssi:%ld   base:%d\n\r",rssi,WIFI_RSSI_BASE);
  modbusTCPServer.inputRegisterWrite(WIFI_RSSI_BASE, rssi);      
//
  CurMotion = digitalRead(MOTION_PIN);
  modbusTCPServer.inputRegisterWrite(MOTION_PIN_BASE,CurMotion);      
  if (CurMotion != LastMotion) { // change detected
    if (CurMotion==1) {
      MotionCount++;
      modbusTCPServer.inputRegisterWrite(MOTION_CNT_BASE,MotionCount);      
    }
    LastMotion = CurMotion;
  }
// -----  Update register entries for HTU21 #1                
#ifdef USE_HTU21_1    
  if (HTU1State==true) {
        curvalue=htu1.readTemperature();
//        dtostrf(curvalue,8,2,fstr);
//        printf(Serial,"HTU1 temp: %s\n\r",fstr);
        UpdateModbusFloatRegister(HTU1_FP_TEMP_BASE,curvalue);
        UpdateModbusInt16Register(HTU1_I16_TEMP_BASE,curvalue);

        curvalue=htu1.readHumidity();
//        dtostrf(curvalue,8,2,fstr);
//        printf(Serial,"HTU1 humid: %s\n\r",fstr);
        UpdateModbusFloatRegister(HTU1_FP_HUMID_BASE,curvalue);
        UpdateModbusInt16Register(HTU1_I16_HUMID_BASE,curvalue);
  } else {
        curvalue=-9999.0;
        UpdateModbusFloatRegister(HTU1_FP_TEMP_BASE,curvalue);
        UpdateModbusInt16Register(HTU1_I16_TEMP_BASE,curvalue);
        UpdateModbusFloatRegister(HTU1_FP_HUMID_BASE,curvalue);
        UpdateModbusInt16Register(HTU1_I16_HUMID_BASE,curvalue);
  }
#endif      

#ifdef USE_HTU21_2    
  if (HTU2State==true) {
        curvalue=htu2.readTemperature();
//        dtostrf(curvalue,8,2,fstr);
//        printf(Serial,"HTU2 temp: %s\n\r",fstr);
        UpdateModbusFloatRegister(HTU2_FP_TEMP_BASE,curvalue);
        UpdateModbusInt16Register(HTU2_I16_TEMP_BASE,curvalue);

        curvalue=htu2.readHumidity();
//        dtostrf(curvalue,8,2,fstr);
//        printf(Serial,"HTU2 humid: %s\n\r",fstr);
        UpdateModbusFloatRegister(HTU2_FP_HUMID_BASE,curvalue);
        UpdateModbusInt16Register(HTU2_I16_HUMID_BASE,curvalue);
  } else {
        curvalue=-9999.0;
        UpdateModbusFloatRegister(HTU2_FP_TEMP_BASE,curvalue);
        UpdateModbusInt16Register(HTU2_I16_TEMP_BASE,curvalue);
        UpdateModbusFloatRegister(HTU2_FP_HUMID_BASE,curvalue);
        UpdateModbusInt16Register(HTU2_I16_HUMID_BASE,curvalue);
   }
#endif      
#ifdef USE_BMP280_1      
   if (BMP1State==true){
        curvalue=bmp1.readTemperature();
//        dtostrf(curvalue,8,2,fstr);
//        printf(Serial,"BMP1 temp: %s\n\r",fstr);
        UpdateModbusFloatRegister(BMP1_FP_TEMP_BASE,curvalue);
        UpdateModbusInt16Register(BMP1_I16_TEMP_BASE,curvalue);

        curvalue=bmp1.readPressure() / 100.0F;
//        dtostrf(curvalue,8,2,fstr);
//        printf(Serial,"BMP1 press: %s\n\r",fstr);
        UpdateModbusFloatRegister(BMP1_FP_PRESS_BASE,curvalue);
        UpdateModbusInt16Register(BMP1_I16_PRESS_BASE,curvalue);
   } else {
        curvalue=-9999.0;
        UpdateModbusFloatRegister(BMP1_FP_TEMP_BASE,curvalue);
        UpdateModbusInt16Register(BMP1_I16_TEMP_BASE,curvalue);
        UpdateModbusFloatRegister(BMP1_FP_PRESS_BASE,curvalue);
        UpdateModbusInt16Register(BMP1_I16_PRESS_BASE,curvalue);
   }
#endif
#ifdef USE_BMP280_2
   if (BMP2State==true){
        curvalue=bmp2.readTemperature();
//        dtostrf(curvalue,8,2,fstr);
//        printf(Serial,"BMP2 temp: %s\n\r",fstr);
        UpdateModbusFloatRegister(BMP2_FP_TEMP_BASE,curvalue);
        UpdateModbusInt16Register(BMP2_I16_TEMP_BASE,curvalue);

        curvalue=bmp2.readPressure() / 100.0F;
//        dtostrf(curvalue,8,2,fstr);
//        printf(Serial,"BMP2 press: %s\n\r",fstr);
        UpdateModbusFloatRegister(BMP2_FP_PRESS_BASE,curvalue);
        UpdateModbusInt16Register(BMP2_I16_PRESS_BASE,curvalue);
   } else {
        curvalue=-9999.0;
        UpdateModbusFloatRegister(BMP2_FP_TEMP_BASE,curvalue);
        UpdateModbusInt16Register(BMP2_I16_TEMP_BASE,curvalue);

        UpdateModbusFloatRegister(BMP2_FP_PRESS_BASE,curvalue);
        UpdateModbusInt16Register(BMP2_I16_PRESS_BASE,curvalue);
   }
#endif      
#ifdef USE_TSL2561
    if (TSLState==true) {
         if (tsl.ReadLightNoBlock()) { // try until it completes
//           char fstr[11];
//           printf(Serial,"TSL read\n\r");
           curvalue = tsl.LightReading();
//           dtostrf(curvalue,8,2,fstr);
//           printf(Serial,"Lumen:%s ",fstr);
           UpdateModbusFloatRegister(TSL_FP_LUMEN_BASE,curvalue);
           UpdateModbusInt16Register(TSL_I16_LUMEN_BASE,curvalue);
           curvalue = tsl.BroadbandReading();
//           printf(Serial," BB:%ul ",tsl.BroadbandReading());
           UpdateModbusFloatRegister(TSL_FP_BB_BASE,curvalue);
           UpdateModbusInt16Register(TSL_I16_BB_BASE,curvalue);
           curvalue = tsl.irReading();
//           printf(Serial," IR:%ul ",tsl.irReading());
           UpdateModbusFloatRegister(TSL_FP_IR_BASE,curvalue);
           UpdateModbusInt16Register(TSL_I16_IR_BASE,curvalue);
//           printf(Serial,"\n\r");
         }
    } else {
        curvalue=-9999.0;
        UpdateModbusFloatRegister(TSL_FP_LUMEN_BASE,curvalue);
        UpdateModbusInt16Register(TSL_I16_LUMEN_BASE,curvalue);
        UpdateModbusFloatRegister(TSL_FP_BB_BASE,curvalue);
        UpdateModbusInt16Register(TSL_I16_BB_BASE,curvalue);
        UpdateModbusFloatRegister(TSL_FP_IR_BASE,curvalue);
        UpdateModbusInt16Register(TSL_I16_IR_BASE,curvalue);
    }
#endif

}
//
//===========================================================================
//
// Called from main loop at UPDATERATEMS millisecond period
//
// Reads each device configured/available, and stores result into associated
// modbus register (floating and integer)
//

void UpdateModbusRegisters()
{
  float curvalue;
  char fstr[11];  
  
//printf(Serial,"UpdateModbusRegisters entered...\n\r");
#ifdef USE_DS18B20
    if ((ds.getNumberOfDevices()>0)) {
      delta = millis() - DS18UpdateLastRun;
      if (( delta > UPDATERATEDS18 ) ) {  // Run one read per scan
        DS18UpdateLastRun = millis();
//        uint32_t rdelta;
//        rdelta = millis();
//        printf(Serial,"DS18 Reading ");
           ds.select(DS18AddressList[currentDS18]);
           curvalue=ds.getTempC();
//        dtostrf(curvalue,8,2,fstr);
//        printf(Serial,"DS18(%d): %s    millis:%ld\n\r",currentDS18,fstr,millis()-rdelta);
           UpdateModbusFloatRegister(DS18_FP_BASE+(currentDS18*2),curvalue);
           UpdateModbusInt16Register(DS18_I16_BASE+currentDS18,curvalue);
           currentDS18++;
           if ((currentDS18>=ds.getNumberOfDevices())||(currentDS18>=MAXDS)) currentDS18 = 0;
      }
    } else {
           curvalue=-9999.0;
           for (int i=0;i<MAXDS;i++) { 
              UpdateModbusFloatRegister(DS18_FP_BASE+(i*2),curvalue);
              UpdateModbusInt16Register(DS18_I16_BASE+i,curvalue);
            }
    }
#endif
//printf(Serial,"UpdateModbusRegisters exited...\n\r");
  
}
//
//===========================================================================
//
int ByteCount=0;
void PrintByte(unsigned char c)
{
  printf(Serial,"%X ",c);
  ByteCount++;
  if (ByteCount>10) {
    printf(Serial,"\n\r");
    ByteCount = 0;
  }
}
//
//============================================================================
//
// main loop
// Wait for client connection and process modbus commands
// Note this code steps on registers each iteration,
// no device updates to holding registers are retained.
//
unsigned int LastScanMilli;
unsigned int DeltaScanMilli;
unsigned int CurScanMilli;
void loop() {
 //
 // update scan time
   CurScanMilli = millis();
   DeltaScanMilli = CurScanMilli-LastScanMilli;
   modbusTCPServer.inputRegisterWrite(SCAN_MS_BASE, DeltaScanMilli); 
   LastScanMilli = CurScanMilli; 
  
  // test OTA
#ifdef USE_OTA
  ArduinoOTA.handle();
#endif  
  // listen for client
  WiFiClient client = wifiServer.available();
  
  if (client) {
    modbusTCPServer.inputRegisterWrite(SRVR_CLIENT_CNT_BASE, ++ClientCount); 
    // new client
    printf(Serial,"Connect from Client - %s:%d\n\r",client.remoteIP().toString().c_str(), client.remotePort());

    // let the Modbus TCP accept the connection 
    modbusTCPServer.accept(client);
    MainLoopLastRun = 0;
    DS18UpdateLastRun = 0;
//
//  Set the register values for device status and counts
//
    modbusTCPServer.inputRegisterWrite(DEV_BMPCNT_BASE, BMPCount); 
    modbusTCPServer.inputRegisterWrite(DEV_HTUCNT_BASE, HTUCount); 
    modbusTCPServer.inputRegisterWrite(DEV_TSLCNT_BASE, TSLCount); 
    modbusTCPServer.inputRegisterWrite(DEV_DS18CNT_BASE, DS18B20Count); 

    modbusTCPServer.inputRegisterWrite(SRVR_MAJOR_BASE, SRVR_MAJOR); 
    modbusTCPServer.inputRegisterWrite(SRVR_MINOR_BASE, SRVR_MINOR); 
//
//  process function requests from client until disconnected
//
    while (client.connected()) {
 //
  // test OTA
#ifdef USE_OTA
      ArduinoOTA.handle();
#endif
//
//    Read millisecond counter
//  
      CurScanMilli = millis();
//
//    Calculate change since last scan
//
      DeltaScanMilli = CurScanMilli-LastScanMilli;
//
//    Update modbus register with Delta
//      
      modbusTCPServer.inputRegisterWrite(SCAN_MS_BASE, DeltaScanMilli); 
//
//    Store current millie reading for next iteration delta calc
//    
      LastScanMilli = CurScanMilli; 
//
//    Calculate MS since last processing
//      
      UpdateModbusRegistersEveryScan();

      delta = CurScanMilli-MainLoopLastRun;
      if (delta >= UPDATERATEMS) {
        MainLoopLastRun = CurScanMilli; 
        UpdateModbusRegisters();
      }
//      printf(Serial,"Calling modbusTCPServer.poll\n\r");
      // poll for Modbus TCP requests, while client connected
      modbusTCPServer.poll();
// removed, could not get to work on D1 mini
//while (SSerial.available()) PrintByte(SSerial.read());
    }
    printf(Serial,"Client disconnected...\n\r");
  }
}
//
//=============================================================================
//
void printWifiStatus() {
  // SSID
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // IP address received:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Addr: ");
  Serial.println(ip);

  // RSSI:
  long rssi = WiFi.RSSI();
  Serial.print("RSSI:");
  Serial.print(rssi);
  Serial.println(" dBm");
}
