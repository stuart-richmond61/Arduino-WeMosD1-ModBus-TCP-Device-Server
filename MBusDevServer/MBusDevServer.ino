/*
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

  MbusDevServer for WeMosD1
  Created by Stuart Richmond   01-15-2020

  Uses WiFi Modbus TCP Server for multiple devices, using Adafruit libraries in most cases.  Device inclusion is set
  using defines below.

  To update sketch using OTA (over the air):  (VPCWin7P has it)
    1) Open Arduino IDE
    2) Load sketch to upload
    3) Select Tools / Port
       Arduino IP should be detected and show in "Network Ports" list
    4) Select Sketch / Upload
    
#define USE_MQTT      - Build in MQTT client
#define USE_DS18B20   - build with support for 1 or more DS18B20s on OneWire bus
#define USE_BMP280_1  - build with support for BMP280 at alternate address I2C
#define USE_BMP280_2  - build with support for BMP280 at primary address I2C
#define USE_HTU21_1   - build with support for HTU21 at primary address I2C
#define USE_HTU21_2   - build with support for HTU21 at alternate address I2C
#define USE_TSL2561   - build with support for TSL2561 on I2C
#define USE_DO        - build with support for driving digital outputs

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
D0     - input from motion sensor if connected
D1     - SCL for I2C devices BMP, HTU, TSL
D2     - SDA for I2C devices
D4     - OneWire bus for DS18B20s if included
D5     - DO 000001
D6     - DO 000002

  Change log:
    02-01-2020  Modified to update DS18 device one per loop iteration
    05-27-2020  Working on RCWL-0516 radar motion detector
    06-01-2020  Add OTA (over the air) update build option
    06-01-2020  Add Scan timer modbus register (MS per scan)
    06-04-2020  Add Off delay for motion (for 1 minute historian)
    01-23-2021  Add support for DOs
    02-19-2021  Comments and version number update
    02-06-2022  MQTT client addition
    02-10-2022  Separate configuration items into header files
    
  
*/
#include "config.h"

#include "MBusConfig.h"

#include "MQTTConfig.h"

//
// Timers for managing poll and update timing in main loop
//
  uint32_t MainLoopLastRun;
  uint32_t DS18UpdateLastRun;
  uint32_t delta;

//
//---------------------------------------
//
#ifdef USE_MQTT
//const char MQTTtopicPrefix[]  = "DevServer/";
// set prefix to IP address of device
#define MAXPREFIX 20
char MQTTtopicPrefix[MAXPREFIX];

// careful, order tied to STAT defines
const char * MQTTtopicServerStats[] = {
"STAT/SRVR_MAJOR",
"STAT/SRVR_MINOR",
"STAT/WIFI_RSSI",
"STAT/BMPCNT",
"STAT/HTUCNT",
"STAT/DS18CNT",
"STAT/TSLCNT",
"STAT/CLIENT_CNT",
"STAT/MOTION_PIND",
"STAT/MOTION_PIN",
"STAT/MOTION_CNT",
"STAT/SCAN_MS",
0
};

#endif

#define STAT_SRVR_MAJOR 0
#define STAT_SRVR_MINOR 1
#define STAT_WIFI_RSSI 2
#define STAT_BMPCNT 3
#define STAT_HTUCNT 4
#define STAT_DS18CNT 5
#define STAT_TSLCNT 6
#define STAT_CLIENT_CNT 7
#define STAT_MOTION_PIND 8
#define STAT_MOTION_PIN 9
#define STAT_MOTION_CNT 10
#define STAT_SCAN_MS 11

#define NUM_SERVER_STATS 12

int ServerStats[NUM_SERVER_STATS];

//
//=======================================
// DEBUGI2C definition will include an I2C
// address scan in init with output to Serial
//

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



#define BMP1_TEMP 0
#define BMP1_PRESS 1
#define BMP1_HUMIDITY 2
#define BMP2_TEMP 3
#define BMP2_PRESS 4
#define BMP2_HUMIDITY 5
#define HTU1_TEMP 6
#define HTU1_HUMIDITY 7
#define HTU2_TEMP 8
#define HTU2_HUMIDITY 9
#define TSL_LUMIN 10
#define TSL_BB 11
#define TSL_IR 12
#define DS18_0 13
// reserve MAXDS
#define NUM_PVS DS18_0 + 8

#ifdef USE_MQTT
const char * MQTTtopicPV[] = {
"PV/BMP1_TEMP",
"PV/BMP1_PRESS",
"PV/BMP1_HUMIDITY",
"PV/BMP2_TEMP",
"PV/BMP2_PRESS",
"PV/BMP2_HUMIDITY",
"PV/HTU1_TEMP",
"PV/HTU1_HUMIDITY",
"PV/HTU2_TEMP",
"PV/HTU2_HUMIDITY",
"PV/TSL_LUMIN",
"PV/TSL_BB",
"PV/TSL_IR",
"PV/DS18_0",
"PV/DS18_1",
"PV/DS18_2",
"PV/DS18_3",
"PV/DS18_4",
"PV/DS18_5",
"PV/DS18_6",
"PV/DS18_7",
0
};
#endif

float PV_values[NUM_PVS]; //

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

#ifdef USE_MQTT
#include "ArduinoMqttClient.h"
#endif

#ifdef USE_MODBUS
#include "ModbusTCPClient.h"
#include "ModbusTCPServer.h"
#endif
//
// Edit this file with your WIFI settings
//
#include "arduino_secrets.h"
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

//==============  MODBUS configuration ==================
#ifdef USE_MODBUS
WiFiServer wifiServer(502);
ModbusTCPServer modbusTCPServer;
#endif

//============== MQTT configuration ====================
#ifdef USE_MQTT
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
const char MQTTbroker[] = "192.168.1.111";
int        MQTTport     = 1883;

//set interval for sending messages (milliseconds)
const long MQTTinterval = 8000;
unsigned long MQTTpreviousMillis = 0;

int MQTTState = 0;
#endif
//
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
//=======================Motion detection variables ========================================
float LastMotion;
float MotionCount;
#define OFFDELAYMS 60000
float LastOn;

//============================================ Setup =======================================

void setup() {
  int i;
  //Initialize serial and wait for port to open:
  Serial.begin(74880);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
printf(Serial,"-- Init beginning --\n\r");
printf(Serial,"MBusDevServer v%d.%d  %s\n\r",SRVR_MAJOR,SRVR_MINOR,SRVR_DATE);

printf(Serial,"\n\r  Build Options:");
#ifdef USE_MODBUS
printf(Serial,"MODBUS ");
#endif
#ifdef USE_MQTT
printf(Serial,"MQTT ");
#endif
#ifdef DEBUGI2C
printf(Serial,"DEBUGI2C ");
#endif
#ifdef USE_OTA
printf(Serial,"OTA ");
#endif

printf(Serial,"\n\r  Device Options: ");
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
#ifdef USE_DO
printf(Serial,"DO ");
#endif

printf(Serial,"\n\r  MOTION_PIN: %d ",MOTION_PIN);

pinMode(MOTION_PIN, INPUT);
LastMotion = 0;
MotionCount = 0;
//
// 01-23-2021  Add DO functionality
//
#ifdef USE_DO
#ifdef DO0_PIN
pinMode(DO0_PIN,OUTPUT);
digitalWrite(DO0_PIN,INITIAL_DO_STATE);
printf(Serial," DOs: %d ",DO0_PIN);
#endif
#ifdef DO1_PIN
pinMode(DO1_PIN,OUTPUT);
digitalWrite(DO1_PIN,INITIAL_DO_STATE);
printf(Serial,"%d ",DO1_PIN);
#endif
#ifdef DO2_PIN
pinMode(DO2_PIN,OUTPUT);
digitalWrite(DO2_PIN,INITIAL_DO_STATE);
printf(Serial,"%d ",DO2_PIN);
#endif
#ifdef DO3_PIN
pinMode(DO3_PIN,OUTPUT);
digitalWrite(DO3_PIN,INITIAL_DO_STATE);
printf(Serial,"%d ",DO3_PIN);
#endif
#endif
// 01-23-2021 end add DO functionality
printf(Serial,"\n\r");

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
  ServerStats[STAT_BMPCNT] = float(BMPCount);
  ServerStats[STAT_HTUCNT] = float(HTUCount);
  ServerStats[STAT_TSLCNT] = float(TSLCount);
  ServerStats[STAT_DS18CNT] = float(DS18B20Count);
  
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
  IPAddress ip = WiFi.localIP();
  sprintf(MQTTtopicPrefix,"%d.%d.%d.%d/",ip[0],ip[1],ip[2],ip[3]);
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

  // Set bad PV signal value for initialization
  
        float curvalue=-9999.0;

  // Start server listenning
 #ifdef USE_MODBUS
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

  // 01/23/2021 add DO support
  // configure coils for outputs at base address 0x00
  modbusTCPServer.configureCoils(0x00, NUMREGISTERS);

  // Initialize coils to ON
  for(i=0;i<=MAXDOS;i++) {
    modbusTCPServer.coilWrite(DO_BASE+i,INITIAL_DO_STATE);
  }

//  modbusTCPServer.configureDiscreteInputs(0x00, NUMREGISTERS);
//  modbusTCPServer.configureHoldingRegisters(0x00, NUMREGISTERS);
  
  // 01/23/2021 end add DO support
  
  // track number of client connections, and make available on
  // modbus register
  ClientCount = 0;
  ServerStats[STAT_CLIENT_CNT] = float(ClientCount);
//
// initialize flag bad value for all registers (fix me - initialize with first device read if present
//
        UpdateModbusFloatRegister(HTU1_FP_TEMP_BASE,curvalue);  PV_values[HTU1_TEMP]=curvalue;
        UpdateModbusInt16Register(HTU1_I16_TEMP_BASE,curvalue);
        UpdateModbusFloatRegister(HTU1_FP_HUMID_BASE,curvalue); PV_values[HTU1_HUMIDITY]=curvalue;
        UpdateModbusInt16Register(HTU1_I16_HUMID_BASE,curvalue);           
        UpdateModbusFloatRegister(HTU2_FP_TEMP_BASE,curvalue); PV_values[HTU2_TEMP]=curvalue;
        UpdateModbusInt16Register(HTU2_I16_TEMP_BASE,curvalue);
        UpdateModbusFloatRegister(HTU2_FP_HUMID_BASE,curvalue); PV_values[HTU2_HUMIDITY]=curvalue;
        UpdateModbusInt16Register(HTU2_I16_HUMID_BASE,curvalue);
        UpdateModbusFloatRegister(BMP1_FP_TEMP_BASE,curvalue); PV_values[BMP1_TEMP]=curvalue;
        UpdateModbusInt16Register(BMP1_I16_TEMP_BASE,curvalue);
        UpdateModbusFloatRegister(BMP1_FP_PRESS_BASE,curvalue); PV_values[BMP1_PRESS]=curvalue;
        UpdateModbusInt16Register(BMP1_I16_PRESS_BASE,curvalue);
        UpdateModbusFloatRegister(BMP2_FP_TEMP_BASE,curvalue); PV_values[BMP2_TEMP]=curvalue;
        UpdateModbusInt16Register(BMP2_I16_TEMP_BASE,curvalue);
        UpdateModbusFloatRegister(BMP2_FP_PRESS_BASE,curvalue); PV_values[BMP2_PRESS]=curvalue;
        UpdateModbusInt16Register(BMP2_I16_PRESS_BASE,curvalue);
        UpdateModbusFloatRegister(TSL_FP_LUMEN_BASE,curvalue); PV_values[TSL_LUMIN]=curvalue;
        UpdateModbusInt16Register(TSL_I16_LUMEN_BASE,curvalue);
        UpdateModbusFloatRegister(TSL_FP_BB_BASE,curvalue); PV_values[TSL_BB]=curvalue;
        UpdateModbusInt16Register(TSL_I16_BB_BASE,curvalue);
        UpdateModbusFloatRegister(TSL_FP_IR_BASE,curvalue);PV_values[TSL_IR]=curvalue;
        UpdateModbusInt16Register(TSL_I16_IR_BASE,curvalue);
        PV_values[BMP1_HUMIDITY]=curvalue;
        PV_values[BMP2_HUMIDITY]=curvalue;
        for (int i=0;i<MAXDS;i++) { 
              UpdateModbusFloatRegister(DS18_FP_BASE+(i*2),curvalue);
              PV_values[DS18_0+i] = curvalue;
              UpdateModbusInt16Register(DS18_I16_BASE+i,curvalue);
        }
    modbusTCPServer.inputRegisterWrite(DEV_BMPCNT_BASE, BMPCount); 
    modbusTCPServer.inputRegisterWrite(DEV_HTUCNT_BASE, HTUCount); 
    modbusTCPServer.inputRegisterWrite(DEV_TSLCNT_BASE, TSLCount); 
    modbusTCPServer.inputRegisterWrite(DEV_DS18CNT_BASE, DS18B20Count); 

    modbusTCPServer.inputRegisterWrite(SRVR_MAJOR_BASE, SRVR_MAJOR); 
    modbusTCPServer.inputRegisterWrite(SRVR_MINOR_BASE, SRVR_MINOR); 
#endif
//
//    Initialize PV storage array
//
        PV_values[HTU1_TEMP]=curvalue;
        PV_values[HTU1_HUMIDITY]=curvalue;
        PV_values[HTU2_TEMP]=curvalue;
        PV_values[HTU2_HUMIDITY]=curvalue;
        PV_values[BMP1_TEMP]=curvalue;
        PV_values[BMP1_PRESS]=curvalue;
        PV_values[BMP2_TEMP]=curvalue;
        PV_values[BMP2_PRESS]=curvalue;
        PV_values[TSL_LUMIN]=curvalue;
        PV_values[TSL_BB]=curvalue;
        PV_values[TSL_IR]=curvalue;
        PV_values[BMP1_HUMIDITY]=curvalue;
        PV_values[BMP2_HUMIDITY]=curvalue;
        for (int i=0;i<MAXDS;i++) { 
              PV_values[DS18_0+i] = curvalue;
        }

//
// Set up MQTT client
//
#ifdef USE_MQTT
    printf(Serial,"MQTT Broker - %s:%d\n\r",MQTTbroker,MQTTport);
    printf(Serial,"     Server Topics: \n\r");
    for (int i=0;i<NUM_SERVER_STATS;i++) {
         printf(Serial,"         %s%s\n\r",MQTTtopicPrefix,MQTTtopicServerStats[i]);
    }
    printf(Serial,"     PV Topics: \n\r");
    for (int i=0;i<NUM_PVS;i++) {
         printf(Serial,"         %s%s\n\r",MQTTtopicPrefix,MQTTtopicPV[i]);
    }
    if (!mqttClient.connect(MQTTbroker,MQTTport)) {
      MQTTState = 0;
      printf(Serial,"MQTT connection failed %s:%d - Error: ",MQTTbroker, MQTTport);
      Serial.println(mqttClient.connectError());
//    Errors : -2 = connection refused
    } else {
      MQTTState = 1;
      printf(Serial,"MQTT connected at %s:%d\n\r",MQTTbroker, MQTTport);
    }  
#endif

   ServerStats[STAT_SRVR_MAJOR] = SRVR_MAJOR;
   ServerStats[STAT_SRVR_MINOR] = SRVR_MINOR;
  

printf(Serial,"-- Init complete --\n\r");
}
//
//===========================================================================
//
unsigned int LastScanMilli;
unsigned int DeltaScanMilli;
unsigned int CurScanMilli;

void UpdateSrvrVariablesEveryScan()
{
//
//    Calculate MS since last processing
//    Read millisecond counter
//    Calculate change since last scan
//    Update server register with Delta
//    Store current millie reading for next iteration delta calc
//  
  float CurMotion;
  
  CurScanMilli = millis();
  DeltaScanMilli = CurScanMilli-LastScanMilli;
  ServerStats[STAT_SCAN_MS] = DeltaScanMilli;
  LastScanMilli = CurScanMilli; 

  int rssi = WiFi.RSSI();
  ServerStats[STAT_WIFI_RSSI] = float(rssi);
//
  CurMotion = digitalRead(MOTION_PIN);
  // Store instant Discrete in value
  //Delay off for TimeDelayOff MS
  ServerStats[STAT_MOTION_PIN] = float(CurMotion);
  if (CurMotion == 1) {
    LastOn = millis();  
    ServerStats[STAT_MOTION_PIND] = float(CurMotion);
  } else {
    if ((millis()-LastOn)>OFFDELAYMS) {
     ServerStats[STAT_MOTION_PIND] = float(CurMotion);
    }
  }
  if (CurMotion != LastMotion) { // change detected
    if (CurMotion==1) {
      MotionCount++;
      ServerStats[STAT_MOTION_CNT] = float(MotionCount);
    }
    LastMotion = CurMotion;
  }
  
}
//
//===========================================================================
//
void UpdatePVsEveryScan()
{
  float curvalue;
  char fstr[11];  

  
#ifdef USE_BMP280_1      
   if (BMP1State==true){
        curvalue=bmp1.readTemperature();
        PV_values[BMP1_TEMP]=curvalue;

        curvalue=bmp1.readPressure() / 100.0F;
        PV_values[BMP1_PRESS]=curvalue;
   } else {
        curvalue=-9999.0;
        PV_values[BMP1_TEMP]=curvalue;
        PV_values[BMP1_PRESS]=curvalue;
   }
#endif
#ifdef USE_BMP280_2
   if (BMP2State==true){
        curvalue=bmp2.readTemperature();
        PV_values[BMP2_TEMP]=curvalue;

        curvalue=bmp2.readPressure() / 100.0F;
        PV_values[BMP2_PRESS]=curvalue;
   } else {
        curvalue=-9999.0;
        PV_values[BMP2_TEMP]=curvalue;
        PV_values[BMP2_PRESS]=curvalue;
   }
#endif      
#ifdef USE_TSL2561
    if (TSLState==true) {
         if (tsl.ReadLightNoBlock()) { // try until it completes
           curvalue = tsl.LightReading();
           PV_values[TSL_LUMIN]=curvalue;

           curvalue = tsl.BroadbandReading();
           PV_values[TSL_BB]=curvalue;

           curvalue = tsl.irReading();
           PV_values[TSL_IR]=curvalue;
         }
    } else {
        curvalue=-9999.0;
        PV_values[TSL_LUMIN]=curvalue;
        PV_values[TSL_BB]=curvalue;
        PV_values[TSL_IR]=curvalue;
    }
#endif
  
}
//
//===========================================================================
//
void UpdatePVsSomeScans()
{
  float curvalue;
  char fstr[11];  
  
// -----  Update register entries for HTU21 #1                
#ifdef USE_HTU21_1    
  if (HTU1State==true) {
        curvalue=htu1.readTemperature();
        if (curvalue==NAN) { delay(5); curvalue=htu1.readTemperature();} //try twice
        PV_values[HTU1_TEMP]=curvalue;

        curvalue=htu1.readHumidity();
        if (curvalue==NAN) { delay(5); curvalue=htu1.readHumidity();} //try twice
        PV_values[HTU1_HUMIDITY]=curvalue;
  } else {
        curvalue=-9999.0;
        PV_values[HTU1_TEMP]=curvalue;
        PV_values[HTU1_HUMIDITY]=curvalue;
  }
#endif      

#ifdef USE_HTU21_2    
  if (HTU2State==true) {
        curvalue=htu2.readTemperature();
        if (curvalue==NAN) { delay(5); curvalue=htu2.readTemperature();} //try twice
        PV_values[HTU2_TEMP]=curvalue;

        curvalue=htu2.readHumidity();
        if (curvalue==NAN) { delay(5); curvalue=htu2.readHumidity();} //try twice
        PV_values[HTU2_HUMIDITY]=curvalue;
  } else {
        curvalue=-9999.0;
        PV_values[HTU2_TEMP]=curvalue;
        PV_values[HTU2_HUMIDITY]=curvalue;
   }
#endif      

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
           PV_values[DS18_0+currentDS18] = curvalue;
           currentDS18++;
           if ((currentDS18>=ds.getNumberOfDevices())||(currentDS18>=MAXDS)) currentDS18 = 0;
      }
    } else {
           curvalue=-9999.0;
           for (int i=0;i<MAXDS;i++) { 
              PV_values[DS18_0+i] = curvalue;
            }
    }
#endif
  
}
//
//==============================================================================================
//
// set up macro to place float in proper byte order for register storage
// only uncomment 1 of the macros
//
//#define USE_SET_FLOAT_ORDER modbus_set_float_abcd
#define USE_SET_FLOAT_ORDER modbus_set_float_dcba
//#define USE_SET_FLOAT_ORDER modbus_set_float_badc
//#define USE_SET_FLOAT_ORDER modbus_set_float_cdab
//#define USE_SET_FLOAT_ORDER modbus_set_float
//
//===========================================================================
//
// Store floating value into selected register pair
//
#ifdef USE_MODBUS
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
#endif
//
//============================================================================
//
// Store integer value into selected register
//
#ifdef USE_MODBUS
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
#endif
//
//===========================================================================
//
// Called from main loop at UPDATERATEMS millisecond period
//
// Reads each device configured/available, and stores result into associated
// modbus register (floating and integer)
//
#ifdef USE_MODBUS
void UpdateModbusRegistersEveryScan()
{
  modbusTCPServer.inputRegisterWrite(SCAN_MS_BASE, ServerStats[STAT_SCAN_MS]); 
  modbusTCPServer.inputRegisterWrite(WIFI_RSSI_BASE, ServerStats[STAT_WIFI_RSSI]);      

  modbusTCPServer.inputRegisterWrite(MOTION_PIN_BASE,ServerStats[STAT_MOTION_PIN]);      
  modbusTCPServer.inputRegisterWrite(MOTION_PIN_BASED,ServerStats[STAT_MOTION_PIND]);      
  modbusTCPServer.inputRegisterWrite(MOTION_CNT_BASE,ServerStats[STAT_MOTION_CNT]);      

  UpdateModbusFloatRegister(HTU1_FP_TEMP_BASE,PV_values[HTU1_TEMP]);
  UpdateModbusInt16Register(HTU1_I16_TEMP_BASE,PV_values[HTU1_TEMP]);

  UpdateModbusFloatRegister(HTU1_FP_HUMID_BASE,PV_values[HTU1_HUMIDITY]);
  UpdateModbusInt16Register(HTU1_I16_HUMID_BASE,PV_values[HTU1_HUMIDITY]);

  UpdateModbusFloatRegister(HTU2_FP_TEMP_BASE,PV_values[HTU2_TEMP]);
  UpdateModbusInt16Register(HTU2_I16_TEMP_BASE,PV_values[HTU2_TEMP]);

  UpdateModbusFloatRegister(HTU2_FP_HUMID_BASE,PV_values[HTU2_HUMIDITY]);
  UpdateModbusInt16Register(HTU2_I16_HUMID_BASE,PV_values[HTU2_HUMIDITY]);

  UpdateModbusFloatRegister(BMP1_FP_TEMP_BASE,PV_values[BMP1_TEMP]);
  UpdateModbusInt16Register(BMP1_I16_TEMP_BASE,PV_values[BMP1_TEMP]);

  UpdateModbusFloatRegister(BMP1_FP_PRESS_BASE,PV_values[BMP1_PRESS]);
  UpdateModbusInt16Register(BMP1_I16_PRESS_BASE,PV_values[BMP1_PRESS]);

   UpdateModbusFloatRegister(BMP2_FP_TEMP_BASE,PV_values[BMP2_TEMP]);
   UpdateModbusInt16Register(BMP2_I16_TEMP_BASE,PV_values[BMP2_TEMP]);

   UpdateModbusFloatRegister(BMP2_FP_PRESS_BASE,PV_values[BMP2_PRESS]);
   UpdateModbusInt16Register(BMP2_I16_PRESS_BASE,PV_values[BMP2_PRESS]);

   UpdateModbusFloatRegister(TSL_FP_LUMEN_BASE,PV_values[TSL_LUMIN]);
   UpdateModbusInt16Register(TSL_I16_LUMEN_BASE,PV_values[TSL_LUMIN]);

   UpdateModbusFloatRegister(TSL_FP_BB_BASE,PV_values[TSL_BB]);
   UpdateModbusInt16Register(TSL_I16_BB_BASE,PV_values[TSL_BB]);

   UpdateModbusFloatRegister(TSL_FP_IR_BASE,PV_values[TSL_IR]);
   UpdateModbusInt16Register(TSL_I16_IR_BASE,PV_values[TSL_IR]);

}
#endif
//
//===========================================================================
//
#ifdef USE_MODBUS
void UpdateModbusCoilsEveryScan()
{
  float curvalue;
  char fstr[11];  
  long CurDO;
  long LastDO;
  int DOPin;
  int i;
//  // read the current value of the coil
//  int coilValue = modbusTCPServer.coilRead(0x00);

//  if (coilValue) {
//    // coil value set, turn LED on
//    digitalWrite(ledPin, HIGH);
//  } else {
//    // coild value clear, turn LED off
//    digitalWrite(ledPin, LOW);
//  }

  DOPin = -1;
  for (i=DO_BASE;i<=MAXDOS;i++) {
    CurDO = modbusTCPServer.coilRead(DO_BASE+i);
    switch (i) {
      case 0:
        #ifdef DO0_PIN      
          DOPin = DO0_PIN;
        #else
          DOPin = -1;
        #endif
        break;
      case 1:
        #ifdef DO1_PIN      
          DOPin = DO1_PIN;
        #else
          DOPin = -1;
        #endif
        break;
      case 2:
        #ifdef DO2_PIN      
          DOPin = DO2_PIN;
        #else
          DOPin = -1;
        #endif
        break;
      case 3:
        #ifdef DO3_PIN      
          DOPin = DO3_PIN;
        #else
          DOPin = -1;
        #endif
        break;
      case 4:
        #ifdef DO4_PIN      
          DOPin = DO4_PIN;
        #else
          DOPin = -1;
        #endif
        break;
      case 5:
        #ifdef DO5_PIN      
          DOPin = DO5_PIN;
        #else
          DOPin = -1;
        #endif
        break;
      case 6:
        #ifdef DO6_PIN      
          DOPin = DO6_PIN;
        #else
          DOPin = -1;
        #endif
        break;
      case 7:
        #ifdef DO7_PIN      
          DOPin = DO7_PIN;
        #else
          DOPin = -1;
        #endif
        break;
      default:
        DOPin = -1;
    }
    if (DOPin != -1) {
      LastDO = digitalRead(DOPin);
      if (CurDO==0) {
        digitalWrite(DOPin,LOW);
      } else {
        digitalWrite(DOPin,HIGH);
      }
      if (LastDO != CurDO) {
        printf (Serial,"COS: DO[%d] Pin:%d From: %2.2X  To: %2.2X\n\r",i,DOPin,LastDO,CurDO);
      }
    }
  }

// 01-23-2021 end add DO functionality

}
#endif
//
//===========================================================================
//
// Called from main loop at UPDATERATEMS millisecond period
//
// Reads each device configured/available, and stores result into associated
// modbus register (floating and integer)
//
#ifdef USE_MODBUS
void UpdateModbusRegistersDS()
{
  for (int i=0;i<MAXDS;i++) { 
      UpdateModbusFloatRegister(DS18_FP_BASE+(i*2),PV_values[DS18_0+i]);
      UpdateModbusInt16Register(DS18_I16_BASE+i,PV_values[DS18_0+i]);
  }
}
#endif
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
#ifdef USE_MQTT
void UpdateMQTTtopics()
{
  char topic[400];
  char fstr[11];  
  
// update topics for MQTT client

  // server stat topics
  for (int i=0;i<NUM_SERVER_STATS;i++){
    sprintf(topic,"%s%s",MQTTtopicPrefix,MQTTtopicServerStats[i]);    
#ifdef DEBUG
    printf(Serial,"topic: %s = %d\n\r",topic,ServerStats[i]);
#endif    
    if (mqttClient.connected()) {
        mqttClient.beginMessage(topic);
        mqttClient.print(ServerStats[i]);
        mqttClient.endMessage();
     } else {
        printf(Serial,"UpdateMQTTtopics: MQTT not connected at STAT:%d\n\r",i);
        MQTTState = 0;
        return;    
     }
  }
  // device PV topics
  for (int i=0;i<NUM_PVS;i++){
    sprintf(topic,"%s%s",MQTTtopicPrefix,MQTTtopicPV[i]);    
#ifdef DEBUG
    dtostrf(PV_values[i],8,2,fstr);
    printf(Serial,"topic: %s = %s\n\r",topic,fstr);
#endif
    if (mqttClient.connected()) {
        mqttClient.beginMessage(topic);
        mqttClient.print(PV_values[i]);
        mqttClient.endMessage();
     } else {
        printf(Serial,"UpdateMQTTtopics: MQTT not connected at PV:%d\n\r",i);
        MQTTState = 0;
        return;    
     }
  } 
}
#endif
//
//=================================================================================
//
#ifdef USE_MODBUS

  WiFiClient client;
void update_modbus()
{
    // if not connected, listen for client
  if(!client) {
    client = wifiServer.available();
    if (client) {
      modbusTCPServer.inputRegisterWrite(SRVR_CLIENT_CNT_BASE, ++ClientCount); 
      ServerStats[STAT_CLIENT_CNT] = ClientCount;
      // let the Modbus TCP accept the connection 
      modbusTCPServer.accept(client);
      // new client
      printf(Serial,"Connect from Client - %s:%d\n\r",client.remoteIP().toString().c_str(), client.remotePort());
    }
  }
  
  if (client) {
//
//  process function requests from client until disconnected
//
    if (client.connected()) {
      UpdateModbusRegistersDS();
      UpdateModbusRegistersEveryScan();
      UpdateModbusCoilsEveryScan();
    }
  }
  // poll for Modbus TCP requests, while client connected
  modbusTCPServer.poll();
}
#endif
//
//===========================================================================
//
#ifdef USE_MQTT
void update_MQTT()
{
//
// update broker at set interval with topics
//
      delta = CurScanMilli - MQTTpreviousMillis;
      if (delta >= MQTTinterval ) {
        MQTTpreviousMillis = CurScanMilli;

        if (MQTTState == 0) {
          if (!mqttClient.connect(MQTTbroker,MQTTport)) { // try again to connect
              MQTTState = 0;
      printf(Serial,"MQTT connection failed %s:%d - Error: ",MQTTbroker, MQTTport);
      Serial.println(mqttClient.connectError());
           } else {
              MQTTState = 1;
              printf(Serial,"MQTT connected at %s:%d\n\r",MQTTbroker, MQTTport);
           }  
        }

        if (MQTTState == 1) {
          UpdateMQTTtopics();
        }
      }
  mqttClient.poll();
}
#endif
//
//============================================================================
//
// main loop
// Wait for client connection and process modbus commands
// Note this code steps on registers each iteration,
// no device updates to holding registers are retained.
//
void loop() {

// Allow OTA if requested
#ifdef USE_OTA
  ArduinoOTA.handle();
#endif  

//
// Read devices into value arrays
//
 UpdateSrvrVariablesEveryScan();
 UpdatePVsEveryScan();

 delta = CurScanMilli-MainLoopLastRun;
 if (delta >= UPDATERATEMS) {
   MainLoopLastRun = CurScanMilli; 
   UpdatePVsSomeScans();
 }

#ifdef USE_MODBUS
  update_modbus();
#endif

#ifdef USE_MQTT
  update_MQTT();
#endif

}
