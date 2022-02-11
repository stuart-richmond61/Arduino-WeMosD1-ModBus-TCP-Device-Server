//
// MBusconfig.h
//
#ifndef _MBUSCONFIG_H
#define _MBUSCONFIG_H

//
// uncomment to compile in modbus server code
//
#define USE_MODBUS
//
//=======================================  ModBus coil/register address usage
//
// ModBus DO Coils
//
// Coil addresses for DOs
#define DO_BASE 0
//    00001 R(0) DO 1
//    00002 R(1) DO 2
#define MAXDOS 1

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
#define MOTION_PIN_BASED 68
//    40069 R(68) Motion detector Pin Off delayed
#define MOTION_PIN_BASE 69
//    40070 R(69) Motion detector Pin Instant
#define MOTION_CNT_BASE 70
//    40071 R(70) Motion detector counter
#define SCAN_MS_BASE 71
//    40072 R(71) Scan time for loop

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

//================================
//  modbusTCPServer.configureDiscreteInputs(0x00, NUMREGISTERS);
//  modbusTCPServer.configureHoldingRegisters(0x00, NUMREGISTERS);

/*

*/
// NUMREGISTERS is used to configure modbus tables
//              this is used to allocate registers and coils
#define NUMREGISTERS 128+2


#endif
