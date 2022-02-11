//
// config.h
//
#ifndef _CONFIG_H
#define _CONFIG_H

#define SRVR_DATE "02-19-2021  Author: Stu Richmond"
#define SRVR_MAJOR 1
#define SRVR_MINOR 13

//
// Unomment USE_OTA to have build include Over The Air programming code
//
#define USE_OTA
//
// Pin to associate motion detector input (on motion detected, off normal)
//
#define MOTION_PIN D0
//
// Pins for Digital output control
//
#define USE_DO

#ifdef USE_DO
#define DO0_PIN D5
#define DO1_PIN D6
#endif

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
// include output to serial debug messages
//
//#define DEBUG
//
// include scan of I2C bus and print results to serial
//
#define DEBUGI2C


#define INITIAL_DO_STATE LOW
//
//Caution: Keep UPDATERATEDS18 > UPDATERATEMS
//
#define UPDATERATEMS 1000
#define UPDATERATEDS18 10000



#endif
