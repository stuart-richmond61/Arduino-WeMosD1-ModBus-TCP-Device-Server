# Arduino-WeMosD1-ModBus-TCP-Device-Server
Provides ModBus/TCP server on port 502 (modbus) via WIFI.

Supports
	BMP280 (Up to 2)
	HTU21 (Up to 2)
	TSL2561 (1)
	DS18B20 (Up to 8)
	TTL input state change counter (1)

------------------------------------------------------
Arduino IDE setup used for my Wemos D1 Mini 

	Board: "LOLIN(WEMOS) D1 R2 & mini"
	Upload Speed: "921600"
	CPU Frequency: "80 MHz"
	Flash Size: "4MB (FS:2MB OTA:~1019KB"
	Debug port: "Disabled"
	Debug Level: "None"
	IwIP Variant: "v2 Lower Memory"
	VTables: "Flash"

Note: First use Board manager to install support for ESP8266 boards for Wemos D1 Mini if not already done
  
------------------------------------------------------
ModBus Register usage

Floating point (2 register values) device readings

    40001  R(0) R(1)  BMP/E #1 Temperature
    40003  R(2) R(3)  BMP/E #1 Pressure
    40005  R(4) R(5)  BME #1 Humidity (if build using BME280)
    40007  R(6) R(7)  BMP/E #2 Temperature
    40009  R(8) R(9)  BMP/E #2 Pressure
    40011  R(10)R(11) BME #2 Humidity (if build using BME280)
    40021  R(20) R(21) HTU21 #1 Humidity
    40023  R(22) R(23) HTU21 #1 Temperature
    40025  R(24) R(25) HTU21 #2 Humidity
    40027  R(26) R(27) HTU21 #2 Temperature

    40031  R(30) R(31) DS18B20 #0
    40033  R(32) R(33) DS18B20 #1
    40035  R(34) R(35) DS18B20 #2FI
    40037  R(36) R(37) DS18B20 #3
    40039  R(38) R(39) DS18B20 #4
    40041  R(40) R(41) DS18B20 #5
    40043  R(42) R(43) DS18B20 #6
    40045  R(44) R(45) DS18B20 #7

    40051  R(50) R(51)  TSL Luminosity reading
    40053  R(52) R(53)  TSL broadband reading
    40055  R(54) R(55)  TSL ir reading

Integer status values

    40061  R(60) Major version #
    40062  R(61) Minor version #
    40063  R(62) RSSI
    40064  R(63) BMP Count
    40065  R(64) HTU21 Count
    40066  R(65) DS18B20 Count
    40067  R(66) TSL 2561 Count
    40068  R(67) Client connection count
    40069  R(68) D0 input status
    40070  R(69) D0 change of state count (Rising edge)

Integer device readings

    40101  R(100)  BMP/E #1 Temperature
    40102  R(101)  BMP/E #1 Pressure
    40103  R(102)  BME #1 Humidity (if build using BME280)
    40104  R(103)  BMP/E #2 Temperature
    40105  R(104)  BMP/E #2 Pressure
    40106  R(105)  BME #2 Humidity (if build using BME280)
    40111  R(110) HTU21 #1 Humidity
    40112  R(111) HTU21 #1 Temperature
    40113  R(112) HTU21 #2 Humidity
    40114  R(113) HTU21 #2 Temperature
    40115  R(114) TSL Luminosity reading
    40116  R(115) TSL broadband reading
    40117  R(116) TSL ir reading

    40121  R(120) DS18B20 #0
    40122  R(121) DS18B20 #1
    40123  R(122) DS18B20 #2
    40124  R(123) DS18B20 #3
    40125  R(124) DS18B20 #4
    40126  R(125) DS18B20 #5
    40127  R(126) DS18B20 #6
    40128  R(127) DS18B20 #7
