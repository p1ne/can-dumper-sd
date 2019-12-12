# can-dumper-sd

This project is intended to dump selected CAN bus messages to SD card. Since both CAN interface and SD card controller are sitting on single SPI interface - heavy filtering is applied, instead massive CAN messages lost appear while data is saved to SD.

This particular code is designed for Audi (A4B8/Q5 platform) to track RPM, cylinder misfires and cylinder ignition retardation angles. Misfires generate beeps using beeper wired to Analog output of Arduino.

## Hardware used

* Generic CAN device I made for fdim-controller project - see https://github.com/p1ne/fdim-controller
* Amperka Micro SD Troyka module - see http://wiki.amperka.ru/%D0%BF%D1%80%D0%BE%D0%B4%D1%83%D0%BA%D1%82%D1%8B:troyka-sd
* Beeper

SCK, DI, DO, V and G pins of Micro SD module are soldered in parallel to MCP2515 CAN module pins

CS pin of Amperka module is wired to pin 9 of Arduino Pro Micro

Beeper is wired to A2 and GND pins of Arduino Pro Micro

## CAN messages

Response messages:

All messages are 8 bytes long

7E8 05 62 20 0x6F rpm1 rpm2 - RPM message where RPM value is rpm1*256+rpm2

7E8 05 62 20 cyl ret1 ret2 - Retardation angles message, where cyl is 0A-0D for cylinders 1-4, and ret1*256+ret2 is signed int negative HEX value of angle

7E8 05 62 29 cyl misf1 misf2 - Misfire for last 1000 RPM message, where cyl is 1D-20 for cylinders 1-4 and misf1*256+misf2 is number of misfires for last 1000 RPM.

Request messages are the same, but with 7E0 instead of 7E8 and 0x55 in last 4 bytes. Message length is 8 bytes.

## Work logic

all messages but 0x7E8 are filtered by CAN masks/filters to lower CAN traffic that triggers MCP2515 interrupt

only messages listed above are starting processing cycle

RPM values are saved each time we get RPM message

Request messages are sent each 200 msec in 20 msec-delimited bursts - except for retardation request messages - I have it sent with another CAN device.

For misfires only 1st misfire is saved. If more misfires are coming within 1000 RPM - they are saved too. But in order to save log file size, we don't save subsequent messages with the same misfire value. Upon receiption of misfire message 1000 Hz 500msec long beep is generated through beeper

