# The mower: Arduino

## Technology
The Arduino incorperates some open source libraries, provided by Arduino and/or Makeblock, in order to work properly:

- [HardwareSerial](https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/HardwareSerial.h)
- [Wire](https://github.com/Makeblock-official/Makeblock-Libraries/blob/master/src/utility/Wire.h)
- [SoftwareSerial](https://github.com/Makeblock-official/Makeblock-Libraries/blob/master/src/utility/SoftwareSerial.h)
- [Wire](https://github.com/Makeblock-official/Makeblock-Libraries/blob/master/src/utility/Wire.h)
- [MeAuriga](https://github.com/Makeblock-official/Makeblock-Libraries/blob/master/src/MeAuriga.h)
- [stdlib](https://github.com/Patapom/Arduino/blob/master/Libraries/AVR%20Libc/avr-libc-2.0.0/include/stdlib.h)
- [Time](https://github.com/PaulStoffregen/Time)
- [Arduino](https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/Arduino.h)

## Serial communication messages
| During state      | Message    | Incoming/outgoing | Description |
| -----------       | -------    | -------           | ----------- |
| Any               | AM         | Incoming          | Enter auto mode |
| Any               | MM         | Incoming          | Enter manual mode |
| Any               | (X,Y)      | Outgoing          | Coordinates of the robot's current position. Sent every 250 milliseconds and/or when an obstacle appears. Calculated as cm from start position |
| Any               | UC         | Outgoing          | If an unknown command is sent to the Arduino, it will respond with UC |
| Manual mode       | MF         | Incoming          | Manual forward |
| Manual mode       | MB         | Incoming          | Manual backwards |
| Manual mode       | ML         | Incoming          | Manual left |
| Manual mode       | MR         | Incoming          | Manual right |
| Auto mode         | LT         | Incoming          | Lidar triggered |
| Auto mode         | LOK        | Outgoing          | Arduino has reacted to lidar triggered, Arduino is now waiting for Raspberry Pi to take picture |
| Auto mode         | PT         | Incoming          | Picture taken after lidar triggered |
