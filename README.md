Adafruit_nRF8001
================

Driver and example code for Adafruit's nRF8001 Bluetooth Low Energy Breakout.

PINOUT
======

The pin locations are defined in **ble_system.h**. The following pinout is used by default for the Arduino Uno:

* SCK -> Pin 13
* MISO -> Pin 12
* MOSI -> Pin 11
* REQ -> Pin 10
* RDY -> Pin 3 (HW interrupt)
* ACT -> Pin 8 (optional)
* RST -> Pin 9
* 3V0 - > Not connected
* GND -> GND
* VIN -> 5V

RDY must be on pin 3 since this pin requires a HW interrupt.

3V0 is an optional pin that exposes the output of the on-board 3.3V regulator. You can use this to supply 3.3V to other peripherals, but normally it will be left unconnected.

ACT is not currently used in any of the existing examples, and can be left unconnected if necessary.