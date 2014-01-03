/*********************************************************************
This is a library for our nRF8001 Bluetooth Low Energy Breakout

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Kevin Townsend/KTOWN  for Adafruit Industries.  
MIT license, check LICENSE for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

class UartService {
 public:
  UartService ( );
  bool begin ( void );
  void pollACI ( void );

 private:
  void uart_tx();
 
};
