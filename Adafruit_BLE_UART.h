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


extern "C" 
{
  /* Callback prototypes */
  typedef void (*aci_callback)(aci_evt_opcode_t event);
  typedef void (*rx_callback) (uint8_t *buffer, uint8_t len);
}

class Adafruit_BLE_UART
{
 public:
  Adafruit_BLE_UART ( aci_callback aciEvent, rx_callback rxEvent, bool debug = false);
  
  bool begin   ( uint16_t advTimeout = 0, uint16_t advInterval = 80 );
  void pollACI ( void );
  void write   ( uint8_t * buffer, uint8_t len );
  
 private:  
  aci_callback aci_event;
  rx_callback  rx_event; 
  bool         debugMode;
  uint16_t     adv_timeout;
  uint16_t     adv_interval;
};
