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

#ifndef _ADAFRUIT_BLE_UART_H_
#define _ADAFRUIT_BLE_UART_H_

#include "utility/aci_evts.h"


extern "C" 
{
  /* Callback prototypes */
  typedef void (*aci_callback)(aci_evt_opcode_t event);
  typedef void (*rx_callback) (uint8_t *buffer, uint8_t len);
}

class Adafruit_BLE_UART : Print
{
 public:
  Adafruit_BLE_UART (int8_t req, int8_t rdy, int8_t rst);
  
  bool begin   ( uint16_t advTimeout = 0, uint16_t advInterval = 80 );
  void pollACI ( void );
  uint16_t write   ( uint8_t * buffer, uint8_t len );  
  uint16_t write ( uint8_t buffer);
  
  void setACIcallback(aci_callback aciEvent = NULL);
  void setRXcallback(rx_callback rxEvent = NULL);

  uint16_t available(void);
  uint16_t read(void);
  aci_evt_opcode_t getState(void);

 private:  
  void defaultACICallback(aci_evt_opcode_t event);
  void defaultRX(uint8_t *buffer, uint8_t len);

  // callbacks you can set with setCallback function for user extension
  aci_callback aci_event;
  rx_callback  rx_event; 

  bool         debugMode;
  uint16_t     adv_timeout;
  uint16_t     adv_interval;

  aci_evt_opcode_t currentStatus;
  
  // pins usd
  int8_t _REQ, _RDY, _RST;
};

#endif
