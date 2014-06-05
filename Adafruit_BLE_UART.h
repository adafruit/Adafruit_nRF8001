/*********************************************************************
This is a library for our nRF8001 Bluetooth Low Energy Breakout

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1697

These displays use SPI to communicate, 4 or 5 pins are required to  
interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Kevin Townsend/KTOWN  for Adafruit Industries.  
MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#ifndef _ADAFRUIT_BLE_UART_H_
#define _ADAFRUIT_BLE_UART_H_

#include "utility/lib_aci.h"
#include "utility/aci_setup.h"


#define BLE_RW_DEBUG

// Presumed write delay in original Adafruit code, however
// no indiciation in SDK of a min write delay
#define BLE_W_DELAY 35 

extern "C" 
{
  /* Callback prototypes */
  typedef void (*aci_callback)(aci_evt_opcode_t event);
  typedef void (*rx_callback) (uint8_t *buffer, uint8_t len);
}



class Adafruit_BLE_UART : public Stream
{
 public:
  Adafruit_BLE_UART (int8_t req, int8_t rdy, int8_t rst);
  
  bool begin   ( uint16_t advTimeout = 0, uint16_t advInterval = 80 );
  void pollACI ( void );
  size_t write ( uint8_t * buffer, uint8_t len );
  size_t write ( uint8_t buffer);
  size_t print(const char * thestr);
  size_t println(const char * thestr);

  void setACIcallback(aci_callback aciEvent = NULL);
  void setRXcallback(rx_callback rxEvent = NULL);

  // Stream compatibility
  int available(void);
  int read(void);
  int peek(void);
  void flush(void);

  aci_evt_opcode_t getState(void);

 private:  
  void defaultACICallback(aci_evt_opcode_t event);
  void defaultRX(uint8_t *buffer, uint8_t len);
  
  void uart_over_ble_init(void);
  bool uart_tx(uint8_t *buffer, uint8_t buffer_len);
  bool uart_process_control_point_rx(uint8_t *byte, uint8_t length);

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
