/*!
 * @file Adafruit_BLE_UART.h
 */
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
All text above, and the splash screen below must be included in any
redistribution
*********************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifndef _ADAFRUIT_BLE_UART_H_
#define _ADAFRUIT_BLE_UART_H_

#include "utility/aci_evts.h"

/* Uncomment the next line to display debug messages when reading/writing
 * packets */
//#define BLE_RW_DEBUG

extern "C" {
/*! Callback prototype */
typedef void (*aci_callback)(aci_evt_opcode_t event);
/*! Callback prototype */
typedef void (*rx_callback)(uint8_t *buffer, uint8_t len);
}

/*!
 * @brief Class that stores the states and functions for the nRF8001
 */
class Adafruit_BLE_UART : public Stream {
public:
  Adafruit_BLE_UART(int8_t req, int8_t rdy, int8_t rst);

  bool begin(uint16_t advTimeout = 0, uint16_t advInterval = 80);
  void pollACI(void);
  /*!
   * @brief Writes over ble uart
   * @param buffer Pointer to the buffer to write to
   * @param len Length to write
   * @return Returns what was sent
   */
  size_t write(uint8_t *buffer, uint8_t len);
  /*!
   * @brief Writes over ble uart
   * @param buffer Buffer to write to
   * @return Returns what was sent
   */
  size_t write(uint8_t buffer);

  size_t println(const char *thestr);
  /*!
   * @brief Prints a character
   * @param thestr Character to print
   * @return Returns what was written
   */
  size_t print(const char *thestr);
  /*!
   * @brief Prints a string
   * @param thestr String to print
   * @return Returns what was written
   */
  size_t print(String thestr);
  /*!
   * @brief Prints an integer
   * @param theint Integer to print
   * @return Returns what was written
   */
  size_t print(int theint);
  /*!
   * @brief Prints a flash string
   * @param ifsh Pointer to flash string
   * @return Returns what was written
   */
  size_t print(const __FlashStringHelper *ifsh);

  /*!
   * @brief Sets the ACI callback
   * @param aciEvent Event to set the ACI callback to
   */
  void setACIcallback(aci_callback aciEvent = NULL);
  /*!
   * @brief Sets the RX callback
   * @param rxEvent Event to set the RX callback to
   */
  void setRXcallback(rx_callback rxEvent = NULL);
  /*!
   * @brief Sets the device name
   * @param deviceName Name to set the device
   */
  void setDeviceName(const char *deviceName);

  // Stream compatibility
  /*!
   * @brief Finds if the stream is available
   * @return Returns if the stream is available
   */
  int available(void);
  /*!
   * @brief Reads the stream
   * @return Returns the characters read from the stream
   */
  int read(void);
  /*!
   * @brief Performs a peek operation on the stream
   * @return Returns the next character in the stream
   */
  int peek(void);
  /*!
   * @brief Clears the stream
   */
  void flush(void);

  /*!
   * @brief Gets the state of the device
   * @return Returns the current status
   */
  aci_evt_opcode_t getState(void);

private:
  void defaultACICallback(aci_evt_opcode_t event);
  void defaultRX(uint8_t *buffer, uint8_t len);

  // callbacks you can set with setCallback function for user extension
  aci_callback aci_event;
  rx_callback rx_event;

  bool debugMode;
  uint16_t adv_timeout;
  uint16_t adv_interval;
  char device_name[8];

  aci_evt_opcode_t currentStatus;

  // pins usd
  int8_t _REQ, _RDY, _RST;
};

#endif
