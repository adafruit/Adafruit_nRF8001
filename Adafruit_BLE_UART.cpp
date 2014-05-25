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

Updated by Jonathan Fontanez/tato123 for Adafruit Industries.
MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/
#include <SPI.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include <lib_aci.h>
#include <aci_setup.h>
#include "uart/services.h"
#include "uart/uart_over_ble.h"

#include "Adafruit_BLE_UART.h"

/* Get the service pipe data created in nRFGo Studio */
#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
    static services_pipe_type_mapping_t
        services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
    #define NUMBER_OF_PIPES 0
    static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif

/* Store the setup for the nRF8001 in the flash of the AVR to save on RAM */
static const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;

static struct aci_state_t aci_state;            /* ACI state data */
static hal_aci_evt_t  aci_data;                 /* Command buffer */
static bool timing_change_done = false;

// This is the Uart RX buffer, which we manage internally when data is available!
#define ADAFRUIT_BLE_UART_RXBUFFER_SIZE 64
uint8_t adafruit_ble_rx_buffer[ADAFRUIT_BLE_UART_RXBUFFER_SIZE];
volatile uint16_t adafruit_ble_rx_head;
volatile uint16_t adafruit_ble_rx_tail;

static uart_over_ble_t uart_over_ble;
static uint8_t         uart_buffer[20];
static uint8_t         uart_buffer_len = 0;
static uint8_t         dummychar = 0;

/* Define how assert should function in the BLE library */
void __ble_assert(const char *file, uint16_t line)
{
  Serial.print("ERROR ");
  Serial.print(file);
  Serial.print(": ");
  Serial.print(line);
  Serial.print("\n");
  while(1);
}


/**************************************************************************/
/*!
    Constructor for the UART service
*/
/**************************************************************************/
// default RX callback!

void Adafruit_BLE_UART::defaultRX(uint8_t *buffer, uint8_t len)
{
  for(int i=0; i<len; i++)
  {
    uint16_t new_head = (uint16_t)(adafruit_ble_rx_head + 1) % ADAFRUIT_BLE_UART_RXBUFFER_SIZE;
    
    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (new_head != adafruit_ble_rx_tail) {
      adafruit_ble_rx_buffer[adafruit_ble_rx_head] = buffer[i];

      // debug echo print
      // Serial.print((char)buffer[i]); 

      adafruit_ble_rx_head = new_head;
    }
  }

  /*
  Serial.print("Buffer: ");
  for(int i=0; i<adafruit_ble_rx_head; i++)
    {
      Serial.print(" 0x"); Serial.print((char)adafruit_ble_rx_buffer[i], HEX); 
    }
  Serial.println();
  */
}


/* Stream stuff */

int Adafruit_BLE_UART::available(void)
{
  return (uint16_t)(ADAFRUIT_BLE_UART_RXBUFFER_SIZE + adafruit_ble_rx_head - adafruit_ble_rx_tail) 
    % ADAFRUIT_BLE_UART_RXBUFFER_SIZE;
}

int Adafruit_BLE_UART::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (adafruit_ble_rx_head == adafruit_ble_rx_tail) {
    return -1;
  } else {
    unsigned char c = adafruit_ble_rx_buffer[adafruit_ble_rx_tail];
    adafruit_ble_rx_tail ++;
    adafruit_ble_rx_tail %= ADAFRUIT_BLE_UART_RXBUFFER_SIZE;
    return c;
  }
}

int Adafruit_BLE_UART::peek(void)
{
  if (adafruit_ble_rx_head == adafruit_ble_rx_tail) {
    return -1;
  } else {
    return adafruit_ble_rx_buffer[adafruit_ble_rx_tail];
  }
}

void Adafruit_BLE_UART::flush(void)
{
  // MEME: KTOWN what do we do here?
}



//// more callbacks

void Adafruit_BLE_UART::defaultACICallback(aci_evt_opcode_t event)
{
  currentStatus = event;
}

aci_evt_opcode_t Adafruit_BLE_UART::getState(void) {
  return currentStatus;
}



/**************************************************************************/
/*!
    Constructor for the UART service
*/
/**************************************************************************/
Adafruit_BLE_UART::Adafruit_BLE_UART(int8_t req, int8_t rdy, int8_t rst)
{
  debugMode = true;

  rx_event = NULL;
  aci_event = NULL;

  adafruit_ble_rx_head = adafruit_ble_rx_tail = 0;

  currentStatus = ACI_EVT_DISCONNECTED;

  _RST = rst;
  _REQ = req;
  _RDY = rdy;
  
}

void Adafruit_BLE_UART::setACIcallback(aci_callback aciEvent) {
  aci_event = aciEvent;
}

void Adafruit_BLE_UART::setRXcallback(rx_callback rxEvent) {
  rx_event = rxEvent;
}

/**************************************************************************/
/*!
    Transmits data out via the TX characteristic (when available)
*/
/**************************************************************************/
size_t Adafruit_BLE_UART::println(const char * thestr)
{
  uint8_t len     = strlen(thestr),
          written = len ? write((uint8_t *)thestr, len) : 0;
  if(written == len) written += write((uint8_t *)"\r\n", 2);

  return written;
}

size_t Adafruit_BLE_UART::print(const char * thestr)
{
  return write((uint8_t *)thestr, strlen(thestr));
}


size_t Adafruit_BLE_UART::write(uint8_t * buffer, uint8_t len)
{
  
  uint8_t bytesThisPass, sent = 0;

  #ifdef BLE_RW_DEBUG
    Serial.print(F("\tWriting out to BTLE:"));
    for (uint8_t i=0; i<len; i++) {
      Serial.print(F(" 0x")); Serial.print(buffer[i], HEX);
    }
    Serial.println();
  #endif

  while(len) { // Parcelize into chunks
    bool status = false;
    bytesThisPass = len;
    if(bytesThisPass > ACI_PIPE_TX_DATA_MAX_LEN)
       bytesThisPass = ACI_PIPE_TX_DATA_MAX_LEN;

    if(!lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX))
    {
      pollACI();
      break;
    }

    status = lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, &buffer[sent],
      bytesThisPass);
    if ( status )
    {
      aci_state.data_credit_available--;
    }
    

    delay(BLE_W_DELAY); // required delay between sends

    if(!(len -= bytesThisPass)) break;
    sent += bytesThisPass;
  }

  return sent;
}

size_t Adafruit_BLE_UART::write(uint8_t buffer)
{
  bool    status = false;
  size_t  sent   = 0;

  #ifdef BLE_RW_DEBUG
    Serial.print(F("\tWriting one byte 0x")); Serial.println(buffer, HEX);
  #endif
  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX))
  {
    // Get back whether we actually sent this bit or not
    status = lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, &buffer, 1);
    
    // Validate that we actually sent the bit then
    // move the counter
    if ( status )
    {
      aci_state.data_credit_available--;  
      sent = 1;
    }
    
    delay(BLE_W_DELAY); // required delay between sends
    
    return sent;
  }

  pollACI();
  
  return sent;
}

void Adafruit_BLE_UART::uart_over_ble_init(void)
{
  uart_over_ble.uart_rts_local = true;
}

bool Adafruit_BLE_UART::uart_tx(uint8_t *buffer, uint8_t buffer_len)
{
  bool status = false;

  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) &&
      (aci_state.data_credit_available >= 1))
  {
    status = lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, buffer, buffer_len);
    if (status)
    {
      aci_state.data_credit_available--;
    }
  }

  return status;
}

bool Adafruit_BLE_UART::uart_process_control_point_rx(uint8_t *byte, uint8_t length)
{
  bool status = false;
  aci_ll_conn_params_t *conn_params;

  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_TX) )
  {
    Serial.println(*byte, HEX);
    switch(*byte)
    {
      /*
      Queues a ACI Disconnect to the nRF8001 when this packet is received.
      May cause some of the UART packets being sent to be dropped
      */
      case UART_OVER_BLE_DISCONNECT:
        /*
        Parameters:
        None
        */
        lib_aci_disconnect(&aci_state, ACI_REASON_TERMINATE);
        status = true;
        break;


      /*
      Queues an ACI Change Timing to the nRF8001
      */
      case UART_OVER_BLE_LINK_TIMING_REQ:
        /*
        Parameters:
        Connection interval min: 2 bytes
        Connection interval max: 2 bytes
        Slave latency:           2 bytes
        Timeout:                 2 bytes
        Same format as Peripheral Preferred Connection Parameters (See nRFgo studio -> nRF8001 Configuration -> GAP Settings
        Refer to the ACI Change Timing Request in the nRF8001 Product Specifications
        */
        conn_params = (aci_ll_conn_params_t *)(byte+1);
        lib_aci_change_timing( conn_params->min_conn_interval,
                                conn_params->max_conn_interval,
                                conn_params->slave_latency,
                                conn_params->timeout_mult);
        status = true;
        break;

      /*
      Clears the RTS of the UART over BLE
      */
      case UART_OVER_BLE_TRANSMIT_STOP:
        /*
        Parameters:
        None
        */
        uart_over_ble.uart_rts_local = false;
        status = true;
        break;


      /*
      Set the RTS of the UART over BLE
      */
      case UART_OVER_BLE_TRANSMIT_OK:
        /*
        Parameters:
        None
        */
        uart_over_ble.uart_rts_local = true;
        status = true;
        break;
    }
  }

  return status;
}



/**************************************************************************/
/*!
    Handles low level ACI events, and passes them up to an application
    level callback when appropriate
*/
/**************************************************************************/
void Adafruit_BLE_UART::pollACI()
{
  static bool setup_required = false;

  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data))
  {
    aci_evt_t * aci_evt;
    aci_evt = &aci_data.evt;
    switch(aci_evt->evt_opcode)
    {
      /**
      As soon as you reset the nRF8001 you will get an ACI Device Started Event
      */
      case ACI_EVT_DEVICE_STARTED:
      {
        aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
        switch(aci_evt->params.device_started.device_mode)
        {
          case ACI_DEVICE_SETUP:
            /**
            When the device is in the setup mode
            */
            Serial.println(F("Evt Device Started: Setup"));
            setup_required = true;
            break;

          case ACI_DEVICE_STANDBY:
            Serial.println(F("Evt Device Started: Standby"));
            //Looking for an iPhone by sending radio advertisements
            //When an iPhone connects to us we will get an ACI_EVT_CONNECTED event from the nRF8001
            if (aci_evt->params.device_started.hw_error)
            {
              delay(20); //Handle the HW error event correctly.
              if (debugMode) {
                Serial.println(F("Error in ACI Setup"));
              }
            }
            else
            {
              lib_aci_connect(0/* in seconds : 0 means forever */, 0x0050 /* advertising interval 50ms*/);
              defaultACICallback(ACI_EVT_DEVICE_STARTED);
              Serial.println(F("Advertising started : Tap Connect on the nRF UART app or client application"));
            }

            break;
        }
      }
      break; //ACI Device Started Event

      case ACI_EVT_CMD_RSP:
        //If an ACI command response event comes with an error -> stop
        if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
        {
          //ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
          //TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
          //all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command
          Serial.print(F("ACI Command "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
          Serial.print(F("Evt Cmd respone: Status "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_status, HEX);
        }
        if (ACI_CMD_GET_DEVICE_VERSION == aci_evt->params.cmd_rsp.cmd_opcode)
        {
          //Store the version and configuration information of the nRF8001 in the Hardware Revision String Characteristic
          lib_aci_set_local_data(&aci_state, PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET,
            (uint8_t *)&(aci_evt->params.cmd_rsp.params.get_device_version), sizeof(aci_evt_cmd_rsp_params_get_device_version_t));
        }
        break;

      case ACI_EVT_CONNECTED:
        Serial.println(F("Evt Connected"));
        uart_over_ble_init();
        timing_change_done              = false;
        aci_state.data_credit_available = aci_state.data_credit_total;

        /*
        Get the device version of the nRF8001 and store it in the Hardware Revision String
        */
        lib_aci_device_version();
        defaultACICallback(ACI_EVT_CONNECTED);
        break;

      case ACI_EVT_PIPE_STATUS:
        Serial.println(F("Evt Pipe Status"));
        if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) && (false == timing_change_done))
        {
          lib_aci_change_timing_GAP_PPCP(); // change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
                                            // Used to increase or decrease bandwidth
          timing_change_done = true;
        }
        break;

      case ACI_EVT_TIMING:
        Serial.println(F("Evt link connection interval changed"));
        lib_aci_set_local_data(&aci_state,
                                PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET,
                                (uint8_t *)&(aci_evt->params.timing.conn_rf_interval), /* Byte aligned */
                                PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET_MAX_SIZE);
        break;

      case ACI_EVT_DISCONNECTED:
        defaultACICallback(ACI_EVT_DISCONNECTED);
        Serial.println(F("Evt Disconnected/Advertising timed out"));
        lib_aci_connect(0/* in seconds  : 0 means forever */, 0x0050 /* advertising interval 50ms*/);
        defaultACICallback(ACI_EVT_DEVICE_STARTED);
        Serial.println(F("Advertising started. Tap Connect on the nRF UART app"));
        break;

      case ACI_EVT_DATA_RECEIVED:
        Serial.print(F("Pipe Number: "));
        Serial.println(aci_evt->params.data_received.rx_data.pipe_number, DEC);
        defaultRX(aci_evt->params.data_received.rx_data.aci_data, aci_evt->len - 2);
        if (rx_event)
        {          
          rx_event(aci_evt->params.data_received.rx_data.aci_data, aci_evt->len - 2);
        }
        if (PIPE_UART_OVER_BTLE_UART_RX_RX == aci_evt->params.data_received.rx_data.pipe_number)
        {
          Serial.print(F(" Data(Hex) : "));
          for(int i=0; i<aci_evt->len - 2; i++)
          {
            Serial.print((char)aci_evt->params.data_received.rx_data.aci_data[i]);
            uart_buffer[i] = aci_evt->params.data_received.rx_data.aci_data[i];
            Serial.print(F(" "));
          }
          uart_buffer_len = aci_evt->len - 2;
          Serial.println(F(""));
          if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX))
          {
            /*Do this to test the loopback otherwise comment it out*/
            /*
            if (!uart_tx(&uart_buffer[0], aci_evt->len - 2))
            {
              Serial.println(F("UART loopback failed"));
            }
            else
            {
              Serial.println(F("UART loopback OK"));
            }
            */
          }
        }
        if (PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_RX == aci_evt->params.data_received.rx_data.pipe_number)
        {
          uart_process_control_point_rx(&aci_evt->params.data_received.rx_data.aci_data[0], aci_evt->len - 2); //Subtract for Opcode and Pipe number
        }
        break;

      case ACI_EVT_DATA_CREDIT:
        aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
        break;

      case ACI_EVT_PIPE_ERROR:
        //See the appendix in the nRF8001 Product Specication for details on the error codes
        if ( debugMode )
        {
          Serial.print(F("ACI Evt Pipe Error: Pipe #:"));
          Serial.print(aci_evt->params.pipe_error.pipe_number, DEC);
          Serial.print(F("  Pipe Error Code: 0x"));
          Serial.println(aci_evt->params.pipe_error.error_code, HEX);
        }        

        //Increment the credit available as the data packet was not sent.
        //The pipe error also represents the Attribute protocol Error Response sent from the peer and that should not be counted
        //for the credit.
        if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code)
        {
          aci_state.data_credit_available++;
        }
        break;

      case ACI_EVT_HW_ERROR:
        Serial.print(F("HW error: "));
        Serial.println(aci_evt->params.hw_error.line_num, DEC);

        for(uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
        {
          Serial.write(aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
        }
        Serial.println();
        lib_aci_connect(0/* in seconds, 0 means forever */, 0x0050 /* advertising interval 50ms*/);
        Serial.println(F("Advertising started. Tap Connect on the nRF UART app"));
        break;

    }
  }
  else
  {
    if ( debugMode )
    {
      //Serial.println(F("No ACI Events available"));  
    }
    
    // No event in the ACI Event queue and if there is no event in the ACI command queue the arduino can go to sleep
    // Arduino can go to sleep now
    // Wakeup from sleep from the RDYN line
  }

  /* setup_required is set to true when the device starts up and enters setup mode.
   * It indicates that do_aci_setup() should be called. The flag should be cleared if
   * do_aci_setup() returns ACI_STATUS_TRANSACTION_COMPLETE.
   */
  if(setup_required)
  {
    if (SETUP_SUCCESS == do_aci_setup(&aci_state))
    {
      setup_required = false;
    }
  }


}

/**************************************************************************/
/*!
    Configures the nRF8001 and starts advertising the UART Service
    
    @param[in]  advTimeout  
                The advertising timeout in seconds (0 = infinite advertising)
    @param[in]  advInterval
                The delay between advertising packets in 0.625ms units
*/
/**************************************************************************/
bool Adafruit_BLE_UART::begin(uint16_t advTimeout, uint16_t advInterval) 
{
  Serial.println("Initializing UART service");
  /* Store the advertising timeout and interval */
  adv_timeout = advTimeout;   /* ToDo: Check range! */
  adv_interval = advInterval; /* ToDo: Check range! */
  
  /* Setup the service data from nRFGo Studio (services.h) */
  if (NULL != services_pipe_type_mapping)
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
  }
  else
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
  }
  aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
  aci_state.aci_setup_info.setup_msgs         = (hal_aci_data_t*)setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;

 /*
  Tell the ACI library, the MCU to nRF8001 pin connections.
  The Active pin is optional and can be marked UNUSED
  */
  aci_state.aci_pins.board_name = BOARD_DEFAULT; //See board.h for details REDBEARLAB_SHIELD_V1_1 or BOARD_DEFAULT
  aci_state.aci_pins.reqn_pin   = _REQ; //SS for Nordic board, 9 for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.rdyn_pin   = _RDY; //3 for Nordic board, 8 for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.mosi_pin   = MOSI;
  aci_state.aci_pins.miso_pin   = MISO;
  aci_state.aci_pins.sck_pin    = SCK;

  aci_state.aci_pins.spi_clock_divider      = SPI_CLOCK_DIV8;//SPI_CLOCK_DIV8  = 2MHz SPI speed
                                                             //SPI_CLOCK_DIV16 = 1MHz SPI speed
  
  aci_state.aci_pins.reset_pin              = _RST; //4 for Nordic board, UNUSED for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.active_pin             = UNUSED;
  aci_state.aci_pins.optional_chip_sel_pin  = UNUSED;

  aci_state.aci_pins.interface_is_interrupt = false; //Interrupts still not available in Chipkit
  aci_state.aci_pins.interrupt_number       = 1;

  /* Pass the service data into the appropriate struct in the ACI */
  lib_aci_init(&aci_state, debugMode);

  /* ToDo: Check for chip ID to make sure we're connected! */
  
  return true;
}
