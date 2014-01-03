/*********************************************************************
This is a library for our nRF8001 Bluetooth Low Energy Breakout

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/

These displays use SPI to communicate, 4 or 5 pins are required to  
interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Kevin Townsend/KTOWN  for Adafruit Industries.  
MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/
#include <SPI.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include <ble_system.h>
#include <lib_aci.h>
#include <aci_setup.h>
#include "uart/services.h"

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
static hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;

static struct aci_state_t aci_state;            /* ACI state data */
static hal_aci_evt_t  aci_data;                 /* Command buffer */
static bool timing_change_done = false;

static uint8_t uart_buffer[20];
static uint8_t uart_buffer_len = 0;

/**************************************************************************/
/*!
    Constructor for the UART service
*/
/**************************************************************************/
Adafruit_BLE_UART::Adafruit_BLE_UART(aci_callback aciEvent, rx_callback rxEvent)
{
  aci_event = aciEvent;
  rx_event = rxEvent;
}

/**************************************************************************/
/*!
    Transmits data out via the TX characteristic (when available)
*/
/**************************************************************************/
void Adafruit_BLE_UART::write(uint8_t * buffer, uint8_t len)
{
  /* ToDo: handle packets > 20 bytes in multiple transmits! */
  if (len > 20)
  {
    len = 20;
  }
  
  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX))
  {
    lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, buffer, len);
    aci_state.data_credit_available--;
  }
}

/**************************************************************************/
/*!
    Handles low level ACI events, and passes them up to an application
    level callback when appropriate
*/
/**************************************************************************/
void Adafruit_BLE_UART::pollACI()
{
  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data))
  {
    aci_evt_t * aci_evt;
    
    aci_evt = &aci_data.evt;    
    switch(aci_evt->evt_opcode)
    {
        /* As soon as you reset the nRF8001 you will get an ACI Device Started Event */
        case ACI_EVT_DEVICE_STARTED:
        {          
          aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
          switch(aci_evt->params.device_started.device_mode)
          {
            case ACI_DEVICE_SETUP:
            /* Device is in setup mode! */
            if (ACI_STATUS_TRANSACTION_COMPLETE != do_aci_setup(&aci_state))
            {
              Serial.println(F("Error in ACI Setup"));
            }
            break;
            
            case ACI_DEVICE_STANDBY:
              /* Start advertising ... first value is advertising time in seconds, the */
              /* second value is the advertising interval in 0.625ms units */
              lib_aci_connect(adv_timeout, adv_interval);
              aci_event(ACI_EVT_DEVICE_STARTED);
              break;
          }
        }
        break;
        
      case ACI_EVT_CMD_RSP:
        /* If an ACI command response event comes with an error -> stop */
        if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
        {
          // ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
          // TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
          // all other ACI commands will have status code of ACI_STATUS_SUCCESS for a successful command
          Serial.print(F("ACI Command "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
          Serial.println(F("Evt Cmd respone: Error. Arduino is in an while(1); loop"));
          while (1);
        }
        if (ACI_CMD_GET_DEVICE_VERSION == aci_evt->params.cmd_rsp.cmd_opcode)
        {
          // Store the version and configuration information of the nRF8001 in the Hardware Revision String Characteristic
          lib_aci_set_local_data(&aci_state, PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET, 
            (uint8_t *)&(aci_evt->params.cmd_rsp.params.get_device_version), sizeof(aci_evt_cmd_rsp_params_get_device_version_t));
        }        
        break;
        
      case ACI_EVT_CONNECTED:
        aci_state.data_credit_available = aci_state.data_credit_total;
        /* Get the device version of the nRF8001 and store it in the Hardware Revision String */
        lib_aci_device_version();
        aci_event(ACI_EVT_CONNECTED);
        break;
        
      case ACI_EVT_PIPE_STATUS:
        if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) && (false == timing_change_done))
        {
          lib_aci_change_timing_GAP_PPCP(); // change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP. 
                                            // Used to increase or decrease bandwidth
          timing_change_done = true;
        }
        break;
        
      case ACI_EVT_TIMING:
        /* Link connection interval changed */
        break;
        
      case ACI_EVT_DISCONNECTED:
        /* Restart advertising ... first value is advertising time in seconds, the */
        /* second value is the advertising interval in 0.625ms units */
        aci_event(ACI_EVT_DISCONNECTED);
        lib_aci_connect(adv_timeout, adv_interval);
        aci_event(ACI_EVT_DEVICE_STARTED);
        break;
        
      case ACI_EVT_DATA_RECEIVED:
        for(int i=0; i<aci_evt->len - 2; i++)
        {
          /* Fill uart_buffer with incoming data */
          uart_buffer[i] = aci_evt->params.data_received.rx_data.aci_data[i];
        }
        /* Set the buffer len */
        uart_buffer_len = aci_evt->len - 2;
        rx_event(uart_buffer, uart_buffer_len);
        break;
   
      case ACI_EVT_DATA_CREDIT:
        aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
        break;
      
      case ACI_EVT_PIPE_ERROR:
        /* See the appendix in the nRF8001 Product Specication for details on the error codes */
        Serial.print(F("ACI Evt Pipe Error: Pipe #:"));
        Serial.print(aci_evt->params.pipe_error.pipe_number, DEC);
        Serial.print(F("  Pipe Error Code: 0x"));
        Serial.println(aci_evt->params.pipe_error.error_code, HEX);

        /* Increment the credit available as the data packet was not sent */
        aci_state.data_credit_available++;
        break;
    }
  }
  else
  {
    // Serial.println(F("No ACI Events available"));
    // No event in the ACI Event queue and if there is no event in the ACI command queue the arduino can go to sleep
    // Arduino can go to sleep now
    // Wakeup from sleep from the RDYN line
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
  aci_state.aci_setup_info.setup_msgs         = setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;

  /* Pass the service data into the appropriate struct in the ACI */
  lib_aci_init(&aci_state);

  /* ToDo: Check for chip ID to make sure we're connected! */
  
  return true;
}
