/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT. 
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRENTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision: 4808 $
 */ 
/* Attention! 
*  To maintain compliance with Nordic Semiconductor ASA�s Bluetooth profile 
*  qualification listings, this section of source code must not be modified.
*/

/** @file
  @brief Implementation of the ACI library.
 */

#include "hal_platform.h"
#include "aci.h"
#include "aci_cmds.h"
#include "aci_evts.h"
#include "aci_protocol_defines.h"
#include "acilib_defs.h"
#include "acilib_if.h"
#include "hal/hal_aci_tl.h"
#include "lib_aci.h"




#define LIB_ACI_DEFAULT_CREDIT_NUMBER   1


static services_pipe_type_mapping_t * p_services_pipe_type_map;
static uint8_t                        pipe_count;
static hal_aci_data_t *               p_setup_msgs;
static uint8_t                        setup_msgs_count;
                  


static hal_aci_data_t  msg_to_send;

static hal_aci_data_t  *p_rcvd_evt;

static uint8_t cur_transaction_cmd;
static uint8_t memorized_rcvd_cmd_opcode;
static uint8_t memorized_transaction_cmd_opcode;

static bool is_cmd_response_expected;

static uint16_t cx_rf_interval;  // rf_interval = cx_rf_interval * 1.25 ms Range:0x0006 to 0x0C80
static uint16_t current_slave_latency;


static bool is_request_operation_pending;
static bool is_indicate_operation_pending;
static bool is_open_remote_pipe_pending;
static bool is_close_remote_pipe_pending;

static uint8_t request_operation_pipe = 0;
static uint8_t indicate_operation_pipe = 0;


// The following structure (aci_cmd_params_open_adv_pipe) will be used to store the complete command 
// including the pipes to be opened. 
static aci_cmd_params_open_adv_pipe_t aci_cmd_params_open_adv_pipe; 

static uint8_t cur_error_code;

bool lib_aci_is_pipe_available(aci_state_t *aci_stat, uint8_t pipe)
{
  uint8_t byte_idx;

  byte_idx = pipe / 8;
  if (aci_stat->pipes_open_bitmap[byte_idx] & (0x01 << (pipe % 8)))
  {
    return(true);
  }
  return(false);
}


bool lib_aci_is_pipe_closed(aci_state_t *aci_stat, uint8_t pipe)
{
  uint8_t byte_idx;

  byte_idx = pipe / 8;
  if (aci_stat->pipes_closed_bitmap[byte_idx] & (0x01 << (pipe % 8)))
  {
    return(true);
  }
  return(false);
}


bool lib_aci_is_discovery_finished(aci_state_t *aci_stat)
{
  return(aci_stat->pipes_open_bitmap[0]&0x01);
}




void lib_aci_init(aci_state_t *aci_stat)
{
  uint8_t i;

  for (i = 0; i < PIPES_ARRAY_SIZE; i++)
  {
    aci_stat->pipes_open_bitmap[i]          = 0;
    aci_stat->pipes_closed_bitmap[i]        = 0;
    aci_cmd_params_open_adv_pipe.pipes[i]   = 0;
  }
  



  is_request_operation_pending     = false;
  is_indicate_operation_pending    = false; 
  is_open_remote_pipe_pending      = false;
  is_close_remote_pipe_pending     = false;
  cur_transaction_cmd              = ACI_CMD_INVALID;
  memorized_rcvd_cmd_opcode        = ACI_CMD_INVALID;
  memorized_transaction_cmd_opcode = ACI_CMD_INVALID;
  cx_rf_interval                   = 0;
  current_slave_latency            = 0;
  request_operation_pipe           = 0;
  indicate_operation_pipe          = 0;
  cur_error_code                   = 0;
  p_rcvd_evt                       = NULL;
  
  p_services_pipe_type_map = aci_stat->aci_setup_info.services_pipe_type_mapping;
  pipe_count               = aci_stat->aci_setup_info.number_of_pipes;
  p_setup_msgs             = aci_stat->aci_setup_info.setup_msgs;
  setup_msgs_count         = aci_stat->aci_setup_info.num_setup_msgs;
  
  hal_aci_tl_init();
}


uint8_t lib_aci_get_nb_available_credits(aci_state_t *aci_stat)
{
  return aci_stat->data_credit_available;
}

uint16_t lib_aci_get_cx_interval_ms(aci_state_t *aci_stat)
{
  uint32_t cx_rf_interval_ms_32bits;
  uint16_t cx_rf_interval_ms;
  
  cx_rf_interval_ms_32bits  = aci_stat->connection_interval;
  cx_rf_interval_ms_32bits *= 125;                      // the connection interval is given in multiple of 0.125 milliseconds
  cx_rf_interval_ms         = cx_rf_interval_ms_32bits / 100;
  
  return cx_rf_interval_ms;
}


uint16_t lib_aci_get_cx_interval(aci_state_t *aci_stat)
{
  return aci_stat->connection_interval;
}


uint16_t lib_aci_get_slave_latency(aci_state_t *aci_stat)
{
  return aci_stat->slave_latency;
}


bool lib_aci_set_app_latency(uint16_t latency, aci_app_latency_mode_t latency_mode)
{
  aci_cmd_params_set_app_latency_t aci_set_app_latency;
  
  aci_set_app_latency.mode    = latency_mode;
  aci_set_app_latency.latency = latency;  
  acil_encode_cmd_set_app_latency(&(msg_to_send.buffer[0]), &aci_set_app_latency);
  
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_test(aci_test_mode_change_t enter_exit_test_mode)
{
  aci_cmd_params_test_t aci_cmd_params_test;
  aci_cmd_params_test.test_mode_change = enter_exit_test_mode;
  acil_encode_cmd_set_test_mode(&(msg_to_send.buffer[0]), &aci_cmd_params_test);
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_sleep()
{
  acil_encode_cmd_sleep(&(msg_to_send.buffer[0]));
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_radio_reset()
{
  acil_encode_baseband_reset(&(msg_to_send.buffer[0]));
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_direct_connect()
{
  acil_encode_direct_connect(&(msg_to_send.buffer[0]));
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_device_version()
{
  acil_encode_cmd_get_device_version(&(msg_to_send.buffer[0]));
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_set_local_data(aci_state_t *aci_stat, uint8_t pipe, uint8_t *p_value, uint8_t size)
{
  aci_cmd_params_set_local_data_t aci_cmd_params_set_local_data;
  
  if ((p_services_pipe_type_map[pipe-1].location != ACI_STORE_LOCAL)
      ||
      (size > ACI_PIPE_TX_DATA_MAX_LEN))
  {
    return false;
  }

  aci_cmd_params_set_local_data.tx_data.pipe_number = pipe;
  memcpy(&(aci_cmd_params_set_local_data.tx_data.aci_data[0]), p_value, size);
  acil_encode_cmd_set_local_data(&(msg_to_send.buffer[0]), &aci_cmd_params_set_local_data, size);
  return hal_aci_tl_send(&msg_to_send);
}

bool lib_aci_connect(uint16_t run_timeout, uint16_t adv_interval)
{
  aci_cmd_params_connect_t aci_cmd_params_connect;
  aci_cmd_params_connect.timeout      = run_timeout;
  aci_cmd_params_connect.adv_interval = adv_interval;
  acil_encode_cmd_connect(&(msg_to_send.buffer[0]), &aci_cmd_params_connect);
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_disconnect(aci_state_t *aci_stat, aci_disconnect_reason_t reason)
{
  bool ret_val;
  uint8_t i;
  aci_cmd_params_disconnect_t aci_cmd_params_disconnect;
  aci_cmd_params_disconnect.reason = reason;
  acil_encode_cmd_disconnect(&(msg_to_send.buffer[0]), &aci_cmd_params_disconnect);
  ret_val = hal_aci_tl_send(&msg_to_send);
  // If we have actually sent the disconnect
  if (ret_val)
  {
    // Update pipes immediately so that while the disconnect is happening,
    // the application can't attempt sending another message
    // If the application sends another message before we updated this
    //    a ACI Pipe Error Event will be received from nRF8001
    for (i=0; i < PIPES_ARRAY_SIZE; i++)
    {
      aci_stat->pipes_open_bitmap[i] = 0;
      aci_stat->pipes_closed_bitmap[i] = 0;
    }
  }
  return ret_val;
}


bool lib_aci_bond(uint16_t run_timeout, uint16_t adv_interval)
{
  aci_cmd_params_bond_t aci_cmd_params_bond;
  aci_cmd_params_bond.timeout = run_timeout;
  aci_cmd_params_bond.adv_interval = adv_interval;
  acil_encode_cmd_bond(&(msg_to_send.buffer[0]), &aci_cmd_params_bond);
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_wakeup()
{
  acil_encode_cmd_wakeup(&(msg_to_send.buffer[0]));
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_set_tx_power(aci_device_output_power_t tx_power)
{
  aci_cmd_params_set_tx_power_t aci_cmd_params_set_tx_power;
  aci_cmd_params_set_tx_power.device_power = tx_power;
  acil_encode_cmd_set_radio_tx_power(&(msg_to_send.buffer[0]), &aci_cmd_params_set_tx_power);
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_get_address()
{
  acil_encode_cmd_get_address(&(msg_to_send.buffer[0]));
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_get_temperature()
{
  acil_encode_cmd_temparature(&(msg_to_send.buffer[0]));
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_get_battery_level()
{
  acil_encode_cmd_battery_level(&(msg_to_send.buffer[0]));
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_send_data(uint8_t pipe, uint8_t *p_value, uint8_t size)
{
  bool ret_val = false;
  aci_cmd_params_send_data_t aci_cmd_params_send_data;

  
  if(!((p_services_pipe_type_map[pipe-1].pipe_type == ACI_TX) ||
      (p_services_pipe_type_map[pipe-1].pipe_type == ACI_TX_ACK)))
  {
    return false;
  }

  if (size > ACI_PIPE_TX_DATA_MAX_LEN)
  {
    return false;
  }
  {
      aci_cmd_params_send_data.tx_data.pipe_number = pipe;
      memcpy(&(aci_cmd_params_send_data.tx_data.aci_data[0]), p_value, size);
      acil_encode_cmd_send_data(&(msg_to_send.buffer[0]), &aci_cmd_params_send_data, size);
      is_cmd_response_expected = false;
      ret_val = hal_aci_tl_send(&msg_to_send);          
  }
  return ret_val;
}


bool lib_aci_request_data(aci_state_t *aci_stat, uint8_t pipe)
{
  bool ret_val = false;
  aci_cmd_params_request_data_t aci_cmd_params_request_data;

  if(!((p_services_pipe_type_map[pipe-1].location == ACI_STORE_REMOTE)&&(p_services_pipe_type_map[pipe-1].pipe_type == ACI_RX_REQ)))
  {
    return false;
  }


  {

    {



      aci_cmd_params_request_data.pipe_number = pipe;
      acil_encode_cmd_request_data(&(msg_to_send.buffer[0]), &aci_cmd_params_request_data);

      ret_val = hal_aci_tl_send(&msg_to_send);
    }
  }
  return ret_val;
}


bool lib_aci_change_timing(uint16_t minimun_cx_interval, uint16_t maximum_cx_interval, uint16_t slave_latency, uint16_t timeout)
{
  aci_cmd_params_change_timing_t aci_cmd_params_change_timing;
  aci_cmd_params_change_timing.conn_params.min_conn_interval = minimun_cx_interval;
  aci_cmd_params_change_timing.conn_params.max_conn_interval = maximum_cx_interval;
  aci_cmd_params_change_timing.conn_params.slave_latency     = slave_latency;    
  aci_cmd_params_change_timing.conn_params.timeout_mult      = timeout;     
  acil_encode_cmd_change_timing_req(&(msg_to_send.buffer[0]), &aci_cmd_params_change_timing);
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_change_timing_GAP_PPCP()
{
  acil_encode_cmd_change_timing_req_GAP_PPCP(&(msg_to_send.buffer[0]));
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_open_remote_pipe(aci_state_t *aci_stat, uint8_t pipe)
{
  bool ret_val = false;
  aci_cmd_params_open_remote_pipe_t aci_cmd_params_open_remote_pipe;

  if(!((p_services_pipe_type_map[pipe-1].location == ACI_STORE_REMOTE)&&
                ((p_services_pipe_type_map[pipe-1].pipe_type == ACI_RX)||
                (p_services_pipe_type_map[pipe-1].pipe_type == ACI_RX_ACK_AUTO)||
                (p_services_pipe_type_map[pipe-1].pipe_type == ACI_RX_ACK))))
  {
    return false;
  }

  
  {

    is_request_operation_pending = true;
    is_open_remote_pipe_pending = true;
    request_operation_pipe = pipe;
    aci_cmd_params_open_remote_pipe.pipe_number = pipe;
    acil_encode_cmd_open_remote_pipe(&(msg_to_send.buffer[0]), &aci_cmd_params_open_remote_pipe);
    ret_val = hal_aci_tl_send(&msg_to_send);
  }
  return ret_val;
}


bool lib_aci_close_remote_pipe(aci_state_t *aci_stat, uint8_t pipe)
{
  bool ret_val = false;
  aci_cmd_params_close_remote_pipe_t aci_cmd_params_close_remote_pipe;

  if((p_services_pipe_type_map[pipe-1].location == ACI_STORE_REMOTE)&&
        ((p_services_pipe_type_map[pipe-1].pipe_type == ACI_RX)||
         (p_services_pipe_type_map[pipe-1].pipe_type == ACI_RX_ACK_AUTO)||
         (p_services_pipe_type_map[pipe-1].pipe_type == ACI_RX_ACK)))
  {
    return false;
  }  


  {

    is_request_operation_pending = true;
    is_close_remote_pipe_pending = true;
    request_operation_pipe = pipe;
    aci_cmd_params_close_remote_pipe.pipe_number = pipe;
    acil_encode_cmd_close_remote_pipe(&(msg_to_send.buffer[0]), &aci_cmd_params_close_remote_pipe);
    ret_val = hal_aci_tl_send(&msg_to_send);
  }
  return ret_val;
}


bool lib_aci_set_key(aci_key_type_t key_rsp_type, uint8_t *key, uint8_t len)
{
  aci_cmd_params_set_key_t aci_cmd_params_set_key;
  aci_cmd_params_set_key.key_type = key_rsp_type;
  memcpy((uint8_t*)&(aci_cmd_params_set_key.key), key, len);
  acil_encode_cmd_set_key(&(msg_to_send.buffer[0]), &aci_cmd_params_set_key);
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_echo_msg(uint8_t msg_size, uint8_t *p_msg_data)
{
  aci_cmd_params_echo_t aci_cmd_params_echo;
  if(msg_size > (ACI_ECHO_DATA_MAX_LEN))
  {
    return false;
  }

  if (msg_size > (ACI_ECHO_DATA_MAX_LEN))
  {
    msg_size = ACI_ECHO_DATA_MAX_LEN;
  }

  memcpy(&(aci_cmd_params_echo.echo_data[0]), p_msg_data, msg_size);
  acil_encode_cmd_echo_msg(&(msg_to_send.buffer[0]), &aci_cmd_params_echo, msg_size);

  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_bond_request()
{
  acil_encode_cmd_bond_security_request(&(msg_to_send.buffer[0]));
  return hal_aci_tl_send(&msg_to_send);
}

bool lib_aci_event_get(aci_state_t *aci_stat, hal_aci_evt_t *p_aci_evt_data)
{
  bool status;
  status = hal_aci_tl_event_get((hal_aci_data_t *)p_aci_evt_data);
  
  /**
  Update the state of the ACI witn the 
  ACI Events -> Pipe Status, Disconnected, Connected, Bond Status, Pipe Error
  */
  {
    aci_evt_t * aci_evt;
    
    aci_evt = &p_aci_evt_data->evt;  
    
    switch(aci_evt->evt_opcode)
    {
        case ACI_EVT_PIPE_STATUS:
            {
                uint8_t i=0;
                
                for (i=0; i < PIPES_ARRAY_SIZE; i++)
                {
                  aci_stat->pipes_open_bitmap[i]   = aci_evt->params.pipe_status.pipes_open_bitmap[i];
                  aci_stat->pipes_closed_bitmap[i] = aci_evt->params.pipe_status.pipes_closed_bitmap[i];
                }
            }
            break;
        
        case ACI_EVT_DISCONNECTED:
            {
                uint8_t i=0;
                
                for (i=0; i < PIPES_ARRAY_SIZE; i++)
                {
                  aci_stat->pipes_open_bitmap[i] = 0;
                  aci_stat->pipes_closed_bitmap[i] = 0;
                }
                aci_stat->confirmation_pending = false;
                aci_stat->data_credit_available = aci_stat->data_credit_total;
                
            }
            break;
            
        case ACI_EVT_TIMING:            
                aci_stat->connection_interval = aci_evt->params.timing.conn_rf_interval;
                aci_stat->slave_latency       = aci_evt->params.timing.conn_slave_rf_latency;
                aci_stat->supervision_timeout = aci_evt->params.timing.conn_rf_timeout;
            break;
        
        
    }
    
  }
  
  return status;
}


bool lib_aci_send_ack(aci_state_t *aci_stat, const uint8_t pipe)
{
  bool ret_val = false;
  {
    acil_encode_cmd_send_data_ack(&(msg_to_send.buffer[0]), pipe);
    is_cmd_response_expected = false;
    ret_val = hal_aci_tl_send(&msg_to_send);
  }
  return ret_val;
}


bool lib_aci_send_nack(aci_state_t *aci_stat, const uint8_t pipe, const uint8_t error_code)
{
  bool ret_val = false;
  
  {
    is_cmd_response_expected = false;
    acil_encode_cmd_send_data_nack(&(msg_to_send.buffer[0]), pipe, error_code);
    ret_val = hal_aci_tl_send(&msg_to_send);
  }
  return ret_val;
}


bool lib_aci_broadcast(const uint16_t timeout, const uint16_t adv_interval)
{
  aci_cmd_params_broadcast_t aci_cmd_params_broadcast;
  if (timeout > 16383)
  {
    return false;
  }  
  
  // The adv_interval should be between 160 and 16384 (which translates to the advertisement 
  // interval values 100 ms and 10.24 s.
  if ((160 > adv_interval) || (adv_interval > 16384))
  {
    return false;
  }

  aci_cmd_params_broadcast.timeout = timeout;
  aci_cmd_params_broadcast.adv_interval = adv_interval;
  acil_encode_cmd_broadcast(&(msg_to_send.buffer[0]), &aci_cmd_params_broadcast);
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_open_adv_pipes(const uint8_t * const adv_service_data_pipes)
{
  uint8_t i;
    
  for (i = 0; i < PIPES_ARRAY_SIZE; i++)
  {
    aci_cmd_params_open_adv_pipe.pipes[i] = adv_service_data_pipes[i];
  }

  acil_encode_cmd_open_adv_pipes(&(msg_to_send.buffer[0]), &aci_cmd_params_open_adv_pipe);
  return hal_aci_tl_send(&msg_to_send);
}

bool lib_aci_open_adv_pipe(const uint8_t pipe)
{
  uint8_t byte_idx = pipe / 8;
  
  aci_cmd_params_open_adv_pipe.pipes[byte_idx] |= (0x01 << (pipe % 8));
  acil_encode_cmd_open_adv_pipes(&(msg_to_send.buffer[0]), &aci_cmd_params_open_adv_pipe);
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_read_dynamic_data()
{
  acil_encode_cmd_read_dynamic_data(&(msg_to_send.buffer[0]));
  return hal_aci_tl_send(&msg_to_send);
}


bool lib_aci_write_dynamic_data(uint8_t sequence_number, uint8_t* dynamic_data, uint8_t length)
{
  acil_encode_cmd_write_dynamic_data(&(msg_to_send.buffer[0]), sequence_number, dynamic_data, length);
  return hal_aci_tl_send(&msg_to_send);
}

bool lib_aci_dtm_command(uint8_t dtm_command_msbyte, uint8_t dtm_command_lsbyte)
{
  aci_cmd_params_dtm_cmd_t aci_cmd_params_dtm_cmd;
  aci_cmd_params_dtm_cmd.cmd_msb = dtm_command_msbyte;
  aci_cmd_params_dtm_cmd.cmd_lsb = dtm_command_lsbyte;
  acil_encode_cmd_dtm_cmd(&(msg_to_send.buffer[0]), &aci_cmd_params_dtm_cmd);
  return hal_aci_tl_send(&msg_to_send);
}

void lib_aci_flush(void)
{
  m_aci_q_flush();
}

void lib_aci_debug_print(bool enable)
{
  hal_aci_debug_print(enable);
}

