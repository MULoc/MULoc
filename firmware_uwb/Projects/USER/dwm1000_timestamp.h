/*! ----------------------------------------------------------------------------
 * @file	dwm1000_timestamp.h
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */


#ifndef DWM1000_TIMESTAMP_H
#define DWM1000_TIMESTAMP_H
#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"
//#include "deca_sleep.h"
#include "port.h"
#include "common_header.h"


extern uint64 get_tx_timestamp_u64(void);
extern uint64 get_rx_timestamp_u64(void);
extern uint64 get_cur_timestamp_u64(void);

extern void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
extern void final_msg_set_ts(uint8 *ts_field, uint64 ts);

extern void final2_msg_set_fp(uint8 *fp_field, uint16 fp_index);
extern void final2_msg_get_fp(const uint8 *fp_field, uint16 *fp_index);

#endif
