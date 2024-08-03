#ifndef BPHERO_UWB_H
#define BPHERO_UWB_H

#include "frame_header.h"
#include "common_header.h"

#include "deca_types.h"
#include "deca_regs.h"

#define RX_NODE
//#define TX_NODE

#ifdef RX_NODE
#define SHORT_ADDR 0x0002
#endif

#ifdef TX_NODE
#define SHORT_ADDR 0x0001
#define LCD_ENABLE
#endif

#define ANCHOR_NUM 4
#define ANCHOR_ID 3

#define ANCHOR_LISTEN 0
#define ANCHOR_SEND 2

#define RX_ANT_DLY 0
#define TX_ANT_DLY 32880

#define SINGLE_LEN 13
#define POA_LEN 4

#define END_LEN 0

#define ALL_MSG_COMMON_LEN 10

#define ALL_MSG_SN_IDX 2
#define SENDING_TX_TS_IDX 10

#define DELAY_TIME 600
#define DELAY_TIME_TURN 600
#define RX_AFTER_TX_DELAY 450
#define RX_TIMEOUT 2500

#define CIR_LEN 3

#define NET_PANID 0xF0F2

extern int psduLength ;
extern srd_msg_dsss msg_f_send ;
extern srd_msg_dsss msg_f_send2 ;

#ifndef SPEED_OF_LIGHT
#define SPEED_OF_LIGHT      (299702547.0) 
#endif

#ifndef FRAME_LEN_MAX
#define FRAME_LEN_MAX 127
#endif

#define UUS_TO_DWT_TIME 65536

#define PRE_TIMEOUT 0

extern uint8 rx_buffer[FRAME_LEN_MAX];
extern uint32 status_reg;
extern uint16 frame_len ;
extern void BPhero_UWB_Message_Init(void);
extern float dwGetReceivePower(void);

extern dwt_config_t config;

#endif
