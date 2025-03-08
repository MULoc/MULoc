#ifndef BPHERO_UWB_H
#define BPHERO_UWB_H

#include "frame_header.h"
#include "common_header.h"

#include "deca_types.h"
#include "deca_regs.h"

// Device configuration
#define RX_NODE
//#define TX_NODE

#ifdef RX_NODE
#define SHORT_ADDR 0x0002
#endif

#ifdef TX_NODE
#define SHORT_ADDR 0x0001
#define LCD_ENABLE
#endif

// Anchor configuration
// Number of anchors used in the system
#define ANCHOR_NUM 4
// Anchor ID of the current anchor
#define ANCHOR_ID 3

#define ANCHOR_LISTEN 0
#define ANCHOR_SEND 2

#define RX_ANT_DLY 0
#define TX_ANT_DLY 32880


/* Payload format (CIR and receiving timestamps of messages from other anchors)

|---CIR(4 byte)---|--RxTime(5 byte)--|******|---CIR(4 byte)---|--RxTime(5 byte)--|-IDX(1 byte)-|
|---------------------------13 byte * (Anchor Number-1)--------------------------|----1 byte---| 
*/
#define SINGLE_LEN 13
#define POA_LEN 4

#define END_LEN 0

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define SENDING_TX_TS_IDX 10

/* Delay between frames, in UWB microseconds. */
#define DELAY_TIME 600
#define DELAY_TIME_TURN 600
#define RX_AFTER_TX_DELAY 450
#define RX_TIMEOUT 2500

/* Length of CIR read from the accumulator buffer*/
#define CIR_LEN 3

#define NET_PANID 0xF0F2

extern int psduLength ;
extern srd_msg_dsss msg_f_send ;
extern srd_msg_dsss msg_f_send2 ;

#ifndef SPEED_OF_LIGHT
#define SPEED_OF_LIGHT      (299702547.0)  // in m/s in air
#endif

#ifndef FRAME_LEN_MAX
#define FRAME_LEN_MAX 127
#endif

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 ? and 1 ? = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

#define PRE_TIMEOUT 0

extern uint8 rx_buffer[FRAME_LEN_MAX];
extern uint32 status_reg;
extern uint16 frame_len ;
extern void BPhero_UWB_Message_Init(void);
extern float dwGetReceivePower(void);

extern dwt_config_t config;

#endif
